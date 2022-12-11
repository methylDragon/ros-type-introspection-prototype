// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// With code blocks from:
// https://vdna.be/site/index.php/2016/05/google-protobuf-at-run-time-deserialization-example-in-c/

#include <glib.h>
#include <iostream>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/tokenizer.h>

#include <google/protobuf/compiler/parser.h>

#include <evolving_protobuf_c/protogen.h>
#include <evolving_serialization_lib/yaml_parser.h>
#include <evolving_serialization_lib/description.h>

// Lol
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/text_format.h>

using namespace google;
using namespace google::protobuf;


protobuf::Message * create_populated_proto_msg(const protobuf::Message * prototype)
{
  protobuf::Message * mutable_msg = prototype->New();
  if (prototype == NULL) {
    std::cerr << "Failed in prototype_msg->New(); to create mutable message";
    return nullptr;
  }
  const protobuf::Descriptor * message_desc = mutable_msg->GetDescriptor();
  const protobuf::Reflection * reflection = mutable_msg->GetReflection();


  // Set by name or field number (field index is 1-based!!)
  const auto string_field = message_desc->FindFieldByName("string_field");
  const auto bool_static_array_field = message_desc->FindFieldByNumber(2);
  const auto nested_field = message_desc->FindFieldByNumber(3);

  reflection->SetString(mutable_msg, string_field, "ROSCon 2022");
  reflection->AddBool(mutable_msg, bool_static_array_field, true);  // Append to bool arr
  reflection->AddBool(mutable_msg, bool_static_array_field, false);
  reflection->AddBool(mutable_msg, bool_static_array_field, true);


  // Populate inner
  protobuf::Message * inner_msg = reflection->MutableMessage(mutable_msg, nested_field);
  const protobuf::Reflection * inner_reflection = inner_msg->GetReflection();

  const auto inner__doubly_nested_field = inner_msg->GetDescriptor()->FindFieldByNumber(1);

  inner_reflection->AddMessage(inner_msg, inner__doubly_nested_field);
  inner_reflection->AddMessage(inner_msg, inner__doubly_nested_field);


  // Populate inner-inner
  // Repeated array index is 0-based! (Whereas field index is 1-based...)
  protobuf::Message * inner_inner_msg_0 =
    inner_reflection->MutableRepeatedMessage(inner_msg, inner__doubly_nested_field, 0);
  const protobuf::Reflection * inner_inner_0_reflection = inner_inner_msg_0->GetReflection();

  const auto inner_inner__doubly_nested_float32_field =
    inner_inner_msg_0->GetDescriptor()->FindFieldByNumber(1);

  inner_inner_0_reflection->SetFloat(
    inner_inner_msg_0, inner_inner__doubly_nested_float32_field, 0.1);

  return mutable_msg;
}


protobuf::Message * read_proto_msg_from_str(
  const protobuf::Message * prototype, std::string msg_bytes)
{
  protobuf::Message * mutable_msg = prototype->New();
  if (prototype == NULL) {
    std::cerr << "Failed in prototype_msg->New(); to create mutable message";
    return nullptr;
  }

  mutable_msg->ParseFromString(msg_bytes);
  return mutable_msg;
}


void print_proto_msg(const protobuf::Message * msg)
{
  auto printer = protobuf::TextFormat::Printer();
  printer.SetUseShortRepeatedPrimitives(true);
  if (std::string s; printer.PrintToString(*msg, &s)) {
    std::cout << "\n== Message as string ==\n" << s << std::endl;
  } else {
    std::cerr << "Message not valid (partial content: " << msg->ShortDebugString() << ")\n";
  }
}


int main()
{
  const std::string message_type("ExampleMsg");
  const std::string message_file("example_pub_seq_of_nested_msg_msg.yaml");

  const char * main_dir = g_path_get_dirname(__FILE__);
  char * path = g_strjoin("/", main_dir, "..", "..", "msg", message_file.c_str(), NULL);


  // GENERATE AND PARSE .PROTO =====================================================================
  // This generates the contents of a proto file dynamically using protogen!
  char * proto_str = generate_proto_from_yaml_file(path);

  protobuf::io::ArrayInputStream raw_input(proto_str, strlen(proto_str));
  protobuf::io::Tokenizer input(&raw_input, NULL);

  protobuf::FileDescriptorProto file_desc_proto;
  protobuf::compiler::Parser parser;
  if (!parser.Parse(&input, &file_desc_proto)) {
    std::cerr << "Failed to parse .proto definition:" << proto_str;
    return -1;
  }
  file_desc_proto.set_name(message_type);


  // CREATE DESCRIPTOR =============================================================================
  protobuf::DescriptorPool pool;
  const protobuf::FileDescriptor * file_desc = pool.BuildFile(file_desc_proto);
  assert(file_desc != NULL);

  const protobuf::Descriptor * message_desc = file_desc->FindMessageTypeByName(message_type);
  assert(message_desc != NULL);

  protobuf::DynamicMessageFactory factory;
  const protobuf::Message * dyn_type = factory.GetPrototype(message_desc); // immutable
  assert(dyn_type != NULL);


  // CREATE DYNAMIC MESSAGE ========================================================================
  std::cout << "\n== GENERATED PROTO ==\n" << proto_str << std::endl;

  // Create and serialize msg
  std::string ser_msg;
  protobuf::Message * mutable_msg = create_populated_proto_msg(dyn_type);
  mutable_msg->SerializeToString(&ser_msg);

  // Deserialize!
  protobuf::Message * deser_msg = read_proto_msg_from_str(dyn_type, ser_msg);

  assert(mutable_msg->DebugString() == deser_msg->DebugString());
  print_proto_msg(deser_msg);

  return 0;
}
