# Evolving Sub Example

This example includes:
- A normal FastDDS publisher publishing a hardcoded message with a hardcoded message definition
- A FastDDS subscriber using the `serialization_support_fastrtps_c` "plugin" and the `serialization_support_lib` to do evolving deserialization, dynamically creating the type and reading the message at runtime

The example is modified from the [FastDDS DynamicHelloWorldExample](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/DynamicHelloWorldExample), with the subscriber's type construction and deserialization functions replaced with the evolving functions provisioned by the evolving message types libraries.



## Notes

Conveniently, FastDDS supports type-code distribution, so we can make use of that feature to double-check if the evolving message types libraries is replicating the behavior!
