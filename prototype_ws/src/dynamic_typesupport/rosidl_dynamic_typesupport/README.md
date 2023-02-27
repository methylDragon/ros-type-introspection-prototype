# rosidl_dynamic_typesupport

The serialization support interface for run-time type reflection and construction, and utilities that support it.

Properly implemented, a user should be able to, given a serialized buffer, the serialization technology used to serialize it, and the buffer's message/type description, obtain a way to introspect/reflect its contents.

- `yaml_parser.h` and `tree_traverse.h`:
  - Utilities for loading and manipulating yaml files as messages (for easy mock-ups)
  - Utilities for creating and traversing GLib N-ary trees, used with the yaml_parser
- `description.h`:
  - Type description structs that mimic the type description message defined in the REP
  - Utilities for constructing and traversing said structs (including from yaml files!)
- `api/serialization_support.h`
  - A polymorphic "serialization support" interface in `C` that downstream middlewares can implement to support run-time type construction and reflection, and associated utilities
- `types.h`
  - Macro definitions for type codes used by the serialization support

## The Serialization Support Interface

This is supposed to be an interface that is generic enough to support run-time type reflection, and is written in `C` to allow for easy language binding with any downstream libraries that use it.

Its interface is inspired by the [rmw](https://github.com/ros2/rmw) and the [DDS XTypes 1.3 language bindings spec (pg. 175)](https://www.omg.org/spec/DDS-XTypes/1.3/PDF), but is still intended to be generic, and so should support non-DDS middlewares.

The Serialization Support includes, for each supported middleware:

- Getting and setting type members (type description)
- Getting and setting data members based off that type (dynamic data)
- Support for sequences and nested members
- Utilities (e.g. printing of data)

The interface makes heavy use of `void *` casting to abstract away any necessary objects used with each middleware.

### Caveats

- There currently isn't support for bounded sequences/arrays of bounded strings yet (it'll require updates to the REP)

## Type Codes List

This is pulling from the [official ROS2 list of field types](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html#field-types), with extensions for nested types and sequences.

This is a prototype and might not reflect the final ID list!

- For static arrays, add 32 to the ID
- For unbounded dynamic arrays, add 64 to the ID
- For bounded dynamic arrays, add 96 to the ID
- For bounded strings... it's just a type

| Type             | ID   |
| ---------------- | ---- |
| [RESERVED/UNSET] | 0    |
| nested_type      | 1    |
| bool             | 2    |
| byte             | 3    |
| char             | 4    |
| float32          | 5    |
| float64          | 6    |
| int8             | 7    |
| uint8            | 8    |
| int16            | 9    |
| uint16           | 10   |
| int32            | 11   |
| uint32           | 12   |
| int64            | 13   |
| uint64           | 14   |
| string           | 15   |
| wstring          | 16   |
| bounded string   | 17   |
| bounded wstring  | 18   |
