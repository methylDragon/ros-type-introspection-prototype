# ros-type-introspection-prototype

Reference implementation prototype for the run-time type reflection feature in [[REP-2011] Evolving Message Types](https://github.com/ros-infrastructure/rep/pull/358).

## Packages

This repo contains packages that support or implement run-time type reflection.

### `evolving_serialization_lib`

The core library, including:

- Utilities for loading and manipulating yaml files as messages (for easy mock-ups)
- Type description structs that mimic the type description message defined in the REP
- A polymorphic "evolving type support" (ETS) interface in `C` that downstream middlewares can implement to support run-time type construction and reflection, and associated utilities
- Macro definitions for type codes used by the ETS

### `evolving_serialization_examples`

An examples package...

### And a collection of middleware ETS implementations

e.g. `evolving_fastrtps_c`, a `C++` implementation of the `C` ETS interface, callable by `C`

## Setup

```shell
git clone https://github.com/methylDragon/ros-type-introspection-prototype.git
git module update --init --recursive

cd prototype_ws
colcon build
source install/setup.bash
```

## Notes

This repo uses [a fork of the FastDDS repository](https://github.com/methylDragon/Fast-DDS) with bugfixes that have yet to be released, the relevant `CMakeLists.txt` files have been modified to force the use of the fork's version of FastRTPS, masking the version of FastRTPS installed with ROS 2.

The fixes are just for utility functions that don't affect what is going on the wire and are used just for prototype convenience reasons.