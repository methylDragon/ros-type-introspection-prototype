# ros-type-introspection-prototype

Reference implementation prototype for the run-time type reflection feature in [[REP-2011] Evolving Message Types](https://github.com/ros-infrastructure/rep/pull/358).

## Setup

```shell
git clone https://github.com/methylDragon/ros-type-introspection-prototype.git
cd ros-type-introspection-prototype
mkdir -p prototype_ws/src/ros2_repos
cd prototype_ws

vcs import src/ros2_repos < src/dynamic_subscription_ws.repos

rosdep install --from-paths src -r -y --ignore-src
colcon build
source install/setup.bash
```

## Demo

In one terminal...

```shell
source install/setup.bash
ros2 run dynamic_typesupport_examples static_pub
```

In another, run the [dynamic subscription](./prototype_ws/src/examples/dynamic_typesupport_examples/src/dynamic_sub_direct.cpp)

```shell
source install/setup.bash
ros2 run dynamic_typesupport_examples dynamic_sub_direct
```
