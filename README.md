# FL SLAM

A fast and light version SLAM could run with low hardware devices

------

### Prerequisites

- Cmake ( Version >= 3.10 )
- GCC  (Version >= 7.5)
- fastddsgen
- For arm version: cross build enviroment
- For x86 version: ubuntu

------

### How to Build

- Clone the repository:

```shell
git clone git@gitlab.ubtrobot.com:mbu/slam/fl-slam.git --recurse-submodules
```

- Build

```shell
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PWD/flslam -DBUILD_TARGET_CPU_TYPE=x86-64
make;make install
```

------

### Plateform

export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/project/fl-slam/build/flslam/lib

------

### How to Run

- Mapping

```shell
./flslam/bin/mapping_node -config_dir=flslam/config -config_mapping=turtlebot3_lds_2d_gazebo.lua  -config_localization=turtlebot3_gazebo_localization.lua -robot_model=flslam/urdf/turtlebot3_burger.urdf.xacro --logtostderr=1
```

- Navigation

```shell
./flslam/bin/move_base_node ./flslam/urdf/turtlebot3_burger.urdf.xacro
```

- Sensor-Bridge

```shell
./flslam/bin/ros_sensor_bridge <Local-Ip> <Remote-Ip>
```

- App-Server

```shell
./flslam/bin/app_server
```
- Trimmer

```shell
./flslam/bin/trimmer_node -DDISPLAY_IMSHOW=OFF
```

export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/project/fl-slam/build/flslam/bin
### Plateform

Only test on Ubuntu 18.04 64-bit and Arm Linux aarch64.
