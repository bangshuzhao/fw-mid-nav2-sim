# fw-mid-nav2-sim
FW‑mid 四轮转向机器人仿真：基于 ROS 2 Humble、Gazebo Fortress，集成 Nav2 导航栈、三维激光雷达体素代价地图与 EKF 扩展卡尔曼滤波。2026-11-22 宁夏.银川

## 第九步 · 从规划到行人横穿停车

在已安装 ROS 2 Humble、Gazebo Fortress 和工作区构建依赖的 Ubuntu 22.04 上，从仓库根目录初始化源码并构建。首次构建前，将仓库附带的 MPPI 补丁应用到 Navigation2 子模块。补丁已经应用的工作区可以直接构建。

```bash
git submodule update --init --recursive
git -C src/navcode/navigation2 apply "$PWD/patches/navigation2-humble-mppi-feasible-control.patch"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fw_mid_sim \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

在两个已加载上述 ROS 和工作区环境的终端中分别启动仿真和导航。

```bash
ros2 launch fw_mid_sim sim.launch.py scene:=narrow spawn_x:=-4.8 gui:=false
```

```bash
ros2 launch fw_mid_sim navigate_step9.launch.py stage:=traffic \
  map:="$PWD/src/simcode/fw_mid_sim/maps/narrow_office.yaml"
```

等导航终端出现 `Managed nodes are active`，在第三个已加载环境的终端发送目标。

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.80, y: 0.0}, orientation: {w: 1.0}}}}'
```

中央行人横穿时，导航终端依次打印 `Robot to stop due to CrossingStop polygon`、`Robot to continue normal operation` 和 `Goal succeeded`。
