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
ros2 launch fw_mid_sim sim.launch.py scene:=narrow spawn_x:=-4.8 gui:=true
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

## 第十步 · 录制并评测整车导航

重新构建 `fw_mid_sim` 安装新增的场景、地图和规划器配置。评测脚本需要 Python 的 NumPy 和 Shapely，以及工作区的 ROS 2 环境。先结束之前的 Gazebo 和导航进程，再从仓库根目录逐条运行。脚本每次打开 Gazebo GUI，并在实验结束后关闭本轮启动的进程。每次给一个新输出目录。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --base-paths src/simcode/fw_mid_sim --packages-select fw_mid_sim
mkdir -p log/lesson10
bench=src/simcode/tools/navigation_benchmark
"$bench/run.bash" office log/lesson10/my-office
"$bench/run.bash" crossing log/lesson10/my-crossing
"$bench/run.bash" planners log/lesson10/my-planners
```

| 实验 | 输入和输出 |
|---|---|
| `office` | 普通办公室内连续执行东侧、西侧、回访东侧和返回四段。打开 `metrics.csv` 查看各段状态、净空、终点误差、换向、纵向 jerk 和回正。 |
| `crossing` | 复跑 9D 中央横穿。`metrics.csv` 给出行人入道、零速指令、车身停稳、离道及恢复通行的仿真时间，并统计人车净空和接触消息。 |
| `planners` | 给 Hybrid-A* 和 State Lattice 发送相同的四组起终点，每组每种重复三次。`planners.csv` 给出规划状态、路径长度、换向点及耗时。 |

`recording.json.gz` 保存时序数据。`inputs/` 保存本轮地图、场景和参数，`inputs.txt` 保存启动选项、Git 版本和哈希。两份启动日志保留运行过程。办公室 `map` 原点对应 Gazebo 世界 `(-4.50, -1.15)`，窄办公室地图与世界坐标对齐。CSV 的终点误差用 Gazebo 真值计算，Nav2 的到点状态使用自身的定位估计。净空采用二维足迹投影的采样结果。

`office` 使用开阔走廊内的四个巡航目标。`office-reference` 保留参考工程的原始四段目标用于对照。当前参考路线在东侧到西侧的短换向段仍会触发进度检查中止，录制会保留已执行的段。
