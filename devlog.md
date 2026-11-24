# fw-mid-nav2-sim

我在 Ubuntu 22.04 上，用 ROS 2 Humble 和 Gazebo Fortress 从头复现一辆 FW-mid 四轮独立转向、四轮独立驱动小车。整套项目按十步做，目标是把底盘、C32 三维激光雷达、H30 IMU、EKF 和 Nav2 连成一套能建图、定位和导航的办公室仿真。

## 第 1 步 · 下载 Nav2 源码，编译原版 MPPI

刚开始时，我先在自己的工作区编译了一个上游包 `nav2_mppi_controller`。构建后运行 `ros2 pkg prefix nav2_mppi_controller`，看到它指向本项目的 `install/nav2_mppi_controller`，这才确认当前终端能找到自己编译的版本。

| 文件或目录 | 我用它来做什么 |
|---|---|
| `src/navcode/navigation2/` | 从官方仓库引入的源码，固定在 Humble `1.1.20`，提交 `a097086719c88f781aa59788eca29ac6ca5e56db`。以后读到某个函数，可以回到这个版本核对。 |
| [`.gitmodules`](.gitmodules) | 记录 Nav2 子模块的地址和路径。主仓库用一个提交号固定上游版本。 |
| [`.gitignore`](.gitignore) | 忽略本机生成的 `build/`、`install/`、`log/`、Python 缓存和 VS Code 本地设置。 |

这一步我先看 `nav2_core/include/nav2_core/controller.hpp` 的 `setPlan()` 和 `computeVelocityCommands()` 声明，再进 `nav2_mppi_controller/src/controller.cpp` 看实现。`MPPIController::setPlan()` 把全局路径交给 `PathHandler` 保存。`MPPIController::computeVelocityCommands()` 每拍根据当前车姿截取局部参考路径，再让 `Optimizer::evalControl()` 算本拍车速。原来路线一直没变，车也能一边走一边修正，因为每拍读到的位姿、速度和障碍都可能变。

## 第 2 步 · 追到路径从哪来，建自己的 ROS 包

做完第一步，我还想知道谁把路径交给 MPPI。于是从 `nav2_rviz_plugins/src/nav2_panel.cpp` 的 `Nav2Panel::onNewGoal()` 开始追。导航器在 `navigate_to_pose.cpp` 的 `initializeGoalPose()` 把目标写进行为树的 `{goal}`。规划器收到 `ComputePathToPose` 请求后生成路径，行为树再通过 `FollowPath` 把同一条 `{path}` 送给 `controller_server`。最后我在 `controller_server.cpp` 的 `setPlannerPath()` 找到了第一步读过的 `setPlan()` 调用。

这一课我也在 `src/simcode/` 写了自己的第一个包 `fw_mid_sim`。包能被发现以后，第三步写的底盘节点、模型和启动文件就有了固定位置。

| 新文件 | 用处 |
|---|---|
| [`package.xml`](src/simcode/fw_mid_sim/package.xml) | 声明 ROS 包名、版本、构建类型和依赖。第二步先写 `ament_python`，第三步再加入底盘和仿真依赖。 |
| [`setup.py`](src/simcode/fw_mid_sim/setup.py) | 安装 Python 包、包索引和包清单。第三步再把模型、配置、场景、启动文件和 `base_control` 入口加进去。 |
| [`setup.cfg`](src/simcode/fw_mid_sim/setup.cfg) | 指定节点入口安装到 `lib/fw_mid_sim/`，供 `ros2 run` 和 launch 按名称查找。 |
| [`resource/fw_mid_sim`](src/simcode/fw_mid_sim/resource/fw_mid_sim) | 空文件，安装进 ament 索引后，ROS 可以按包名找到安装目录。 |
| [`fw_mid_sim/__init__.py`](src/simcode/fw_mid_sim/fw_mid_sim/__init__.py) | 空文件，标记内层 `fw_mid_sim` 是 Python 包。 |

当 `colcon list` 显示 `fw_mid_sim` 是 `ros.ament_python`，而 `ros2 pkg prefix fw_mid_sim` 指向本项目的安装目录时，这一课就过了。

## 第 3 步 · 算四轮运动学，把车放进 Gazebo

### 3A · 先手算，再用 Python 正反验证

我把车体原点放在四轮几何中心，X 轴向前、Y 轴向左。轴距 `0.40 m`，轮距 `0.42 m`，轮胎半径 `0.12 m`。用 `vx=0.20 m/s`、`wz=0.30 rad/s` 手算左转时，我发现前后轮的转向角符号相反，左侧内轮的转角更大、轮速更低。

[`base_control.py` 的 `wheel_targets()`](src/simcode/fw_mid_sim/fw_mid_sim/base_control.py#L22) 按 FL、FR、RL、RR 的顺序，把车体指令换成四个转向角和四个驱动轮速。[`body_velocity()`](src/simcode/fw_mid_sim/fw_mid_sim/base_control.py#L51) 用轮角和轮速反算 `vx`、`vy`、`wz`。直行、左转、右转和沿同一圆弧倒车四组都做了往返验算。

### 3B · 给八个数字找到对应的八个关节

| 新文件 | 用处 |
|---|---|
| [`config/geometry.yaml`](src/simcode/fw_mid_sim/config/geometry.yaml) | 保存半轴距、半轮距、轮胎半径和宽度。Xacro 用它摆放四轮，控制代码用它计算轮速和里程计。 |
| [`urdf/fw_mid.urdf.xacro`](src/simcode/fw_mid_sim/urdf/fw_mid.urdf.xacro) | 描述车身、转向节和轮胎。`wheel` 宏展开四个轮组，每组有一个绕 Z 轴的转向关节和一个绕自身 Y 轴的驱动关节。`wheel_interfaces` 宏为这八个关节声明 ROS 2 控制和反馈接口。 |
| [`config/controllers.yaml`](src/simcode/fw_mid_sim/config/controllers.yaml) | 将转向关节接到 `position` 控制器，将驱动关节接到 `velocity` 控制器。两组都按 FL、FR、RL、RR 排列，`joint_state_broadcaster` 发布关节反馈。 |

我用 `check_urdf` 看展开后的模型。从 `base_footprint` 到 `base_link`，再从 `base_link` 分出四条轮组链。检查左前轮时，最容易记住的是 `fl_steer_joint` 接收弧度，`fl_drive_joint` 接收弧度每秒。

### 3C · 发速度、读反馈、算里程计

| 新增或修改的文件 | 用处 |
|---|---|
| 新增 [`worlds/workshop.sdf`](src/simcode/fw_mid_sim/worlds/workshop.sdf) | 提供一块平整的地板和物理环境，让我先在开阔场地测底盘。 |
| 新增 [`launch/sim.launch.py`](src/simcode/fw_mid_sim/launch/sim.launch.py) | `generate_launch_description()` 声明 `gui` 参数。`make_sim()` 加载 Xacro、启动 Gazebo 和 `/clock` 桥接，生成小车后激活控制器，最后启动底盘节点。 |
| 扩写 [`fw_mid_sim/base_control.py`](src/simcode/fw_mid_sim/fw_mid_sim/base_control.py) | `receive_cmd()` 读取 `/cmd_vel` 并限制 06 档曲率。`control()` 发轮角和轮速指令，按转向对齐误差缩放驱动轮速。`receive_feedback()` 按名称读取 `/joint_states`，用反馈时间戳积分位姿。`publish_odom()` 发布 `/odom` 和可选的 `odom → base_footprint` TF。 |
| 更新 [`setup.py`](src/simcode/fw_mid_sim/setup.py) 和 [`package.xml`](src/simcode/fw_mid_sim/package.xml) | 把 `base_control` 注册为可执行入口，安装工坊场景和启动文件，并声明运行所需的 ROS 依赖。 |

我把 3A 的左转指令发给仿真，三个控制器都进入 `active`，读到了下面这一组关节反馈。

| 轮组 | 实测转角 | 实测驱动轮速 |
|---|---:|---:|
| 左前 FL | +23.65° | 1.246 rad/s |
| 右前 FR | +12.85° | 2.248 rad/s |
| 左后 RL | −23.65° | 1.246 rad/s |
| 右后 RR | −12.85° | 2.248 rad/s |

`/odom` 反算得到 `vx≈0.20 m/s`、`vy≈0`、`wz≈0.30 rad/s`。停止发送 `/cmd_vel` 1 秒后，驱动轮速和里程计速度都接近零。这一段最有意思的是，同一组数字先在纸上算出来，再从 Gazebo 的关节反馈里读回来。

## 在本机重跑第 3 步

首次下载项目后，在工作区根目录运行 `git submodule update --init --recursive` 取得固定版本的 Nav2 源码。以下命令使用本机已安装的 ROS 2 Humble、Gazebo Fortress、`ros_gz` 和 `gz_ros2_control`。

在终端 A 构建并启动。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/simcode/fw_mid_sim
source install/local_setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1 IGN_PARTITION=fw_lesson_3c GZ_PARTITION=fw_lesson_3c
ros2 launch fw_mid_sim sim.launch.py gui:=true
```

在终端 B 检查控制器并持续发送左转指令。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 control list_controllers
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.20}, angular: {z: 0.30}}'
```

在终端 C 读取反馈。读完后，在终端 B 按 `Ctrl+C` 停止发指令，再读一帧 `/odom` 检查速度是否归零。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /odom nav_msgs/msg/Odometry --once
```

下一篇接着读 `base_control.py` 的 `publish_odom()`。我会把轮式里程计接给 EKF，再分别检查 C32 点云和 H30 IMU 的时间戳、坐标系与频率。

**2026 年 11 月 23 日 · 宁夏银川**

## 第 4 步 · 接通 C32、H30 和 EKF

第三步结束时，小车已经能按 06 档的运动学行驶，也能用四轮反馈推算自身运动。第四步接了两条数据链。C32 扫描环境，保留三维点云并生成建图用的二维扫描。H30 测量车体转动，和轮式速度一起交给 EKF，输出连续的平面里程计。

### 4A · 在 Gazebo 里安装传感器

| 新增或修改的文件 | 我用它来做什么 |
|---|---|
| 更新 [`config/geometry.yaml` 的 `lidar_xyz` 到 `imu_rpy`](src/simcode/fw_mid_sim/config/geometry.yaml#L5) | 设置 C32、H30 相对 `base_link` 的位置和方向。H30 的 +X 朝车头、+Y 朝左、+Z 朝上，和车体轴一致，所以把原先写成绕 Z 轴转半圈的 `imu_rpy` 改成 `0 0 0`。平移量沿用参考模型。 |
| 新增 [`config/sensors.yaml`](src/simcode/fw_mid_sim/config/sensors.yaml) | 保存 C32 和 H30 的仿真参数。C32 以 20 Hz 产生三维扫描，H30 以 100 Hz 产生惯性测量。 |
| 扩写 [`urdf/fw_mid.urdf.xacro` 的 `lidar_joint`](src/simcode/fw_mid_sim/urdf/fw_mid.urdf.xacro#L113) 和 [`imu_joint`](src/simcode/fw_mid_sim/urdf/fw_mid.urdf.xacro#L165) | 把两个传感器固定到车身上。Gazebo 的 `gpu_lidar` 生成点云，`imu` 生成角速度和加速度。 |
| 更新 [`worlds/workshop.sdf`](src/simcode/fw_mid_sim/worlds/workshop.sdf) | 启用 Gazebo 的传感器系统，在工坊里加一面墙，方便检查雷达是否收到回波。 |

启动时遇到过 `The yaml dictionary has no key 'lidar_xyz'`。Xacro 的 `lidar_joint` 已经开始读取 `geom.lidar_xyz`，需要在 `geometry.yaml` 里补齐两件传感器的 `xyz` 和 `rpy`，再确认启动加载的是本工作区构建后的配置。

### 4B · 把测量从 Gazebo 接到 ROS

新增的 [`config/bridge.yaml`](src/simcode/fw_mid_sim/config/bridge.yaml) 声明了 Gazebo 到 ROS 的三条桥接规则。`/clock` 给仿真节点提供同一套时间，`/lidar/points` 携带 `PointCloud2`，`/imu/gazebo` 携带 Gazebo 的 IMU 消息。

新增的 [`h30_sensor.py` 的 `H30Sensor.forward()`](src/simcode/fw_mid_sim/fw_mid_sim/h30_sensor.py#L47) 接收 `/imu/gazebo`，保留测量时间戳，将 `frame_id` 设为 `imu_link`，按 H30 配置限制量程并填写角速度和加速度的协方差。它用 `orientation_covariance[0] = -1` 标记姿态字段不可用，再发布 `/imu/data_raw`。传感器测量轴由 4A 的固定关节设定。

### 4C · 投影二维扫描，融合轮速和陀螺仪

- 新增 [`config/estimation.yaml` 的 `pointcloud_to_laserscan`](src/simcode/fw_mid_sim/config/estimation.yaml#L1)。投影器在 `lidar_link` 坐标系按高度和距离筛点，再按水平方位角分格，每格取最近的有效距离，输出 `/scan`。原始三维点云继续通过 `/lidar/points` 发布，后续可以独立接到 Nav2 的体素层。
- 更新 [`base_control.py` 的 `BaseControl.publish_odom()`](src/simcode/fw_mid_sim/fw_mid_sim/base_control.py#L180)。轮式速度从实际轮角和轮速反算，发布时为位姿和速度填入协方差。`7 * i` 是把 `6 × 6` 矩阵按行展开后的对角线下标，等于第 `i` 行的起始位置 `6 * i` 加上第 `i` 列。
- 在 [`config/estimation.yaml` 的 `odom0_config` 和 `imu0_config`](src/simcode/fw_mid_sim/config/estimation.yaml#L28) 中，从 `/wheel/odom` 选择纵向和横向速度 `vx`、`vy`，从 `/imu/data_raw` 选择偏航角速度 `wz`。EKF 按平面模式输出 `/odometry/filtered` 和 `odom → base_footprint`。
- 更新 [`sim.launch.py` 的 `make_sim()`](src/simcode/fw_mid_sim/launch/sim.launch.py#L69)，把轮式里程计重映射为 `/wheel/odom`，关掉底盘节点对里程计 TF 的广播，由 EKF 统一发布。启动文件同时接入桥接器、H30 适配器、点云投影器和 EKF。[`setup.py`](src/simcode/fw_mid_sim/setup.py) 注册 `h30_sensor` 并安装相关配置，[`package.xml`](src/simcode/fw_mid_sim/package.xml) 声明投影和融合依赖。

### 4D · 在本工作区运行并验收

前面第 3 步的命令对应当时发布 `/odom` 的版本。完成第 4 步后，用 `/wheel/odom` 看轮式反馈，用 `/odometry/filtered` 看融合结果。

终端 A 从工作区根目录构建并启动。运行前确认当前环境能找到 `robot_localization` 和 `pointcloud_to_laserscan`。本机验收用的是解包到 `src/othercode/.dependencies/root` 的依赖。如果该目录还没准备好，可以把本机参考工程留存的 `.deb` 解到这里：

```bash
cd /home/ubuntu/share/yuhesen/fw-mid-nav2-sim
mkdir -p src/othercode/.dependencies/root
for deb in /home/ubuntu/share/yuhesen/nav2/src/othercode/.dependencies/*.deb; do
  dpkg-deb -x "$deb" src/othercode/.dependencies/root
done
```

```bash
cd /home/ubuntu/share/yuhesen/fw-mid-nav2-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/simcode/fw_mid_sim
source install/local_setup.bash

FW_DEP_ROOT="$PWD/src/othercode/.dependencies/root"
export AMENT_PREFIX_PATH="$FW_DEP_ROOT/opt/ros/humble:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$FW_DEP_ROOT/opt/ros/humble/lib:$FW_DEP_ROOT/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
ros2 pkg prefix robot_localization
ros2 pkg prefix pointcloud_to_laserscan

export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1 IGN_PARTITION=fw_lesson_4 GZ_PARTITION=fw_lesson_4
ros2 launch fw_mid_sim sim.launch.py gui:=false
```

终端 B 查 IMU 安装方向。看到 TF 后按 `Ctrl+C`，再查点云、扫描和融合速度。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 run tf2_ros tf2_echo base_link imu_link
```

```bash
ros2 topic echo /lidar/points --once --field header
ros2 topic echo /scan --once --field ranges
ros2 topic echo /odometry/filtered --once --field child_frame_id
ros2 topic echo /odometry/filtered --field twist.twist
```

保持 B 监听速度，在终端 C 连续发送约三秒的左转指令。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 topic pub -w 1 -r 20 -t 60 -p 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.20}, angular: {z: 0.30}}'
```

| 验收项 | 本工作区的仿真实测 |
|---|---|
| C32 点云与二维扫描 | 点云的 `frame_id` 为 `lidar_link`。`/lidar/points` 与 `/scan` 的消息时间戳间隔中位数均对应 20 Hz。一帧扫描的 1000 个角度格中有 178 个有限距离值。 |
| H30 静止和安装方向 | `/imu/data_raw` 为 100 Hz，静止时 Z 轴加速度约 `+9.798 m/s²`。`base_link → imu_link` 的旋转四元数为 `[0, 0, 0, 1]`。 |
| 里程计与 TF | `/wheel/odom` 和 `/odometry/filtered` 的消息时间戳间隔中位数均对应 100 Hz，两者的 `child_frame_id` 都是 `base_footprint`，可查到 `odom → base_footprint`。 |
| 稳定左转 | 发 `vx=0.20 m/s`、`wz=+0.30 rad/s` 时，轮式 `vx` 为 `0.2000 m/s`，H30 `wz` 为 `+0.3005 rad/s`，EKF 输出 `vx=0.2024 m/s`、`wz=+0.3005 rad/s`。 |

我一开始看到 `/joint_states` 显示 `base_link`，又在融合速度里读到接近零的值。查了 [`fw_mid.urdf.xacro` 的 `base_footprint_joint`](src/simcode/fw_mid_sim/urdf/fw_mid.urdf.xacro#L29) 才弄清楚，`base_link` 位于 `base_footprint` 正上方 `0.12 m`，两个坐标系都用得上。[`BaseControl.control()`](src/simcode/fw_mid_sim/fw_mid_sim/base_control.py#L126) 在指令中断 `0.30 s` 后会把驱动轮目标速度设为零，所以要在发指令的同时看 B，才能读到稳定左转的速度。

这次验收确认了传感器、投影和融合链路能够运行。下一步用 SLAM Toolbox 读取 `/scan` 和 `odom → base_footprint`，构建二维地图并发布 `map → odom`。

**2026 年 11 月 24 日**
