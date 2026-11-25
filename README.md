# FW-mid Nav2 导航小车

基于 ROS 2 Humble 和 Gazebo Fortress 的 FW-mid 四轮转向小车仿真。支持在仓库和办公室地图中定位、规划路径并自主行驶。小车使用三维激光雷达构建局部代价图，并用 EKF 融合轮式里程计与 IMU。

## 安装与构建

以下命令适用于 **Ubuntu 22.04（Jammy）**，从系统依赖装起。使用 Gazebo 窗口或 RViz 时需要桌面显示环境。

### 安装 ROS 2 Humble 和 Gazebo Fortress

开启 Ubuntu Universe，先安装构建与下载工具。

```bash
sudo apt-get update
sudo apt-get install -y software-properties-common ca-certificates curl locales \
  git cmake build-essential
sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
```

添加 ROS 2 和 Gazebo 的 APT 软件源，然后安装 Humble Desktop、Fortress、构建工具和开发库。已配置对应官方源的机器可以跳过该源的配置命令。

```bash
ros_apt_version=$(curl -fsSL -o /dev/null -w '%{url_effective}' \
  https://github.com/ros-infrastructure/ros-apt-source/releases/latest | awk -F/ '{print $NF}')
curl -fsSL -o /tmp/fw-mid-ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.jammy_all.deb"
sudo dpkg -i /tmp/fw-mid-ros2-apt-source.deb

sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n' \
  "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt-get update
sudo apt-get install -y ros-humble-desktop ignition-fortress \
  libignition-gazebo6-dev libignition-plugin-dev libxsimd-dev \
  python3-colcon-common-extensions python3-rosdep
```

### 下载源码并安装工作区依赖

如果仓库是私有的，请先用有读取权限的 GitHub 账号认证。

```bash
git clone --recurse-submodules https://github.com/Iwannabestronger/fw-mid-nav2-sim.git
cd fw-mid-nav2-sim
```

如果已经克隆了项目，在根目录运行 `git submodule update --init --recursive`。其余构建和启动命令也都在项目根目录执行。

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=fortress

# rosdep 在一台机器上只需初始化一次。
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
mapfile -t fw_package_paths < <(colcon list --paths-only \
  --base-paths src/navcode src/othercode src/simcode --packages-up-to fw_mid_sim)
rosdep install --from-paths "${fw_package_paths[@]}" \
  --ignore-src --rosdistro humble --skip-keys geographiclib \
  -t build -t build_export -t buildtool -t buildtool_export -t exec -y
```

`colcon list` 找出小车需要构建的包及其依赖，`rosdep` 根据这些包的 `package.xml` 安装其余系统依赖。`--ignore-src` 让工作区内的 Nav2、机器人定位、激光投影和 Gazebo 桥接等包走源码构建。GeographicLib 也从子模块构建，因此在 `rosdep` 中跳过它。

### 编译导航小车

首次构建时应用仓库自带的 MPPI 补丁。已应用的工作区可跳过这一条。

```bash
git -C src/navcode/navigation2 apply "$PWD/patches/navigation2-humble-mppi-feasible-control.patch"
```

```bash
cmake -S src/othercode/geographiclib -B build/geographiclib \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/geographiclib" \
  -DGEOGRAPHICLIB_LIB_TYPE=STATIC -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build build/geographiclib --parallel 2
cmake --install build/geographiclib

export CMAKE_PREFIX_PATH="$PWD/install/geographiclib${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
  --base-paths src/navcode src/othercode src/simcode \
  --packages-up-to fw_mid_sim --parallel-workers 3 \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

## 启动 Gazebo 和 Nav2

选择下面一张地图。每个终端都先进入项目根目录并加载环境。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

先在终端 A 启动 Gazebo，等小车出现，再在终端 B 启动 Nav2。无桌面显示时，可把 `gui:=true` 改成 `gui:=false`。切换地图前用 `Ctrl+C` 结束当前的导航和 Gazebo。

### 仓库地图

**终端 A · 启动仓库场景**

```bash
ros2 launch fw_mid_sim sim.launch.py scene:=workshop spawn_x:=0.95 gui:=true
```

**终端 B · 加载仓库地图并启动导航**

```bash
ros2 launch fw_mid_sim navigate_step9.launch.py stage:=direct \
  map:="$PWD/results/step5/workshop.yaml"
```

### 办公室地图

**终端 A · 启动办公室场景**

```bash
ros2 launch fw_mid_sim sim.launch.py scene:=office \
  spawn_x:=-4.50 spawn_y:=-1.15 cmd_vel_topic:=/cmd_vel_feasible gui:=true
```

**终端 B · 加载办公室地图并启动导航**

```bash
ros2 launch fw_mid_sim navigate_step9.launch.py stage:=office \
  map:="$(ros2 pkg prefix --share fw_mid_sim)/maps/office.yaml"
```

## 发送导航目标

终端 B 出现 `Managed nodes are active` 后，在已加载环境的终端 C 发送目标。下面的坐标均基于 `map`，车头朝 `+x`。

**仓库目标 `(1.55, 0.00)`**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.55, y: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

**办公室目标 `(6.35, 0.70)`**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 6.35, y: 0.70}, orientation: {w: 1.0}}}}' \
  --feedback
```

到点后，终端 C 显示 `SUCCEEDED`。也可以运行 `rviz2 --ros-args -p use_sim_time:=true`，把 **Fixed Frame** 设为 `map`，添加 **Map**（`/map`）和 **Path**（`/plan`）观察地图和规划路径。Map 显示的 **Durability Policy** 设为 **Transient Local**，让 RViz 在导航启动后打开时也能收到地图。
