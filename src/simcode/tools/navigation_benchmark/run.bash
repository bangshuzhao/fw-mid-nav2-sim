#!/usr/bin/env bash
set -eo pipefail
set -m

bench=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(cd -- "$bench/../../../.." && pwd)
scenario=${1:?用法: run.bash office|office-reference|crossing|planners 输出目录}
output=${2:?请给出一个新输出目录}
case "$scenario" in
  office|office-reference|planners) scene=office; stage=office; map=office; x=-4.50; y=-1.15; command=/cmd_vel_feasible ;;
  crossing) scene=narrow; stage=traffic; map=narrow_office; x=-4.8; y=0.0; command=/cmd_vel ;;
  *) printf '未知场景 %s\n' "$scenario" >&2; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source "$root/install/local_setup.bash"
share=$(ros2 pkg prefix --share fw_mid_sim)
mkdir -- "$output"
mkdir -- "$output/inputs"
cp -- "$share/config/nav2_step9.yaml" "$share/config/nav2_step9_traffic.yaml" \
  "$share/config/nav2_step10_office.yaml" "$share/config/office_lattice_004m.json" \
  "$share/maps/$map.yaml" "$share/maps/$map.pgm" "$share/worlds/$map.sdf" "$output/inputs/"
{
  printf 'scenario=%s scene=%s stage=%s spawn_x=%s spawn_y=%s gui=true\n' "$scenario" "$scene" "$stage" "$x" "$y"
  git -C "$root" rev-parse HEAD
  sha256sum "$output/inputs/"* "$bench"/*.py "$bench/run.bash"
} > "$output/inputs.txt"

ros2 launch fw_mid_sim sim.launch.py "scene:=$scene" "spawn_x:=$x" "spawn_y:=$y" \
  "cmd_vel_topic:=$command" gui:=true > "$output/simulation.log" 2>&1 &
sim_pid=$!
nav_pid=
finish() {
  if [[ -n "$nav_pid" ]]; then kill -INT -- "-$nav_pid" 2>/dev/null || true; wait "$nav_pid" || true; fi
  kill -INT -- "-$sim_pid" 2>/dev/null || true
  wait "$sim_pid" || true
}
trap finish EXIT
trap 'exit 130' INT TERM

timeout 45s ros2 topic echo /odometry/filtered nav_msgs/msg/Odometry --once > "$output/startup-odom.yaml"
ros2 launch fw_mid_sim navigate_step9.launch.py "stage:=$stage" \
  "map:=$share/maps/$map.yaml" > "$output/navigation.log" 2>&1 &
nav_pid=$!
if [[ "$scenario" == planners ]]; then
  python3 "$bench/planners.py" "$output/planners.csv" --world "$output/inputs/$map.sdf"
else
  python3 "$bench/record.py" "$scenario" "$output/recording.json.gz"
  if [[ "$scenario" != crossing ]]; then
    python3 "$bench/analyze.py" "$output/recording.json.gz" "$output/metrics.csv" \
      --world "$output/inputs/$map.sdf"
  else
    python3 "$bench/traffic.py" "$output/recording.json.gz" "$output/metrics.csv"
  fi
fi
