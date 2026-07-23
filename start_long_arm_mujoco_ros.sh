#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${ROOT_DIR}/docker-compose.ros-mujoco.yml"
SERVICE="long_arm_mujoco_ros"

# Defaults deliberately avoid the old piper_ros_surf compose project and port
# 11311, because an earlier viewer container may be stuck there.
PROJECT="${PROJECT:-piper_ros_runtime}"
ROS_PORT="${ROS_PORT:-11312}"
ROS_MASTER_URI="http://localhost:${ROS_PORT}"
MUJOCO_GL="${MUJOCO_GL:-osmesa}"
BASE_MUJOCO_GL="${MUJOCO_GL}"
PUBLISH_TF="${PUBLISH_TF:-false}"
WEB="${WEB:-true}"
WEB_PORT="${WEB_PORT:-8088}"
PUBLISH_CAMERA_IMAGE="${PUBLISH_CAMERA_IMAGE:-false}"
CAMERA_WIDTH="${CAMERA_WIDTH:-320}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-240}"
CAMERA_RATE_HZ="${CAMERA_RATE_HZ:-10}"
VIEWER="${VIEWER:-false}"
LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
__GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-mesa}"

compose() {
  COMPOSE_PROJECT_NAME="${PROJECT}" \
  ROS_PORT="${ROS_PORT}" \
  ROS_MASTER_URI="${ROS_MASTER_URI}" \
  MUJOCO_GL="${MUJOCO_GL}" \
  PUBLISH_TF="${PUBLISH_TF}" \
  WEB="${WEB}" \
  WEB_PORT="${WEB_PORT}" \
  PUBLISH_CAMERA_IMAGE="${PUBLISH_CAMERA_IMAGE}" \
  CAMERA_WIDTH="${CAMERA_WIDTH}" \
  CAMERA_HEIGHT="${CAMERA_HEIGHT}" \
  CAMERA_RATE_HZ="${CAMERA_RATE_HZ}" \
  VIEWER="${VIEWER}" \
  LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE}" \
  MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE}" \
  __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME}" \
  docker compose -p "${PROJECT}" -f "${COMPOSE_FILE}" "$@"
}

usage() {
  cat <<EOF
用法:
  ./start_long_arm_mujoco_ros.sh
  ./start_long_arm_mujoco_ros.sh <command>

不带 command 时进入中文交互菜单。

常用短命令:
  start       重启并打开 MuJoCo 原生界面，等待前端/ROS 控制。
  camera      重启控制模式，并发布手部相机图像。
  stop        停止仿真。
  status      检查容器、前端、关节和相机状态。
  logs        查看日志。
  open        张开夹爪。
  close       闭合夹爪。
  half        半开夹爪。

其它命令:
  demo, control, viewer, viewer-control, camera-control
  restart-viewer-control, restart-camera-control
  frontend, topics, echo, api-test, pub-joint, pub-pose
  shell, ps, health, doctor, cleanup, build, old-stop

环境变量:
  PROJECT=${PROJECT}
  ROS_PORT=${ROS_PORT}
  MUJOCO_GL=${MUJOCO_GL}
  VIEWER=${VIEWER}
  PUBLISH_TF=${PUBLISH_TF}
  WEB=${WEB}
  WEB_PORT=${WEB_PORT}
  PUBLISH_CAMERA_IMAGE=${PUBLISH_CAMERA_IMAGE}

示例:
  ./start_long_arm_mujoco_ros.sh
  ./start_long_arm_mujoco_ros.sh start
  ./start_long_arm_mujoco_ros.sh camera
  ./start_long_arm_mujoco_ros.sh open
  ./start_long_arm_mujoco_ros.sh stop
  ROS_PORT=11313 PROJECT=piper_test ./start_long_arm_mujoco_ros.sh demo
EOF
}

require_docker() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "docker is not installed or not in PATH" >&2
    exit 1
  fi
}

runtime_container() {
  echo "${PROJECT}-${SERVICE}-1"
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "$1"
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "$1"
}

cleanup_nonrunning_conflicts() {
  local current
  current="$(runtime_container)"
  local name
  while IFS= read -r name; do
    [[ -z "${name}" ]] && continue
    if container_running "${name}"; then
      continue
    fi
    echo "清理残留容器: ${name}"
    docker rm -f "${name}" >/dev/null || true
  done < <(docker ps -a --format '{{.Names}}' | grep -E "(^|_)${current}$" || true)
}

preflight_start() {
  local current
  current="$(runtime_container)"
  cleanup_nonrunning_conflicts
  if container_running "${current}"; then
    echo "已有仿真容器正在运行: ${current}"
    echo "不会重复创建同名容器。"
    echo
    echo "前端地址: http://localhost:${WEB_PORT}"
    echo "查看日志: ./start_long_arm_mujoco_ros.sh logs"
    echo "查看状态: ./start_long_arm_mujoco_ros.sh ps"
    echo "重新启动: 先在菜单选择“停止仿真”，再选择启动项"
    return 1
  fi
  return 0
}

print_started() {
  echo
  echo "启动成功。"
  echo "前端地址: http://localhost:${WEB_PORT}"
  echo "查看日志: ./start_long_arm_mujoco_ros.sh logs"
}

wait_frontend() {
  local seconds="${1:-30}"
  local i
  if [[ "${WEB}" != "true" ]]; then
    return 0
  fi
  for ((i = 0; i < seconds; i++)); do
    if curl -fsS "http://localhost:${WEB_PORT}/api/state" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

health_check() {
  echo "容器:"
  show_ps
  echo
  echo "前端:"
  if curl -fsS -m 2 "http://localhost:${WEB_PORT}/api/state" >/tmp/piper_mujoco_state.json; then
    echo "  OK: http://localhost:${WEB_PORT}"
    python3 - <<'PY' 2>/dev/null || true
import json
with open("/tmp/piper_mujoco_state.json", "r") as f:
    state = json.load(f)
names = state.get("joint_state", {}).get("name", [])
camera = state.get("hand_camera_pose", {}).get("position")
print("  joints:", ", ".join(names) if names else "no joint state")
print("  hand_camera_pose:", "OK" if camera else "missing")
if not camera:
    print("  提示: 当前运行的可能是旧容器；在菜单选择 8 或 9 重启到新模型。")
PY
  else
    echo "  FAIL: http://localhost:${WEB_PORT}/api/state 无响应"
    echo "  建议先看日志: ./start_long_arm_mujoco_ros.sh logs"
  fi
}

print_banner() {
  local mode="$1"
  cat <<EOF

启动 Piper MuJoCo ROS 仿真
  模式:        ${mode}
  Docker项目: ${PROJECT}
  ROS master: ${ROS_MASTER_URI}
  MuJoCo GL:  ${MUJOCO_GL}
  原生界面:    ${VIEWER}
  发布 /tf:    ${PUBLISH_TF}
  Web前端:     ${WEB}
  前端地址:    http://localhost:${WEB_PORT}
  模型:        piper_wall_mount_description.xml
  安装方式:    墙面垂直安装
  URDF:        piper_description.urdf

ROS 话题:
  /joint_states
  /long_arm/joint_target
  /long_arm/gripper_target
  /long_arm/target_pose
  /long_arm/tool_force
  /long_arm/end_effector_pose
  /long_arm/hand_camera_pose
  /long_arm/stability

EOF
}

start_runtime() {
  local demo="$1"
  local mode="$2"
  local viewer="${3:-false}"
  if [[ "${viewer}" == "true" ]]; then
    VIEWER=true
    MUJOCO_GL=glfw
    if command -v xhost >/dev/null 2>&1; then
      xhost +local:docker >/dev/null || true
    fi
  else
    VIEWER=false
    MUJOCO_GL="${BASE_MUJOCO_GL}"
  fi
  print_banner "${mode}"
  cd "${ROOT_DIR}"
  preflight_start || return 1
  if ! docker image inspect piper-ros-mujoco:noetic >/dev/null 2>&1; then
    echo "Docker image piper-ros-mujoco:noetic is missing. Building it first."
    compose build
  fi
  DEMO="${demo}" VIEWER="${VIEWER}" compose up --remove-orphans
}

start_runtime_detached() {
  local demo="$1"
  local mode="$2"
  local viewer="${3:-false}"
  if [[ "${viewer}" == "true" ]]; then
    VIEWER=true
    MUJOCO_GL=glfw
    if command -v xhost >/dev/null 2>&1; then
      xhost +local:docker >/dev/null || true
    fi
  else
    VIEWER=false
    MUJOCO_GL="${BASE_MUJOCO_GL}"
  fi
  print_banner "${mode}"
  cd "${ROOT_DIR}"
  preflight_start || return 1
  if ! docker image inspect piper-ros-mujoco:noetic >/dev/null 2>&1; then
    echo "Docker image piper-ros-mujoco:noetic is missing. Building it first."
    compose build
  fi
  if DEMO="${demo}" VIEWER="${VIEWER}" compose up -d --remove-orphans; then
    if wait_frontend 30; then
      print_started
    else
      echo
      echo "容器已启动，但前端 30 秒内没有响应。"
      echo "查看日志: ./start_long_arm_mujoco_ros.sh logs"
      return 1
    fi
  else
    echo
    echo "启动失败。当前 Docker 容器状态:"
    show_ps
    return 1
  fi
}

exec_in_service() {
  cd "${ROOT_DIR}"
  compose exec "${SERVICE}" bash -lc "source devel/setup.bash && export ROS_MASTER_URI=${ROS_MASTER_URI} && $1"
}

publish_joint_demo() {
  exec_in_service "rostopic pub -1 /long_arm/joint_target sensor_msgs/JointState \"name: [joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8]
position: [0.0, 0.85, -1.45, 0.0, 0.85, 0.0, 0.010, -0.010]\""
}

publish_pose_demo() {
  exec_in_service "rostopic pub -1 /long_arm/target_pose geometry_msgs/PoseStamped \"header:
  frame_id: world
pose:
  position:
    x: 0.16
    y: 0.0
    z: 0.13
  orientation:
    x: 0.0
    y: -0.8011
    z: 0.0
    w: -0.5985\""
}

publish_gripper() {
  local opening="$1"
  exec_in_service "rostopic pub -1 /long_arm/gripper_target std_msgs/Float64 \"data: ${opening}\""
}

show_ps() {
  docker ps --format 'table {{.ID}}\t{{.Image}}\t{{.Status}}\t{{.Names}}' | grep -E 'piper-ros-mujoco|NAMES' || true
}

wait_not_running() {
  local name="$1"
  local seconds="${2:-8}"
  local i
  for ((i = 0; i < seconds; i++)); do
    if ! docker ps --format '{{.Names}}' | grep -qx "${name}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

doctor() {
  echo "Docker:"
  docker --version || true
  docker compose version || true
  echo
  echo "Runtime project:"
  echo "  PROJECT=${PROJECT}"
  echo "  ROS_PORT=${ROS_PORT}"
  echo "  ROS_MASTER_URI=${ROS_MASTER_URI}"
  echo "  MUJOCO_GL=${MUJOCO_GL}"
  echo "  VIEWER=${VIEWER}"
  echo "  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
  echo "  MESA_LOADER_DRIVER_OVERRIDE=${MESA_LOADER_DRIVER_OVERRIDE}"
  echo "  __GLX_VENDOR_LIBRARY_NAME=${__GLX_VENDOR_LIBRARY_NAME}"
  echo "  PUBLISH_TF=${PUBLISH_TF}"
  echo "  WEB=${WEB}"
  echo "  WEB_PORT=${WEB_PORT}"
  echo "  PUBLISH_CAMERA_IMAGE=${PUBLISH_CAMERA_IMAGE}"
  echo "  CAMERA_WIDTH=${CAMERA_WIDTH}"
  echo "  CAMERA_HEIGHT=${CAMERA_HEIGHT}"
  echo "  CAMERA_RATE_HZ=${CAMERA_RATE_HZ}"
  echo
  echo "Containers:"
  show_ps
  echo
  echo "Compose command:"
  cd "${ROOT_DIR}"
  compose config | sed -n '/command:/,/environment:/p'
}

old_stop() {
  local old="piper_ros_surf-${SERVICE}-1"
  echo "Trying to stop old container: ${old}"
  if docker stop "${old}"; then
    return 0
  fi
  if wait_not_running "${old}" 30; then
    echo "Old container exited after Docker reported an error."
    return 0
  fi
  echo "docker stop failed; trying to terminate ROS processes inside the container."
  if docker exec "${old}" bash -lc "kill -INT 1 || true; pkill -INT -f roslaunch || true; pkill -TERM -f rosmaster || true; pkill -TERM -f long_arm_static_mujoco_ros.py || true; pkill -TERM -f piper_mujoco_web_bridge.py || true"; then
    if wait_not_running "${old}" 30; then
      echo "Old container exited."
      return 0
    fi
  fi
  echo
  echo "docker stop failed. Try manually:"
  echo "  sudo docker kill ${old}"
  echo "  sudo docker rm -f ${old}"
  echo
  echo "If Docker still says permission denied, restart Docker daemon:"
  echo "  sudo systemctl restart docker"
  echo
  echo "This also stops other Docker containers."
}

stop_runtime() {
  cd "${ROOT_DIR}"
  if compose down --remove-orphans; then
    return 0
  fi

  local current="${PROJECT}-${SERVICE}-1"
  if wait_not_running "${current}" 30; then
    echo "Runtime container is no longer running."
    return 0
  fi

  echo "compose down failed; trying to terminate ROS processes inside ${current}."
  if docker exec "${current}" bash -lc "kill -INT 1 || true; pkill -INT -f roslaunch || true; pkill -TERM -f rosmaster || true; pkill -TERM -f long_arm_static_mujoco_ros.py || true; pkill -TERM -f piper_mujoco_web_bridge.py || true"; then
    if wait_not_running "${current}" 30; then
      echo "Runtime container exited."
      return 0
    fi
  fi

  echo "Could not stop ${current}. Last resort:"
  echo "  sudo systemctl restart docker"
  return 1
}

cleanup_runtime() {
  cleanup_nonrunning_conflicts
  echo "清理完成。当前容器状态:"
  show_ps
}

ensure_runtime_stopped() {
  local current
  current="$(runtime_container)"
  stop_runtime || true
  if wait_not_running "${current}" 30; then
    cleanup_nonrunning_conflicts
    return 0
  fi
  echo "当前容器仍在运行，无法继续重启: ${current}"
  echo "可以尝试: sudo systemctl restart docker"
  return 1
}

restart_runtime_detached() {
  local demo="$1"
  local mode="$2"
  local viewer="${3:-false}"
  echo "先停止当前仿真..."
  ensure_runtime_stopped || return 1
  echo
  echo "重新启动: ${mode}"
  start_runtime_detached "${demo}" "${mode}" "${viewer}"
}

run_command() {
  local command="$1"
  case "${command}" in
    start|restart|restart-viewer)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      restart_runtime_detached false viewer-control true
      ;;
    camera|restart-camera)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=true
      restart_runtime_detached false camera-control false
      ;;
    demo)
      start_runtime true demo false
      ;;
    control)
      start_runtime false control false
      ;;
    viewer)
      start_runtime true viewer true
      ;;
    viewer-control)
      start_runtime false viewer-control true
      ;;
    camera-control)
      PUBLISH_CAMERA_IMAGE=true
      start_runtime false camera-control false
      ;;
    restart-viewer-control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      restart_runtime_detached false viewer-control true
      ;;
    restart-camera-control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=true
      restart_runtime_detached false camera-control false
      ;;
    demo-tf)
      PUBLISH_TF=true
      start_runtime true demo-tf false
      ;;
    control-tf)
      PUBLISH_TF=true
      start_runtime false control-tf false
      ;;
    build)
      cd "${ROOT_DIR}"
      compose build
      ;;
    topics)
      exec_in_service "rostopic list | sort"
      ;;
    echo)
      exec_in_service "rostopic echo -n1 /joint_states && rostopic echo -n1 /long_arm/end_effector_pose && rostopic echo -n1 /long_arm/stability"
      ;;
    frontend)
      echo "Frontend: http://localhost:${WEB_PORT}"
      ;;
    api-test)
      curl -sS -X POST "http://localhost:${WEB_PORT}/api/joint_target" \
        -H "Content-Type: application/json" \
        -d '{"positions":[0.0,0.85,-1.45,0.0,0.85,0.0,0.010,-0.010]}' && echo
      ;;
    pub-joint|joint-demo)
      publish_joint_demo
      ;;
    pub-gripper|pub-gripper-half|half)
      publish_gripper 0.015
      ;;
    pub-gripper-open|open)
      publish_gripper 0.030
      ;;
    pub-gripper-close|close)
      publish_gripper 0.0
      ;;
    pub-pose)
      publish_pose_demo
      ;;
    enter|shell)
      cd "${ROOT_DIR}"
      compose exec "${SERVICE}" bash
      ;;
    ps)
      show_ps
      ;;
    logs)
      cd "${ROOT_DIR}"
      compose logs -f "${SERVICE}"
      ;;
    health|status)
      health_check
      ;;
    doctor)
      doctor
      ;;
    stop)
      stop_runtime
      ;;
    cleanup)
      cleanup_runtime
      ;;
    old-stop|sudo-stop)
      old_stop
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      echo "Unknown command: ${command}" >&2
      usage >&2
      return 2
      ;;
  esac
}

print_menu() {
  clear 2>/dev/null || true
  cat <<EOF
Piper MuJoCo ROS 仿真菜单

当前配置:
  Docker项目: ${PROJECT}
  ROS地址:    ${ROS_MASTER_URI}
  前端地址:   http://localhost:${WEB_PORT}
  模型:       墙面垂直安装 + 夹爪 + 手部相机

常用操作:
  1) 启动仿真        MuJoCo界面 + ROS/前端控制
  2) 状态检查        容器/前端/关节/相机
  3) 夹爪控制        张开/闭合/半开
  4) 前端地址        显示 http://localhost:${WEB_PORT}
  5) 日志
  6) 停止仿真
  7) 高级功能        相机/清理/topics/shell/build
  q) 退出

短命令也可直接输入: start, camera, status, open, close, half, stop
EOF
}

menu_to_command() {
  case "$1" in
    1) echo start ;;
    2) echo status ;;
    3) echo gripper-menu ;;
    4) echo frontend ;;
    5) echo logs ;;
    6) echo stop ;;
    7|a|A|advanced) echo advanced-menu ;;
    m|M|more) echo help ;;
    h|H|help|--help|-h) echo help ;;
    q|Q|quit|exit) echo quit ;;
    *) echo "$1" ;;
  esac
}

run_menu_command() {
  local command="$1"
  case "${command}" in
    demo)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      start_runtime_detached true demo false
      ;;
    control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      start_runtime_detached false control false
      ;;
    viewer)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      start_runtime_detached true viewer true
      ;;
    viewer-control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      start_runtime_detached false viewer-control true
      ;;
    camera-control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=true
      start_runtime_detached false camera-control false
      ;;
    restart-viewer-control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=false
      restart_runtime_detached false viewer-control true
      ;;
    restart-camera-control)
      PUBLISH_TF=false
      PUBLISH_CAMERA_IMAGE=true
      restart_runtime_detached false camera-control false
      ;;
    demo-tf)
      PUBLISH_TF=true
      PUBLISH_CAMERA_IMAGE=false
      start_runtime_detached true demo-tf false
      ;;
    control-tf)
      PUBLISH_TF=true
      PUBLISH_CAMERA_IMAGE=false
      start_runtime_detached false control-tf false
      ;;
    gripper-menu)
      gripper_menu
      ;;
    advanced-menu)
      advanced_menu
      ;;
    *)
      run_command "${command}"
      ;;
  esac
}

gripper_menu() {
  while true; do
    clear 2>/dev/null || true
    cat <<EOF
夹爪控制

  1) 张开
  2) 闭合
  3) 半开
  b) 返回

EOF
    read -r -p "请选择: " selection
    case "${selection}" in
      1|open)
        run_command open
        ;;
      2|close)
        run_command close
        ;;
      3|half)
        run_command half
        ;;
      b|B|back|q|Q)
        return 0
        ;;
      *)
        echo "无效选项: ${selection}"
        ;;
    esac
    echo
    read -r -p "按 Enter 继续..." _
  done
}

advanced_menu() {
  while true; do
    clear 2>/dev/null || true
    cat <<EOF
高级功能

  1) 启动相机图像       camera
  2) 清理残留容器       cleanup
  3) ROS话题列表        topics
  4) 读取一次状态       echo
  5) 测试关节命令       api-test
  6) 发布示例关节       pub-joint
  7) 发布示例位姿       pub-pose
  8) 进入容器shell      shell
  9) 诊断信息           doctor
 10) 构建镜像           build
 11) 更多命令帮助       help
  b) 返回

EOF
    read -r -p "请选择: " selection
    case "${selection}" in
      1|camera)
        run_menu_command camera
        ;;
      2|cleanup)
        run_command cleanup
        ;;
      3|topics)
        run_command topics
        ;;
      4|echo)
        run_command echo
        ;;
      5|api-test)
        run_command api-test
        ;;
      6|pub-joint)
        run_command pub-joint
        ;;
      7|pub-pose)
        run_command pub-pose
        ;;
      8|shell|enter)
        run_command shell
        ;;
      9|doctor)
        run_command doctor
        ;;
      10|build)
        run_command build
        ;;
      11|help|h|H)
        run_command help
        ;;
      b|B|back|q|Q)
        return 0
        ;;
      *)
        echo "无效选项: ${selection}"
        ;;
    esac
    echo
    read -r -p "按 Enter 继续..." _
  done
}

interactive_menu() {
  while true; do
    print_menu
    echo
    read -r -p "请选择: " selection
    local command
    command="$(menu_to_command "${selection}")"

    if [[ -z "${command}" ]]; then
      continue
    fi
    if [[ "${command}" == "quit" ]]; then
      return 0
    fi

    echo
    echo "执行命令: ${command}"
    echo
    set +e
    run_menu_command "${command}"
    local status=$?
    set -e

    echo
    if [[ ${status} -ne 0 ]]; then
      echo "命令返回状态 ${status}: ${command}"
    fi
    read -r -p "按 Enter 返回菜单..." _
  done
}

main() {
  require_docker
  if [[ $# -eq 0 ]]; then
    interactive_menu
    return
  fi
  run_command "$1"
}

main "$@"
