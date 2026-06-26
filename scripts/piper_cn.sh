#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${REPO_ROOT}/docker/humble/compose.yaml"
CONTAINER_NAME="piper_humble"
IMAGE_NAME="piper-ros:humble"
CAN_PORT="${CAN_PORT:-can0}"

cd "${REPO_ROOT}"

SUDO_KEEPALIVE_PID=""
PENDING_CHOICE=""

init_sudo() {
  if [ "${PIPER_CN_SKIP_SUDO:-0}" = "1" ]; then
    return 0
  fi

  if [ "$(id -u)" -eq 0 ]; then
    return 0
  fi

  echo "本脚本需要 sudo 权限来激活 USB-CAN。"
  echo "请先输入一次 sudo 密码，后续菜单里不会反复询问。"
  if ! sudo -v; then
    echo "sudo 验证失败，无法继续。"
    exit 1
  fi

  while true; do
    sudo -n true 2>/dev/null || exit
    sleep 60
  done &
  SUDO_KEEPALIVE_PID="$!"
}

cleanup() {
  if [ -n "${SUDO_KEEPALIVE_PID}" ]; then
    kill "${SUDO_KEEPALIVE_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

run_sudo() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

pause() {
  local answer=""
  echo
  read -r -p "按回车返回菜单，或直接输入下一个选项： " answer
  if [ -n "${answer}" ]; then
    PENDING_CHOICE="${answer}"
  fi
}

has_container() {
  docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

ensure_image() {
  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "未找到镜像 ${IMAGE_NAME}，开始构建 Humble 镜像..."
    docker build -t "${IMAGE_NAME}" -f docker/humble/Dockerfile .
  fi
}

ensure_container() {
  ensure_image
  if ! container_running; then
    echo "启动 Humble 容器 ${CONTAINER_NAME}..."
    docker compose -f "${COMPOSE_FILE}" up -d
  fi
}

ros_exec() {
  ensure_container
  docker exec -it "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source /ws/piper_ros/install/setup.bash && $*"
}

ros_run() {
  ensure_container
  docker exec "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source /ws/piper_ros/install/setup.bash && $*"
}

topic_exists() {
  local topic="$1"
  ros_run "ros2 topic list | grep -qx '${topic}'" >/dev/null 2>&1
}

show_status() {
  echo "==== Docker 容器状态 ===="
  docker ps -a --filter "name=${CONTAINER_NAME}" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
  echo
  echo "==== CAN 状态 ===="
  if ip link show "${CAN_PORT}" >/dev/null 2>&1; then
    ip -br link show "${CAN_PORT}"
    ip -details -statistics link show "${CAN_PORT}" | grep -E 'state|bitrate|parentdev|re-started|RX:|TX:|^[[:space:]]+[0-9]+' || true
  else
    echo "没有找到 ${CAN_PORT}。请确认 USB-CAN 已插入。"
  fi
}

start_container() {
  ensure_container
  show_status
}

enter_container() {
  ensure_container
  echo "进入容器后如需手动运行 ROS，环境会自动 source。"
  docker exec -it "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source /ws/piper_ros/install/setup.bash && exec bash"
}

activate_can() {
  echo "准备激活 ${CAN_PORT}，波特率 1000000。"
  if ! ip link show "${CAN_PORT}" >/dev/null 2>&1; then
    echo "错误：没有找到 ${CAN_PORT}。请先插上 USB-CAN。"
    return 1
  fi

  run_sudo ip link set "${CAN_PORT}" down || true
  run_sudo ip link set "${CAN_PORT}" type can bitrate 1000000
  run_sudo ip link set "${CAN_PORT}" up

  echo
  echo "激活完成，当前状态："
  ip -details link show "${CAN_PORT}" | grep -E 'state|bitrate|parentdev' || true
}

launch_arm() {
  ensure_container
  if ip -details link show "${CAN_PORT}" 2>/dev/null | grep -Eq 'can state (ERROR|BUS-OFF|STOPPED)'; then
    echo "警告：${CAN_PORT} 当前不是健康 CAN 状态："
    ip -details link show "${CAN_PORT}" | grep -E 'state|bitrate|parentdev' || true
    echo "如果随后出现 can0 is loss，请优先检查机械臂电源、CAN-H/CAN-L、终端电阻和线束。"
    echo
  fi
  echo "启动机械臂控制节点。"
  echo "注意：auto_enable=false，不会自动使能；确认反馈正常后再手动使能。"
  echo
  ros_exec "ros2 launch piper start_single_piper.launch.py can_port:=${CAN_PORT} auto_enable:=false gripper_exist:=true gripper_val_mutiple:=2"
}

echo_joint() {
  echo "查看机械臂关节反馈 /joint_states_single。按 Ctrl+C 退出。"
  if ! topic_exists "/joint_states_single"; then
    echo "当前还没有 /joint_states_single。请先选择 5 启动机械臂控制节点。"
    return 0
  fi
  ros_exec "ros2 topic echo /joint_states_single"
}

echo_status() {
  echo "查看机械臂状态 /arm_status。按 Ctrl+C 退出。"
  if ! topic_exists "/arm_status"; then
    echo "当前还没有 /arm_status。请先选择 5 启动机械臂控制节点。"
    return 0
  fi
  ros_exec "ros2 topic echo /arm_status"
}

launch_web_monitor() {
  ensure_container
  echo "启动网页状态预览。"
  echo "浏览器打开：http://localhost:8080"
  echo "如端口被占用，可用 WEB_PORT=8081 scripts/piper_cn.sh 后再启动。"
  echo
  ros_exec "ros2 run piper_web piper_web --host 0.0.0.0 --port ${WEB_PORT:-8080}"
}

restart_web_monitor() {
  ensure_container
  echo "重启网页服务..."
  echo

  echo "1. 停止现有 piper_web 进程..."
  ros_run "pkill -f 'piper_web --host' || true"
  sleep 1

  echo "2. 重新构建 piper_web 包..."
  ros_run "cd /ws/piper_ros && colcon build --packages-select piper_web --symlink-install"

  echo "3. 启动新的 piper_web 服务..."
  ros_run "nohup ros2 run piper_web piper_web --host 0.0.0.0 --port ${WEB_PORT:-8080} > /tmp/piper_web.log 2>&1 &"
  sleep 2

  echo
  echo "重启完成。浏览器打开：http://localhost:${WEB_PORT:-8080}"
}

enable_arm() {
  echo "准备使能机械臂。请确认机械臂工作范围内无人、无障碍物。"
  read -r -p "确认使能请输入 yes： " answer
  if [ "${answer}" != "yes" ]; then
    echo "已取消使能。"
    return 0
  fi
  ros_exec "ros2 service call /enable_srv piper_msgs/srv/Enable '{enable_request: true}'"
}

disable_arm() {
  echo "发送失能命令。"
  ros_exec "ros2 service call /enable_srv piper_msgs/srv/Enable '{enable_request: false}'"
}

launch_joint_menu() {
  echo "启动关节控制小菜单。"
  echo "注意：请先启动机械臂控制节点并使能，确认机械臂工作范围内无人、无障碍物。"
  echo "默认发布到 /joint_states，对应 start_single_piper.launch.py 的控制输入。"
  echo
  ros_exec "ros2 run piper piper_joint_menu"
}

stop_container() {
  echo "停止 Humble 容器。"
  docker compose -f "${COMPOSE_FILE}" down
}

main_menu() {
  init_sudo

  while true; do
    clear
    echo "Piper ROS2 Humble 中文助手"
    echo "工作目录：${REPO_ROOT}"
    echo "CAN 端口：${CAN_PORT}"
    echo
    echo "1) 查看 Docker 和 CAN 状态"
    echo "2) 启动 Humble 容器"
    echo "3) 进入 Humble 容器 Shell"
    echo "4) 激活 USB-CAN (${CAN_PORT}, 1000000)"
    echo "5) 启动机械臂控制节点（不自动使能）"
    echo "6) 查看关节反馈 /joint_states_single"
    echo "7) 查看机械臂状态 /arm_status"
    echo "8) 启动网页状态预览（http://localhost:${WEB_PORT:-8080}）"
    echo "9) 重启网页服务（重新构建并启动）"
    echo "10) 使能机械臂"
    echo "11) 失能机械臂"
    echo "12) 关节控制小菜单"
    echo "13) 停止 Humble 容器"
    echo "0) 退出"
    echo
    if [ -n "${PENDING_CHOICE}" ]; then
      choice="${PENDING_CHOICE}"
      PENDING_CHOICE=""
      echo "请选择： ${choice}"
    else
      read -r -p "请选择： " choice
    fi
    echo

    case "${choice}" in
      1) show_status; pause ;;
      2) start_container; pause ;;
      3) enter_container ;;
      4) activate_can; pause ;;
      5) launch_arm; pause ;;
      6) echo_joint; pause ;;
      7) echo_status; pause ;;
      8) launch_web_monitor; pause ;;
      9) restart_web_monitor; pause ;;
      10) enable_arm; pause ;;
      11) disable_arm; pause ;;
      12) launch_joint_menu; pause ;;
      13) stop_container; pause ;;
      0) exit 0 ;;
      *) echo "无效选择。"; pause ;;
    esac
  done
}

main_menu
