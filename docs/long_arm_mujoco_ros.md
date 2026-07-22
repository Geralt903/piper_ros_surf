# Long Arm MuJoCo ROS Simulation

这是固定底盘、绝对控制条件下的 MuJoCo + ROS1 仿真入口。

模型使用 `piper_ros_surf` 自带的真实 Piper 描述：

- ROS URDF：`src/piper_description/urdf/piper_description.urdf`
- MuJoCo 物理模型：`src/piper_description/mujoco_model/piper_wall_mount_description.xml`

默认使用带夹爪模型。机械臂关节是 `joint1` 到 `joint6`，夹爪滑动关节是 `joint7` 和 `joint8`。
默认安装方式是墙面垂直安装：MuJoCo 里新增了固定墙体和 `wall_mount` 固定 body，整机绕 Y 轴旋转 90 度安装在墙面上。原始桌面模型仍保留在 `src/piper_description/mujoco_model/piper_description.xml`。

说明：原始 URDF 里的 mesh 是 `package://...` 路径，MuJoCo Python 不能直接解析这个路径；因此仿真物理加载仓库已有的 Piper MuJoCo XML，这个 XML 使用同一套真实 STL、关节名和惯量。节点同时把真实 URDF 发布到 `/robot_description`。

## Docker 启动

在 `piper_ros_surf` 根目录运行：

```bash
./start_long_arm_mujoco_ros.sh
```

无参数会进入命令行菜单。也可以直接运行：

```bash
./start_long_arm_mujoco_ros.sh start
```

`start` 会停止旧容器并启动最常用模式：MuJoCo 原生界面 + ROS topic / 前端控制。

微型前端默认随仿真一起启动：

```bash
./start_long_arm_mujoco_ros.sh frontend
```

默认地址：

```text
http://localhost:8088
```

前端控制链路是：浏览器 UI -> HTTP bridge -> ROS topic -> MuJoCo -> ROS 状态 topic -> 前端状态刷新。

显示走 ROS topic。默认节点发布 `/joint_states`、末端状态和稳定性，不启动 MuJoCo viewer，避免 Docker 里的 NVIDIA/OpenGL 驱动问题。

如果要 MuJoCo 原生界面 + 前端/ROS 控制：

```bash
./start_long_arm_mujoco_ros.sh start
```

脚本会强制走 Mesa 软件渲染，避免 Docker 里加载宿主 `nvidia-drm`：

```text
LIBGL_ALWAYS_SOFTWARE=1
MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
__GLX_VENDOR_LIBRARY_NAME=mesa
```

如果需要 `/tf` 给 RViz，先更新镜像，再用 `demo-tf`：

```bash
./start_long_arm_mujoco_ros.sh build
./start_long_arm_mujoco_ros.sh demo-tf
```

如果要发布手部相机图像 topic：

```bash
./start_long_arm_mujoco_ros.sh camera
```

这会额外发布 `/long_arm/hand_camera/image_raw`。不需要图像时，普通 `control` 仍只发布相机位姿，开销更小。

## ROS 通讯

常用脚本命令只需要记这几个：

```bash
./start_long_arm_mujoco_ros.sh start     # MuJoCo界面 + ROS/前端控制
./start_long_arm_mujoco_ros.sh camera    # 手部相机图像
./start_long_arm_mujoco_ros.sh status    # 检查状态
./start_long_arm_mujoco_ros.sh open      # 张开夹爪
./start_long_arm_mujoco_ros.sh close     # 闭合夹爪
./start_long_arm_mujoco_ros.sh half      # 半开夹爪
./start_long_arm_mujoco_ros.sh stop      # 停止仿真
```

旧长命令仍然保留，例如：

```bash
./start_long_arm_mujoco_ros.sh topics
./start_long_arm_mujoco_ros.sh echo
./start_long_arm_mujoco_ros.sh pub-joint
./start_long_arm_mujoco_ros.sh pub-pose
./start_long_arm_mujoco_ros.sh cleanup
./start_long_arm_mujoco_ros.sh build
```

脚本默认使用 `PROJECT=piper_ros_runtime` 和 `ROS_PORT=11312`，这样旧容器即使卡在 `piper_ros_surf-long_arm_mujoco_ros-1` / `11311` 上，也不会阻塞新仿真。

发布绝对末端位姿。末端 body 默认为真实 Piper 模型中的 `link6`；夹爪开合不参与 IK，通过 `joint7/joint8` 直接控制：

```bash
rostopic pub /long_arm/target_pose geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position: {x: 0.18, y: 0.0, z: 0.25}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

推荐用独立夹爪 topic 控制开合，`data` 是单侧夹爪滑动量，单位是米，范围默认 `0.0` 到 `0.03`：

```bash
rostopic pub -1 /long_arm/gripper_target std_msgs/Float64 "data: 0.03"
rostopic pub -1 /long_arm/gripper_target std_msgs/Float64 "data: 0.0"
```

前端也提供 Gripper 滑条和 Open/Close 按钮。HTTP API：

```bash
curl -X POST http://localhost:8088/api/gripper_target \
  -H "Content-Type: application/json" \
  -d '{"opening":0.02}'
```

发布绝对末端力：

```bash
rostopic pub /long_arm/tool_force geometry_msgs/WrenchStamped "header:
  frame_id: 'world'
wrench:
  force: {x: 18.0, y: 0.0, z: 0.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}"
```

输出话题：

- `/joint_states`：当前关节角、速度和电机力矩。
- `/long_arm/gripper_target`：夹爪开合目标，`std_msgs/Float64`。
- `/long_arm/end_effector_pose`：末端世界坐标位姿。
- `/long_arm/hand_camera_pose`：手部相机世界坐标位姿。
- `/long_arm/hand_camera/image_raw`：手部相机图像，只有 `camera-control` 或 `PUBLISH_CAMERA_IMAGE=true` 时发布。
- `/long_arm/stability`：`[com_x, com_y, com_z, cop_x, support_margin, total_mass]`。

## 手部摄像头

MuJoCo 模型中已经在 `link6` 上加入了 `hand_camera` 和一个小的相机外壳 geom。相机 pose 随仿真发布到 `/long_arm/hand_camera_pose`。

要接真实视觉链路时，推荐保持 topic 名不变：

- 仿真图像：`/long_arm/hand_camera/image_raw`
- 真实相机：用真实驱动发布同名或 remap 到同名 topic
- 标定：后续增加 `/long_arm/hand_camera/camera_info`

如果要调整相机安装位置，改 `src/piper_description/mujoco_model/piper_wall_mount_description.xml` 里 `link6` 下的：

```xml
<camera name="hand_camera" pos="0.035 0 0.105" euler="0 1.5708 0" fovy="70" />
```
