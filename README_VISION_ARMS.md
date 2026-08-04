# 左右臂 Orbbec RGB-D 视觉服务部署与验收

本文记录 `vision_leftarm` 与 `vision_rightarm` 两个 ROS 2 包在 Ubuntu 工控机上的构建、启动和验收方法，并为后续增加 `systemd` 开机自启动保留维护入口。

> 当前状态（2026-08-04）：左右臂源码已由用户在工控机上完成运行验收；本文中的 Python 语法、ROS 包名、launch 文件、Topic、Service、模型安装项和 XML/CMake 配置已在 Windows 源码副本静态核对。本文没有把尚未完成的开机自启动写成已验证功能。

## 1. 系统链路

```text
Gemini 305 相机
  -> Orbbec ROS 2 Wrapper
  -> RGB/Depth Topic
  -> 左/右臂 YOLO 分割服务
  -> 深度与三维后处理
  -> DetectionResult Service/Client
```

## 2. 部署信息

工控机 ROS 2 工作空间：

```text
/home/caseim/comorobot_ws
```

源码包路径：

```text
/home/caseim/comorobot_ws/src/comorobot_vision-dev/vision_leftarm
/home/caseim/comorobot_ws/src/comorobot_vision-dev/vision_rightarm
```

| 项目 | 左臂 | 右臂 |
| --- | --- | --- |
| ROS 包 | `vision_leftarm` | `vision_rightarm` |
| 相机型号 | Orbbec Gemini 305 | Orbbec Gemini 305 |
| 相机序列号 | `CV2T661000VN` | `CV2T66100016` |
| 相机名称 | `left_arm_rgbd` | `right_arm_rgbd` |
| 彩色 Topic | `/vision/camera/left_arm_rgbd/color/image_raw` | `/vision/camera/right_arm_rgbd/color/image_raw` |
| 深度 Topic | `/vision/camera/left_arm_rgbd/depth/image_raw` | `/vision/camera/right_arm_rgbd/depth/image_raw` |
| 检测 Service | `/leftarm/yolo_detection` | `/vision/rightarm/yolo_detection` |
| 服务输出单位 | `width/height/coord` 为毫米 | `width/height/coord` 为米 |
| 默认推理尺寸 | `imgsz=960` | ROS 参数 `inference_size=640` |

相机 launch 的共同配置为：彩色图 `1280x800 @ 10 FPS`、MJPG、深度对齐彩色图、硬件 D2C、关闭点云和设备帧同步。

> 左臂 Service 使用相对名称 `leftarm/yolo_detection`。按源码直接运行且没有额外 namespace 时，完整名称为 `/leftarm/yolo_detection`；如果部署命令额外设置了 ROS namespace，应以 `ros2 service list` 的实机输出为准。

## 3. 依赖

基础依赖包括：

- ROS 2 与本仓库的 `orbbec_camera` 包；
- `rclpy`、`sensor_msgs`、`message_filters`、`launch`、`launch_ros`；
- Python 3、NumPy、OpenCV、PyTorch；
- 两个包各自 `scripts/best.pt` 模型文件。

仓库内随包保存了 Ultralytics 源码，但 PyTorch 及其 CUDA/CPU 运行环境仍需在工控机上单独可用。执行构建前，先确认实际包路径：

```bash
cd /home/caseim/comorobot_ws
colcon list | grep -E '^vision_(leftarm|rightarm)[[:space:]]'
```

不要由包名猜测源码目录，也不要直接修改 `install/` 下的构建产物。

## 4. 构建

```bash
cd /home/caseim/comorobot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select vision_leftarm vision_rightarm
source install/setup.bash
```

每次修改 `src` 下的源码、launch、Service 接口或模型安装配置后，都要重新构建并重新 `source install/setup.bash`。

## 5. 手动启动

以下命令分别在独立终端运行。每个新终端都需要加载 ROS 环境：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/caseim/comorobot_ws/install/setup.bash
```

### 5.1 左臂

启动相机：

```bash
ros2 launch vision_leftarm vision_leftarm_launch.py
```

启动检测服务：

```bash
ros2 run vision_leftarm vision_leftarm_server.py
```

调用客户端：

```bash
ros2 run vision_leftarm vision_leftarm_client.py
```

### 5.2 右臂

启动相机：

```bash
ros2 launch vision_rightarm vision_rightarm_launch.py
```

启动检测服务：

```bash
ros2 run vision_rightarm vision_rightarm_server.py
```

调用客户端：

```bash
ros2 run vision_rightarm vision_rightarm_client.py
```

右臂服务可通过 ROS 参数覆盖模型、置信度和推理尺寸，例如：

```bash
ros2 run vision_rightarm vision_rightarm_server.py --ros-args \
  -p confidence_threshold:=0.25 \
  -p inference_size:=960
```

远距离小目标验收时，应在相同场景、距离、模型和置信度下单独比较推理尺寸。此前实机已确认 `imgsz=960` 能解决因低分辨率缩放导致的远距离漏检问题，但更高尺寸会增加推理开销。

## 6. 验收步骤

### 6.1 相机与 Topic

先确认节点和 Publisher 存在，再确认 Topic 确实持续出帧：

```bash
ros2 node list | grep -E 'left_arm_rgbd|right_arm_rgbd'
ros2 topic hz /vision/camera/left_arm_rgbd/color/image_raw
ros2 topic hz /vision/camera/left_arm_rgbd/depth/image_raw
ros2 topic hz /vision/camera/right_arm_rgbd/color/image_raw
ros2 topic hz /vision/camera/right_arm_rgbd/depth/image_raw
```

检查尺寸、编码和 QoS：

```bash
ros2 topic echo --once /vision/camera/left_arm_rgbd/color/image_raw
ros2 topic echo --once /vision/camera/left_arm_rgbd/depth/image_raw
ros2 topic info -v /vision/camera/right_arm_rgbd/color/image_raw
ros2 topic info -v /vision/camera/right_arm_rgbd/depth/image_raw
```

验收不能只看 Topic 名称存在；至少要确认 Publisher、实际频率、`1280x800` 尺寸、彩色/深度编码和 QoS。

### 6.2 检测服务

```bash
ros2 service list | grep yolo_detection
ros2 service type /leftarm/yolo_detection
ros2 service type /vision/rightarm/yolo_detection
```

分别运行左右臂客户端。通过标准为：

1. Server 已加载正确模型并持续收到同步的 RGB/Depth 帧；
2. 场景中放置已训练类别后，Client 返回非空 `name`；
3. `width`、`height` 和 `coord` 均为合理非空值；
4. 左臂按毫米解释，右臂按米解释；
5. 连续多次调用无模型加载失败、图像陈旧、无效深度或进程退出。

已完成的实机验收记录包括：左臂识别 `yellow_cuboid` 并返回非空毫米坐标；右臂识别 `blue_cylinder` 并返回非空米制宽高和三维坐标。该记录来自用户在工控机上的运行确认。

## 7. 常见故障

### `libusb -6` 或相机无法打开

`libusb -6` 优先按“设备已被其他进程占用”排查。历史上，遗留的 `save_rgb_depth.service` 会启动旧脚本并抢占当前 Gemini 305。检查：

```bash
systemctl status save_rgb_depth.service
ps -ef | grep -E 'save_rgb_depth|orbbec|gemini' | grep -v grep
```

如果确认旧服务就是占用来源，可停用它：

```bash
sudo systemctl disable --now save_rgb_depth.service
```

释放 USB 后，应重启此前初始化失败的 Orbbec launch；已经失败的 pipeline 不会自动恢复。不要把临时 PID、USB Bus/Device 编号或设备 index 写入永久配置。

### Topic 存在但服务没有图像

依次检查：Publisher、Topic 频率、尺寸、编码、时间戳和 QoS。只有 RGB/Depth 同步帧进入服务后，才继续排查模型和三维后处理。

### 近距离可检测、远距离漏检

先核对原始图像尺寸和模型实际 `imgsz`，再比较缩放后目标剩余像素。检测尚无候选框时，不优先排查 D2C、深度或 Service。

## 8. 后续：开机自动启动服务（待实施）

本节暂时只保留实施和验收要求。创建 `systemd` 单元前，先只读核对工控机上头部相机已有服务：

```bash
systemctl list-unit-files | grep -Ei 'vision|orbbec|camera'
systemctl cat <头部相机实际服务名>
systemctl show <头部相机实际服务名> \
  -p User -p Group -p WorkingDirectory -p ExecStart -p Restart -p After -p Wants
```

后续建议将左右臂拆成独立单元，并使用单独的环境加载/启动脚本，明确以下内容：

- 固定的 `User`、`WorkingDirectory` 和 ROS 2 工作空间；
- `/opt/ros/$ROS_DISTRO/setup.bash` 与工作空间 `install/setup.bash`；
- 每路相机绑定明确 SN，不允许找不到设备时回退到任意 Orbbec；
- 相机 launch 与检测 Server 的启动顺序；
- `Restart=on-failure`、合理的重启间隔和日志查看方式；
- 停止时正确结束整个 ROS 进程组，避免残留进程继续占用 USB。

冷启动验收标准：

1. `save_rgb_depth.service` 保持 `disabled/inactive`；
2. 左右臂及头部三路相机均绑定正确 SN，节点名和 namespace 不冲突；
3. 三路 RGB/Depth Topic 均有实际帧频；
4. 左右臂 Client 均返回非空检测与三维坐标；
5. `journalctl` 中没有 `libusb -6`、重复节点、无限重启或残留进程。

完成 systemd 配置后，应把实际 unit 文件、启动脚本、安装命令、回滚命令和一次完整冷启动验收日志补充到本节。

## 9. 维护原则

- 工控机成功运行版本是部署事实的权威来源；同步前先核对两端差异，不用 Windows 副本整目录覆盖工控机。
- 修改前记录相同场景下的基线；一次尽量只改变一个变量。
- “静态检查通过”“节点启动”“请求完成”和“最终识别结果正确”是不同证据等级。
- 任何部署变更最终都要在 Ubuntu 工控机、真实相机和相同验收场景下复测。
