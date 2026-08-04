#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_leftarm.srv import DetectionResult
from ultralytics import YOLO
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image

# Topic 默认参数，与 save_rgb_depth.py 保持一致
DEFAULT_COLOR_TOPIC = "/vision/camera/left_arm_rgbd/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/vision/camera/left_arm_rgbd/depth/image_raw"
DEFAULT_SYNC_SLOP = 0.05

class YOLODetectionServer(Node):
    def __init__(self):
        super().__init__('leftarm_detection_server')
        # 声明可配置参数（复用save脚本逻辑）
        self.declare_parameter("color_topic", DEFAULT_COLOR_TOPIC)
        self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
        self.declare_parameter("sync_slop", DEFAULT_SYNC_SLOP)
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        sync_slop = float(self.get_parameter("sync_slop").value)

        # 创建服务
        self.srv = self.create_service(DetectionResult, 'leftarm/yolo_detection', self.detection_callback)

        # 加载 YOLO 模型
        self.model_path = "/home/caseim/comorobot_ws/src/comorobot_vision-dev/vision_leftarm/scripts/best.pt"
        self.model = YOLO(self.model_path)

        # 相机内参
        self.fx = 616.0441
        self.fy = 616.1274
        self.cx = 639.7722
        self.cy = 410.8039

        # ========== 新增：同步订阅RGB/Depth Topic 替代读本地文件 ==========
        # QoS 与 Orbbec 驱动匹配（RELIABLE，KEEP_LAST 10）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
        self._depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)

        # 时间同步器
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub], queue_size=10, slop=sync_slop
        )
        self._sync.registerCallback(self._cache_frame_callback)

        # 缓存最新同步图像（服务回调直接读取缓存，不阻塞订阅）
        self.latest_color_bgr = None
        self.latest_depth_u16 = None
        self._frame_lock = False  # 简易标记是否有有效帧缓存

        self.get_logger().info(f"YOLO检测服务启动，订阅：color={color_topic}, depth={depth_topic}, 同步窗口={sync_slop:.3f}s")

    # 同步回调：缓存最新一对对齐的彩色+深度图（照搬save脚本解码逻辑）
    def _cache_frame_callback(self, color_msg: Image, depth_msg: Image):
        try:
            self.latest_color_bgr = self._color_msg_to_bgr(color_msg)
            self.latest_depth_u16 = self._depth_msg_to_u16(depth_msg)
            self._frame_lock = True
        except ValueError as err:
            self.get_logger().warn(f"图像解码失败，丢弃帧: {err}")
            self._frame_lock = False

    # 从save_rgb_depth.py复制的RGB消息解码函数
    @staticmethod
    def _color_msg_to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"不支持的彩色图编码: {msg.encoding!r}")
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size != msg.height * msg.width * 3:
            raise ValueError(f"彩色图数据尺寸不匹配: {arr.size} vs {msg.height}*{msg.width}*3")
        arr = arr.reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr

    # 从save_rgb_depth.py复制的深度消息解码函数
    @staticmethod
    def _depth_msg_to_u16(msg: Image) -> np.ndarray:
        if msg.encoding not in ("16UC1", "mono16"):
            raise ValueError(f"不支持的深度图编码: {msg.encoding!r}")
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        if arr.size != msg.height * msg.width:
            raise ValueError(f"深度图数据尺寸不匹配: {arr.size} vs {msg.height}*{msg.width}")
        return arr.reshape((msg.height, msg.width))

    # 像素坐标转相机坐标，输入深度和输出 X/Y/Z 的单位均为毫米
    def pixel_to_3d(self, x, y, depth_val):
        depth_mm = float(depth_val)
        X = (x - self.cx) * depth_mm / self.fx
        Y = (y - self.cy) * depth_mm / self.fy
        Z = depth_mm
        return X, Y, Z

    # YOLO检测主逻辑：修改为读取缓存帧，不再读取本地png
    def yolo_detect(self):
        # 替换原有读取本地文件逻辑
        if not self._frame_lock or self.latest_color_bgr is None or self.latest_depth_u16 is None:
            self.get_logger().error("无同步RGB/Depth图像缓存，请检查相机话题！")
            return []
        n = 0
        while n < 10:
            try:
                depth_data = self.latest_depth_u16
                color_img = self.latest_color_bgr
                # YOLO推理直接传入numpy数组，不再传文件路径
                results = self.model.predict(source=color_img, imgsz=960, half=False, verbose=False)
                res = results[0]
                detect_list = []
                if res.masks is not None:
                    h, w = res.orig_shape
                    for i, mask in enumerate(res.masks.data):
                        cls_id = int(res.boxes.cls[i])
                        cls_name = res.names[cls_id]
                        mask_np = mask.cpu().numpy()
                        mask_np = cv2.resize(mask_np, (w, h))
                        mask_binary = (mask_np > 0.5).astype(np.uint8) * 255
                        contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if not contours:
                            continue
                        cnt = max(contours, key=cv2.contourArea)
                        M = cv2.moments(cnt)
                        if M["m00"] == 0:
                            continue
                        cx_pix = int(M["m10"] / M["m00"])
                        cy_pix = int(M["m01"] / M["m00"])
                        leftmost = cnt[cnt[:, :, 0].argmin()][0]
                        rightmost = cnt[cnt[:, :, 0].argmax()][0]
                        topmost = cnt[cnt[:, :, 1].argmin()][0]
                        bottommost = cnt[cnt[:, :, 1].argmax()][0]
                        if 0 <= cy_pix < depth_data.shape[0] and 0 <= cx_pix < depth_data.shape[1]:
                            depth_val = depth_data[cy_pix, cx_pix]
                            if depth_val <= 0:
                                continue
                            X, Y, Z = self.pixel_to_3d(cx_pix, cy_pix, depth_val)
                            left_X, _, _ = self.pixel_to_3d(leftmost[0], leftmost[1], depth_val)
                            right_X, _, _ = self.pixel_to_3d(rightmost[0], rightmost[1], depth_val)
                            _, top_Y, _ = self.pixel_to_3d(topmost[0], topmost[1], depth_val)
                            _, bottom_Y, _ = self.pixel_to_3d(bottommost[0], bottommost[1], depth_val)
                            width = abs(right_X - left_X)
                            height = abs(bottom_Y - top_Y)
                            new_z = Z + min(width, height)/3
                            detect_list.append({
                                'name': cls_name,
                                'width': width,
                                'height': height,
                                'coord': [X, Y, new_z]
                            })
                else:
                    self.get_logger().info('未检测到目标！')
                return detect_list
            except Exception as e:
                n += 1
                self.get_logger().warn(f"检测异常重试 {n}/10: {str(e)}")
                continue
        return []

    # 服务回调函数（逻辑基本不变）
    def detection_callback(self, request, response):
        self.get_logger().info("收到检测请求，开始执行...")
        results = self.yolo_detect()
        for obj in results:
            response.names.append(obj['name'])
            response.widths.append(obj['width'])
            response.heights.append(obj['height'])
            response.coords_x.append(obj['coord'][0])
            response.coords_y.append(obj['coord'][1])
            response.coords_z.append(obj['coord'][2])
        self.get_logger().info(f"检测完成，共找到 {len(results)} 个目标")
        return response

def main():
    rclpy.init()
    server = YOLODetectionServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
