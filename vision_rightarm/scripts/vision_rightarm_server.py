#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_rightarm.srv import DetectionResult
from ultralytics import YOLO
import cv2
import numpy as np
import os
import time
from sensor_msgs.msg import Image

# Topic 默认参数，与 save_rgb_depth.py 保持一致
DEFAULT_COLOR_TOPIC = "/vision/camera/right_arm_rgbd/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/vision/camera/right_arm_rgbd/depth/image_raw"
DEFAULT_SYNC_SLOP = 0.05

class YOLODetectionServer(Node):
    def __init__(self):
        super().__init__('rightarm_detection_server')
        # 声明可配置参数（复用save脚本逻辑）
        self.declare_parameter("color_topic", DEFAULT_COLOR_TOPIC)
        self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
        self.declare_parameter("sync_slop", DEFAULT_SYNC_SLOP)
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("inference_size", 640)
        self.declare_parameter("diagnostic_image_path", "")
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        sync_slop = float(self.get_parameter("sync_slop").value)
        confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        inference_size = int(self.get_parameter("inference_size").value)
        diagnostic_image_path = str(self.get_parameter("diagnostic_image_path").value)
        if inference_size < 32:
            raise ValueError(f"inference_size必须不小于32，当前值: {inference_size}")

        # 创建服务
        self.srv = self.create_service(DetectionResult, '/vision/rightarm/yolo_detection', self.detection_callback)

        # 加载 YOLO 模型
        configured_model_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = configured_model_path or os.path.join(os.path.dirname(__file__), "best.pt")
        self.confidence_threshold = confidence_threshold
        self.inference_size = inference_size
        self.diagnostic_image_path = diagnostic_image_path.strip()
        self._load_model()

        # 相机内参
        self.fx = 612.848206
        self.fy = 612.879211
        self.cx = 641.527649
        self.cy = 406.177002

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
        self.received_frame_pairs = 0
        self.last_frame_monotonic = None
        self._diagnostic_image_saved = False

        self.get_logger().info(f"YOLO检测服务启动，订阅：color={color_topic}, depth={depth_topic}, 同步窗口={sync_slop:.3f}s")
        self.get_logger().info(
            "诊断已启用：服务请求时会报告模型状态、同步帧数量/帧龄、图像尺寸、"
            f"有效深度比例、推理耗时及候选框数量；YOLO输入尺寸={self.inference_size}"
        )

    def _load_model(self):
        """加载模型并输出足以确认模型身份的诊断信息。"""
        absolute_path = os.path.abspath(self.model_path)
        if not os.path.isfile(absolute_path):
            raise FileNotFoundError(f"YOLO模型不存在: {absolute_path}")

        model_size = os.path.getsize(absolute_path)
        load_started = time.perf_counter()
        try:
            self.model = YOLO(absolute_path)
        except Exception as err:
            self.get_logger().error(f"YOLO模型加载失败: {absolute_path}: {err}")
            raise

        names = getattr(self.model, "names", None)
        if not names:
            raise RuntimeError(f"YOLO模型已打开但未读到类别名称: {absolute_path}")
        elapsed_ms = (time.perf_counter() - load_started) * 1000.0
        self.get_logger().info(
            f"[MODEL_OK] 模型加载成功: path={absolute_path}, size={model_size} bytes, "
            f"task={getattr(self.model, 'task', 'unknown')}, classes={names}, "
            f"load_time={elapsed_ms:.1f} ms"
        )

    # 同步回调：缓存最新一对对齐的彩色+深度图（照搬save脚本解码逻辑）
    def _cache_frame_callback(self, color_msg: Image, depth_msg: Image):
        try:
            self.latest_color_bgr = self._color_msg_to_bgr(color_msg)
            self.latest_depth_u16 = self._depth_msg_to_u16(depth_msg)
            self._frame_lock = True
            self.received_frame_pairs += 1
            self.last_frame_monotonic = time.monotonic()

            if self.received_frame_pairs == 1:
                valid_depth_ratio = float(np.count_nonzero(self.latest_depth_u16)) / self.latest_depth_u16.size
                self.get_logger().info(
                    f"[FRAME_OK] 收到首对同步图像: color={self.latest_color_bgr.shape}, "
                    f"encoding={color_msg.encoding}, mean={self.latest_color_bgr.mean():.1f}; "
                    f"depth={self.latest_depth_u16.shape}, encoding={depth_msg.encoding}, "
                    f"valid_depth={valid_depth_ratio:.1%}"
                )
                self._save_diagnostic_image()
        except ValueError as err:
            self.get_logger().warn(f"图像解码失败，丢弃帧: {err}")
            self._frame_lock = False

    def _save_diagnostic_image(self):
        if not self.diagnostic_image_path or self._diagnostic_image_saved:
            return
        output_path = os.path.abspath(os.path.expanduser(self.diagnostic_image_path))
        output_dir = os.path.dirname(output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        if not cv2.imwrite(output_path, self.latest_color_bgr):
            self.get_logger().error(f"[FRAME_ERROR] 诊断图像保存失败: {output_path}")
            return
        self._diagnostic_image_saved = True
        self.get_logger().info(f"[FRAME_SAVED] 首帧彩色图已保存，可人工核对: {output_path}")

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

    # 像素坐标转 3D 坐标
    def pixel_to_3d(self, x, y, depth_val):
        depth_m = depth_val * 0.001
        X = (x - self.cx) * depth_m / self.fx
        Y = (y - self.cy) * depth_m / self.fy
        Z = depth_m
        return X, Y, Z

    # YOLO检测主逻辑：修改为读取缓存帧，不再读取本地png
    def yolo_detect(self):
        # 替换原有读取本地文件逻辑
        if not self._frame_lock or self.latest_color_bgr is None or self.latest_depth_u16 is None:
            self.get_logger().error(
                f"[FRAME_ERROR] 无同步RGB/Depth图像缓存 (received_pairs={self.received_frame_pairs})，"
                "请检查两个Topic的Publisher、频率、编码、时间戳及QoS"
            )
            return []
        frame_age = time.monotonic() - self.last_frame_monotonic
        if frame_age > 2.0:
            self.get_logger().warn(
                f"[FRAME_STALE] 当前缓存图像已停止更新 {frame_age:.2f}s，"
                f"累计收到 {self.received_frame_pairs} 对同步帧"
            )
        n = 0
        while n < 10:
            try:
                # 拷贝同一时刻的缓存，避免推理期间订阅回调替换数组引用。
                depth_data = self.latest_depth_u16.copy()
                color_img = self.latest_color_bgr.copy()
                self.get_logger().info(
                    f"[FRAME_USED] 使用同步帧 #{self.received_frame_pairs}: "
                    f"color={color_img.shape}, depth={depth_data.shape}, age={frame_age:.3f}s"
                )
                # YOLO推理直接传入numpy数组，不再传文件路径
                inference_started = time.perf_counter()
                results = self.model.predict(
                    source=color_img,
                    imgsz=self.inference_size,
                    conf=self.confidence_threshold,
                    half=False,
                    verbose=False,
                )
                inference_ms = (time.perf_counter() - inference_started) * 1000.0
                if not results:
                    raise RuntimeError("YOLO predict未返回结果对象")
                res = results[0]
                box_count = 0 if res.boxes is None else len(res.boxes)
                self.get_logger().info(
                    f"[INFERENCE_OK] 模型推理完成: time={inference_ms:.1f}ms, "
                    f"conf={self.confidence_threshold:.2f}, boxes={box_count}, "
                    f"masks={0 if res.masks is None else len(res.masks.data)}"
                )
                detect_list = []
                invalid_depth_count = 0
                if res.masks is not None:
                    candidates = [
                        f"{res.names[int(cls_id)]}:{float(conf):.3f}"
                        for cls_id, conf in zip(res.boxes.cls, res.boxes.conf)
                    ]
                    self.get_logger().info(f"[MODEL_DETECTIONS] 模型候选目标: {candidates}")
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
                                invalid_depth_count += 1
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
                    self.get_logger().info(
                        f"[POSTPROCESS_OK] 候选={box_count}, 输出3D目标={len(detect_list)}, "
                        f"中心点无有效深度={invalid_depth_count}"
                    )
                else:
                    self.get_logger().warn(
                        "[NO_DETECTION] 图像已读取且模型已完成推理，但没有分割掩码；"
                        "应继续检查模型类别、置信度、训练数据和目标成像大小"
                    )
                return detect_list
            except Exception as e:
                n += 1
                self.get_logger().warn(f"检测异常重试 {n}/10: {str(e)}")
                continue
        return []

    # 服务回调函数（逻辑基本不变）
    def detection_callback(self, request, response):
        model_names = getattr(self.model, "names", {})
        self.get_logger().info(
            f"收到检测请求: model_loaded={self.model is not None}, classes={model_names}, "
            f"received_pairs={self.received_frame_pairs}"
        )
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
