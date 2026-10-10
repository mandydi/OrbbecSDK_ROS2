# ******************************************************************************
#  Copyright (c) 2024 Orbbec 3D Technology, Inc
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# ******************************************************************************
from typing import Union, Any, Optional  # 导入类型注解模块

import cv2  # 导入OpenCV库，用于图像格式转换
import numpy as np  # 导入NumPy库，用于数组处理

from pyorbbecsdk import FormatConvertFilter, VideoFrame, Device  # 导入Orbbec SDK核心类
from pyorbbecsdk import OBFormat, OBConvertFormat, OBSensorType  # 导入Orbbec格式枚举


def is_astra_mini_device(vid: int, pid: int) -> bool:
    """
    判断设备是否为Astra Mini系列
    :param vid: 设备厂商ID（Vendor ID）
    :param pid: 设备产品ID（Product ID）
    :return: 是Astra Mini返回True，否则返回False
    """
    # 匹配Astra Mini的VID/PID组合
    if (vid == 0x2bc5) and (pid == 0x069d or pid == 0x065b or pid == 0x065e):
        return True
    return False


def is_lidar_device(device: Device) -> bool:
    """
    判断设备是否为激光雷达（LiDAR）设备
    :param device: Orbbec设备对象
    :return: 是LiDAR设备返回True，否则返回False
    """
    # 获取设备的传感器列表
    sensor_list = device.get_sensor_list()
    count = sensor_list.get_count()
    # 遍历所有传感器，检查是否包含LiDAR传感器
    for index in range(count):
        sensor_type = sensor_list.get_sensor_by_index(index).get_type()
        if sensor_type == OBSensorType.LIDAR_SENSOR:
            return True
    return False


def yuyv_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    将YUYV格式图像转换为BGR格式（OpenCV默认显示格式）
    :param frame: YUYV格式原始数据（numpy数组）
    :param width: 图像宽度
    :param height: 图像高度
    :return: BGR格式图像数组
    """
    # 重塑数组为(height, width, 2)，适配YUYV格式
    yuyv = frame.reshape((height, width, 2))
    # OpenCV转换YUYV到BGR
    bgr_image = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
    return bgr_image


def uyvy_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    将UYVY格式图像转换为BGR格式
    :param frame: UYVY格式原始数据（numpy数组）
    :param width: 图像宽度
    :param height: 图像高度
    :return: BGR格式图像数组
    """
    # 重塑数组为(height, width, 2)，适配UYVY格式
    uyvy = frame.reshape((height, width, 2))
    # OpenCV转换UYVY到BGR
    bgr_image = cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
    return bgr_image


def i420_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    将I420（YUV420P）格式图像转换为BGR格式
    :param frame: I420格式原始数据（numpy数组）
    :param width: 图像宽度
    :param height: 图像高度
    :return: BGR格式图像数组
    """
    # 拆分I420的Y/U/V通道：Y占height*width，U/V各占1/4
    y = frame[0:height, :]
    u = frame[height:height + height // 4].reshape(height // 2, width // 2)
    v = frame[height + height // 4:].reshape(height // 2, width // 2)
    # 合并Y/U/V通道
    yuv_image = cv2.merge([y, u, v])
    # OpenCV转换I420到BGR
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)
    return bgr_image


def nv21_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    将NV21格式图像转换为BGR格式
    :param frame: NV21格式原始数据（numpy数组）
    :param width: 图像宽度
    :param height: 图像高度
    :return: BGR格式图像数组
    """
    # 拆分NV21的Y/UV通道：Y占height*width，UV占1/2
    y = frame[0:height, :]
    uv = frame[height:height + height // 2].reshape(height // 2, width)
    # 合并Y/UV通道
    yuv_image = cv2.merge([y, uv])
    # OpenCV转换NV21到BGR
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV21)
    return bgr_image


def nv12_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    将NV12格式图像转换为BGR格式
    :param frame: NV12格式原始数据（numpy数组）
    :param width: 图像宽度
    :param height: 图像高度
    :return: BGR格式图像数组
    """
    # 拆分NV12的Y/UV通道：Y占height*width，UV占1/2
    y = frame[0:height, :]
    uv = frame[height:height + height // 2].reshape(height // 2, width)
    # 合并Y/UV通道
    yuv_image = cv2.merge([y, uv])
    # OpenCV转换NV12到BGR
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)
    return bgr_image


def determine_convert_format(frame: VideoFrame):
    """
    根据输入帧的格式，确定对应的RGB转换格式
    :param frame: Orbbec VideoFrame对象
    :return: OBConvertFormat枚举值（转换格式），不支持则返回None
    """
    frame_format = frame.get_format()
    if frame_format == OBFormat.I420:
        return OBConvertFormat.I420_TO_RGB888
    elif frame_format == OBFormat.MJPG:
        return OBConvertFormat.MJPG_TO_RGB888
    elif frame_format == OBFormat.YUYV:
        return OBConvertFormat.YUYV_TO_RGB888
    elif frame_format == OBFormat.NV21:
        return OBConvertFormat.NV21_TO_RGB888
    elif frame_format == OBFormat.NV12:
        return OBConvertFormat.NV12_TO_RGB888
    elif frame_format == OBFormat.UYVY:
        return OBConvertFormat.UYVY_TO_RGB888
    else:
        # 不支持的格式返回None
        return None


def frame_to_rgb_frame(frame: VideoFrame) -> Union[Optional[VideoFrame], Any]:
    """
    将任意格式的VideoFrame转换为RGB格式的VideoFrame
    :param frame: 输入VideoFrame对象
    :return: RGB格式VideoFrame，转换失败返回None
    """
    # 如果已是RGB格式，直接返回
    if frame.get_format() == OBFormat.RGB:
        return frame
    # 确定转换格式
    convert_format = determine_convert_format(frame)
    if convert_format is None:
        print("不支持的帧格式")
        return None
    print(f"转换格式: {convert_format}")

    # 创建格式转换滤波器
    convert_filter = FormatConvertFilter()
    # 设置转换目标格式
    convert_filter.set_format_convert_format(convert_format)
    # 执行转换
    rgb_frame = convert_filter.process(frame)
    if rgb_frame is None:
        print(f"将 {frame.get_format()} 转换为RGB失败")
    return rgb_frame


def frame_to_bgr_image(frame: VideoFrame) -> Union[Optional[np.array], Any]:
    """
    将Orbbec VideoFrame转换为OpenCV兼容的BGR格式图像数组
    :param frame: 输入VideoFrame对象
    :return: BGR格式numpy数组，转换失败返回None
    """
    # 获取帧的基本信息
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    # 将帧数据转换为numpy数组
    data = np.asanyarray(frame.get_data())
    # 初始化BGR图像数组（高度x宽度x3通道，8位无符号整数）
    image = np.zeros((height, width, 3), dtype=np.uint8)

    # 根据不同格式进行转换
    if color_format == OBFormat.RGB:
        # RGB格式：重塑数组后转换为BGR（OpenCV默认）
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif color_format == OBFormat.BGR:
        # BGR格式：重塑数组后转换为RGB（兼容处理）
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif color_format == OBFormat.YUYV:
        # YUYV格式：调用专用转换函数
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
    elif color_format == OBFormat.MJPG:
        # MJPG（MJPEG）格式：OpenCV解码
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    elif color_format == OBFormat.I420:
        # I420格式：调用专用转换函数
        image = i420_to_bgr(data, width, height)
        return image
    elif color_format == OBFormat.NV12:
        # NV12格式：调用专用转换函数
        image = nv12_to_bgr(data, width, height)
        return image
    elif color_format == OBFormat.NV21:
        # NV21格式：调用专用转换函数
        image = nv21_to_bgr(data, width, height)
        return image
    elif color_format == OBFormat.UYVY:
        # UYVY格式：调用专用转换函数
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
    else:
        # 不支持的格式
        print(f"不支持的颜色格式: {color_format}")
        return None
    return image
