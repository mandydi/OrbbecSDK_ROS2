#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from vision_leftarm.srv import DetectionResult

class YOLODetectionClient(Node):
    def __init__(self):
        super().__init__('leftarm_yolo_detection_client')
        # 创建客户端
        self.client = self.create_client(DetectionResult, 'leftarm/yolo_detection')
        # 等待服务启动
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待服务端启动...')
        self.request = DetectionResult.Request()

    # 发送请求
    def send_request(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = YOLODetectionClient()
    response = client.send_request()

    # 解析为你需要的字典列表格式
    detect_result = []
    for i in range(len(response.names)):
        detect_result.append({
            'name': response.names[i],
            'width': response.widths[i],
            'height': response.heights[i],
            'coord': [
                response.coords_x[i],
                response.coords_y[i],
                response.coords_z[i]
            ]
        })

    # 打印最终结果（和你提供的格式完全一样）
    print("检测结果（width/height/coord 单位：mm）：")
    print(detect_result)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
