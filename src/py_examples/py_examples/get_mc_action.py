#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
查询机器人运控（MC）当前模式示例程序
功能说明：
  作为 ROS 2 服务客户端，调用 `/aimdk_5Fmsgs/srv/GetMcAction` 服务，
  查询机器人当前处于哪种运动控制模式（例如稳定站立、走跑模式、上肢兼容分体模式等），
  属于纯只读查询，不会对机器人发送任何运动或模式切换指令。
"""

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import GetMcAction
from aimdk_msgs.msg import CommonRequest


class GetMcActionClient(Node):
    """
    运控模式查询客户端类，继承自 ROS 2 基础节点 Node
    """
    def __init__(self):
        super().__init__('get_mc_action_client')

        # 创建 ROS 2 服务客户端（Client）
        # 服务类型: GetMcAction（获取运控动作/模式）
        # 服务名称: '/aimdk_5Fmsgs/srv/GetMcAction'
        self.client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
        self.get_logger().info('✅ GetMcAction 客户端节点创建成功。')

        # 等待服务端（MC 运控节点）上线就绪，每次超时 2.0 秒
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ 服务未就绪，正在等待服务端上线...')

        self.get_logger().info('🟢 服务端已就绪，准备发送查询请求。')

    def send_request(self):
        """
        构造并发送模式查询请求，解析返回的运控模式信息
        """
        # 1. 构造请求对象与通用请求体
        request = GetMcAction.Request()
        request.request = CommonRequest()

        self.get_logger().info('📨 正在发送请求以获取机器人运控模式...')

        # 2. 异步调用服务，带重试机制（最多重试 8 次，每次等待 0.25 秒）
        for i in range(8):
            # 刷新请求时间戳
            request.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            # 若已收到响应则跳出重试循环
            if future.done():
                break

            self.get_logger().info(f'正在尝试连接服务端... [{i}]')

        # 3. 提取服务响应结果
        response = future.result()
        if response is None:
            self.get_logger().error('❌ 服务调用失败或超时。')
            return

        # 4. 解析并打印当前运控模式信息
        # - action_desc: 模式名称，例如:
        #     STAND_DEFAULT (稳定站立)
        #     LOCOMOTION_DEFAULT (走跑模式)
        #     UPPERBODY_REMOTE_SPLIT (上肢兼容分体模式，可控制手部)
        # - status.value: 运行状态，0=空闲(IDLE), 100=运行中(RUNNING), 200=切换中(TRANSITION)
        self.get_logger().info('✅ 成功获取机器人运控模式：')
        self.get_logger().info(f'模式名称 (Mode name): {response.info.action_desc}')
        self.get_logger().info(f'模式状态 (Mode status): {response.info.status.value}')


def main(args=None):
    """
    主程序入口：初始化 ROS 2 节点，发送请求后清理退出
    """
    rclpy.init(args=args)
    node = None
    try:
        node = GetMcActionClient()
        node.send_request()
    except KeyboardInterrupt:
        # 捕获用户 Ctrl+C 信号平稳退出
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'程序发生异常退出: {e}')

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
