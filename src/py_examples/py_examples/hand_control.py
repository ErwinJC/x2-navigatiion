#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双手机械末端（夹爪/手部）控制示例节点
功能说明：
  通过 ROS 2 话题向机器人硬件抽象层（HAL）发送双手开合控制指令。
  以 50Hz 周期持续发布目标位置，并每隔 2 秒循环切换一组预设的开合动作。

注意区别：
  - 本脚本使用的是夹爪模式（HandType.value = 2，单手 1 个开合度，范围 0.0~1.0）。
  - 如果机器人安装的是五指多关节灵巧手（OmniHand，单手 10 个主动关节），
    请参考同目录下的 omnihand_control.py（使用 HandType.value = 1 灵巧手模式）。
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import HandCommandArray, HandCommand, HandType, MessageHeader
import time


class HandControl(Node):
    """
    手部控制节点类，继承自 ROS 2 的基础节点 Node
    """
    def __init__(self):
        super().__init__('hand_control')

        # 预设的左右手开合比例列表：[(左手开合度, 右手开合度), ...]
        # 数值含义：1.0 表示完全张开，0.0 表示完全闭合，介于 0~1 之间表示中间开度
        self.position_pairs = [
            (1.0, 1.0),   # 动作 0：双手完全张开
            (0.0, 0.0),   # 动作 1：双手完全闭合
            (0.5, 0.5),   # 动作 2：双手中度半开
            (0.2, 0.8),   # 动作 3：左手偏闭合 (0.2)，右手偏张开 (0.8)
            (0.7, 0.3)    # 动作 4：左手偏张开 (0.7)，右手偏闭合 (0.3)
        ]
        self.current_index = 0  # 当前执行的预设动作索引
        self.last_switch_time = self.get_clock().now().nanoseconds / 1e9  # 记录上次切换动作的时间戳（秒）

        # 创建 ROS 2 发布者（Publisher）
        # 话题名: '/aima/hal/joint/hand/command'（硬件抽象层手部指令话题）
        # 消息类型: HandCommandArray（手部指令数组，包含左右手指令和手部类型）
        # 队列深度 (QoS History Depth): 10
        self.publisher_ = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        # 创建定时器（Timer），以 50 Hz（周期 0.02 秒 = 20 毫秒）触发 publish_hand_commands 回调函数
        # 机器人底层驱动通常需要高频周期性心跳指令，否则可能因超时触发安全停止
        self.timer_ = self.create_timer(
            0.02,  # 20 ms = 50 Hz
            self.publish_hand_commands
        )

        self.get_logger().info("Hand control node started!（手部控制示例节点已启动）")

    def publish_hand_commands(self):
        """
        定时器回调函数：每 20ms 执行一次，负责构造并发布手部控制消息
        """
        # 1. 检查运行时间，每隔 2.0 秒切换到下一个预设手势
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_switch_time >= 2.0:
            self.current_index = (self.current_index + 1) % len(self.position_pairs)
            self.last_switch_time = now_sec
            self.get_logger().info(
                f"Switched to preset: {self.current_index}, left={self.position_pairs[self.current_index][0]:.2f}, right={self.position_pairs[self.current_index][1]:.2f}"
            )

        # 2. 读取当前选中的左右手开合比例
        left_position, right_position = self.position_pairs[self.current_index]

        # 3. 构造总手部消息 HandCommandArray
        msg = HandCommandArray()
        msg.header = MessageHeader()  # 消息头（包含时间戳与序列号）

        # 4. 配置左手指令字段（物理含义详述）
        left_hand = HandCommand()
        left_hand.name = "left_hand"                # 左手标识符
        left_hand.position = float(left_position)   # 目标开合位置（0.0 全闭 ~ 1.0 全开）
        left_hand.velocity = 1.0                    # 目标运动速度上限（归一化系数或 rad/s）
        left_hand.acceleration = 1.0                # 目标加速度上限
        left_hand.deceleration = 1.0                # 目标减速度上限
        left_hand.effort = 1.0                      # 最大抓取力矩/力度上限（归一化比例，防止夹碎物体）

        # 5. 配置右手指令字段
        right_hand = HandCommand()
        right_hand.name = "right_hand"              # 右手标识符
        right_hand.position = float(right_position) # 目标开合位置（0.0 全闭 ~ 1.0 全开）
        right_hand.velocity = 1.0                   # 目标运动速度上限
        right_hand.acceleration = 1.0               # 目标加速度上限
        right_hand.deceleration = 1.0               # 目标减速度上限
        right_hand.effort = 1.0                     # 最大抓取力矩/力度上限

        # 6. 设置手部模式并装载数组
        # value=2 表示二指夹爪模式 (gripper mode)；如果是五指灵巧手，则为 value=1
        msg.left_hand_type = HandType(value=2)
        msg.right_hand_type = HandType(value=2)
        msg.left_hands = [left_hand]
        msg.right_hands = [right_hand]

        # 7. 发布控制指令到 ROS 2 总线
        self.publisher_.publish(msg)


def main(args=None):
    """
    主入口函数：初始化 ROS 2 上下文，启动事件循环
    """
    rclpy.init(args=args)
    hand_control_node = HandControl()

    try:
        # 保持节点运行，响应定时器中断
        rclpy.spin(hand_control_node)
    except KeyboardInterrupt:
        # 捕获用户 Ctrl+C 中断信号，平稳退出
        pass
    finally:
        # 清理并销毁节点，释放 ROS 2 通信资源
        hand_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
