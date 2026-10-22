#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OmniHand 灵巧手多关节手势控制示例节点
功能说明：
  通过 ROS 2 话题向机器人硬件抽象层（HAL）发送五指多关节灵巧手的控制指令。
  以 1.0 秒为周期，依次循环演示 10 种不同的手指屈伸动作（从逐指弯曲、握拳到逐指松开、完全展平）。

与普通夹爪的区别：
  - 本节点使用灵巧手模式（HandType.value = 1）。
  - 单手包含 5 根手指、每指 2 个主动自由度，共 10 个独立控制的关节（left/right_hand_joint0 ~ joint9）。
  - 数组共 20 个参数（前 10 个为左手，后 10 个为右手）。
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import HandCommand, HandCommandArray, HandType, MessageHeader


class OmnihandControl(Node):
    """
    五指灵巧手控制节点类，继承自 ROS 2 基础节点 Node
    """
    def __init__(self):
        super().__init__('omnihand_control')

        # 机构自由度参数定义
        self.num_fingers = 5          # 每只手 5 根手指（拇指、食指、中指、无名指、小指）
        self.joints_per_finger = 2    # 每根手指 2 个主动关节
        self.total_joints = self.num_fingers * self.joints_per_finger  # 单手共 10 个主动关节

        # ===== 预设手势序列（10 组手势） =====
        # 每一组包含 20 个关节控制参数：
        #   - 索引 0~9  ：左手 10 个关节目标值
        #   - 索引 10~19：右手 10 个关节目标值
        # 数值物理含义：1.0 = 弯曲/屈指 (bent)；0.0 = 伸直/展平 (straight)
        self.gesture_sequence = [
            # 手势 1：弯曲第 1 根手指（拇指屈曲）
            [-1.0, 1.0, -0.84, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 手势 2：弯曲 2 根手指
            [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 手势 3：弯曲 3 根手指
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            # 手势 4：弯曲 4 根手指
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0],
            # 手势 5：弯曲全部 5 根手指（紧握拳头 Fist）
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            # 手势 6：松开第 1 根手指（小指开始伸直）
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0],
            # 手势 7：松开 2 根手指
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            # 手势 8：松开 3 根手指
            [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 手势 9：松开 4 根手指
            [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 手势 10：全部手指伸直（手掌完全展开 Open Palm）
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]

        self.current_index = 0                       # 当前手势索引（0~9）
        self.last_switch_time = self.get_clock().now()  # 记录上次切换手势的时间

        # 创建 ROS 2 指令发布者
        # 话题名: '/aima/hal/joint/hand/command'
        # 消息类型: HandCommandArray
        self.publisher = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        # 创建定时器：以 1.0 秒为周期执行 publish_hand_commands 回调函数
        self.timer = self.create_timer(
            1.0,
            self.publish_hand_commands
        )

        self.get_logger().info("Omnihand demo control node started!（OmniHand灵巧手控制示例节点已启动）")

    def publish_hand_commands(self):
        """
        定时器回调函数：每秒执行一次，切换手势并发布 20 维关节指令
        """
        now_time = self.get_clock().now()
        # 1. 判断时间间隔，每隔 1.0 秒步进到下一个预设手势
        if (now_time - self.last_switch_time).nanoseconds / 1e9 >= 1.0:
            self.current_index = (self.current_index + 1) % len(self.gesture_sequence)
            self.last_switch_time = now_time
            self.get_logger().info(
                f'Switched to gesture {self.current_index + 1}/10（切换到手势 {self.current_index + 1}/10）'
            )

        # 取出当前手势的 20 维关节数组
        joint_array = self.gesture_sequence[self.current_index]

        # 2. 构造 ROS 2 消息
        msg = HandCommandArray()
        msg.header = MessageHeader()  # 消息头

        # 3. 构造左手 10 个关节的控制指令
        for i in range(self.total_joints):
            cmd = HandCommand()
            cmd.name = f"left_hand_joint{i}"    # 关节命名：left_hand_joint0 ~ left_hand_joint9
            cmd.position = joint_array[i]       # 关节目标位置（0.0 伸直 ~ 1.0 弯曲）
            cmd.velocity = 1.0                  # 速度上限
            cmd.acceleration = 1.0              # 加速度上限
            cmd.deceleration = 1.0              # 减速度上限
            cmd.effort = 1.0                    # 力矩上限
            msg.left_hands.append(cmd)

        # 4. 构造右手 10 个关节的控制指令（偏移量为 total_joints = 10）
        for i in range(self.total_joints):
            cmd = HandCommand()
            cmd.name = f"right_hand_joint{i}"   # 关节命名：right_hand_joint0 ~ right_hand_joint9
            cmd.position = joint_array[self.total_joints + i]  # 右手参数在数组后半段
            cmd.velocity = 1.0
            cmd.acceleration = 1.0
            cmd.deceleration = 1.0
            cmd.effort = 1.0
            msg.right_hands.append(cmd)

        # 5. 指定手部通信协议类型：value = 1 为五指多关节灵巧手模式 (Dexterous Hand Mode)
        msg.left_hand_type = HandType()
        msg.left_hand_type.value = 1
        msg.right_hand_type = HandType()
        msg.right_hand_type.value = 1

        # 6. 发布指令给硬件抽象层（HAL）
        self.publisher.publish(msg)


def main(args=None):
    """
    主程序入口：初始化 ROS 2 节点并运行
    """
    rclpy.init(args=args)
    node = OmnihandControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 中断信号，安全退出
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
