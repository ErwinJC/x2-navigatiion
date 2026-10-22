#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
【节点功能说明】
本节点为 OmniHand 灵巧手触觉传感器（Touch Sensor）监听与手部命令发布示例节点。

主要工作流程：
1. 【触觉数据订阅】订阅话题 `/aima/hal/joint/hand/state` (HandStateArray)，高频接收灵巧手反馈，
   打印左右手手掌、手背以及五指触觉压力点阵阵列（每个触觉单元为 0~255 的压力离散值）。
2. 【手部姿态发布】通过定时器（每秒 1 次）向话题 `/aima/hal/joint/hand/command` (HandCommandArray)
   发布手部关节控制指令，控制灵巧手各手指回到初始 0 位。

【适用硬件说明】
- 点阵触觉数据（Touch Sensors）仅在配备 T2.1 定制触觉传感器的灵巧手上有效；
- 若为标准版 OmniHand（仅电机编码器），底层消息主要包含 left_hands / right_hands 电机状态。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import HandStateArray, HandCommandArray, HandType


class HandStateSubscriber(Node):
    """
    OmniHand 灵巧手状态监听与命令发布节点类
    """
    def __init__(self):
        super().__init__('hand_state_subscriber')

        # -------------------------------------------------------------------------
        # 1. 创建手部命令发布者 (Publisher)
        # -------------------------------------------------------------------------
        # 目标话题：/aima/hal/joint/hand/command
        # 消息类型：HandCommandArray（灵巧手多关节目标控制指令数组）
        # 队列深度：10
        self.publisher_ = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        # -------------------------------------------------------------------------
        # 2. 配置通信服务质量策略 (QoS Profile)
        # -------------------------------------------------------------------------
        # 使用 BEST_EFFORT（尽力传输模式）：
        # 机器人底层高频传感器（如 50Hz 的关节/触觉反馈）数据量大、实时性要求高。
        # 尽力传输模式下网络拥堵时丢弃老包而不是阻塞重发，保证最新状态的极低延迟。
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # -------------------------------------------------------------------------
        # 3. 创建手部状态订阅者 (Subscriber)
        # -------------------------------------------------------------------------
        # 监听话题：/aima/hal/joint/hand/state
        # 消息类型：HandStateArray（包含左右手各自由度电机状态及触觉点阵阵列）
        self.subscription = self.create_subscription(
            HandStateArray,
            '/aima/hal/joint/hand/state',
            self.topic_callback,
            qos_profile
        )

        # -------------------------------------------------------------------------
        # 4. 创建定时器 (Timer)
        # -------------------------------------------------------------------------
        # 周期：1.0 秒触发一次，执行 self.publish_command 发布手部归零指令
        self.timer = self.create_timer(1.0, self.publish_command)

        self.get_logger().info(
            "Subscriber started, listening to /aima/hal/joint/hand/state topic..."
        )

    def topic_callback(self, msg):
        """
        手部状态消息回调函数：当收到底层 HAL 发布的 HandStateArray 时自动触发
        :param msg: HandStateArray 结构体，包含时间戳、左右手类型、电机状态和触觉数据
        """
        # 打印 ROS 2 消息头信息（包含帧序号 sequence 与物理采集纳秒时间戳）
        self.get_logger().info(
            f"Message received - Sequence: {msg.header.sequence}, "
            f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

        # 打印左手触觉传感器数据（若硬件/消息定义包含触觉点阵）
        self.print_touch_sensor_data("Left Hand", msg.left_touch_sensors)

        # 打印右手触觉传感器数据（若硬件/消息定义包含触觉点阵）
        self.print_touch_sensor_data("Right Hand", msg.right_touch_sensors)

        print()

    def print_touch_sensor_data(self, hand_name, sensor_data):
        """
        解析并格式化打印单只手的触觉传感器压力阵列数据
        :param hand_name: 机械手标识字符串（'Left Hand' 或 'Right Hand'）
        :param sensor_data: HandTouchSensorData 触觉阵列对象
        """
        print(f"=== {hand_name} Touch Sensor Data ===")

        # 1. 手掌触觉数据：共 36 个触觉压敏单元点阵 (uint8_t, 范围 0~255)
        print("Palm Touch Data (36 elements): ", end="")
        self.print_array(sensor_data.palm_touch_data)

        # 2. 手背触觉数据：共 36 个触觉压敏单元点阵 (uint8_t, 范围 0~255)
        print("Back of Hand Touch Data (36 elements): ", end="")
        self.print_array(sensor_data.back_of_hand_touch_data)

        # 3. 大拇指触觉数据：指腹共 16 个触觉单元点阵
        print("Thumb Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.thumb_touch_data)

        # 4. 食指触觉数据：指腹共 16 个触觉单元点阵
        print("Index Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.index_finger_touch_data)

        # 5. 中指触觉数据：指腹共 16 个触觉单元点阵
        print("Middle Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.middle_finger_touch_data)

        # 6. 无名指触觉数据：指腹共 16 个触觉单元点阵
        print("Ring Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.ring_finger_touch_data)

        # 7. 小拇指触觉数据：指腹共 16 个触觉单元点阵
        print("Little Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.little_finger_touch_data)

    def print_array(self, arr):
        """
        辅助打印函数：整齐格式化输出 uint8_t 整数数组
        :param arr: 数组或列表 (List/Tuple[int])
        """
        print("[", end="")
        for i, val in enumerate(arr):
            print(f"{val:3d}", end="")
            if i < len(arr) - 1:
                print(" ", end="")
        print("]")

    def publish_command(self):
        """
        定时器触发函数：组装 HandCommandArray 消息并发布到灵巧手控制底层
        """
        message = HandCommandArray()

        # 设置消息头的时间戳与参考坐标系
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "hand_command"

        # 设置手部类型为灵巧手 (NIMBLE_HANDS，枚举值为 1)
        message.left_hand_type.value = HandType.NIMBLE_HANDS
        message.right_hand_type.value = HandType.NIMBLE_HANDS

        from aimdk_msgs.msg import HandCommand

        # 【单手 10 个自由度电机名称说明 (URDF 标准)】
        # - roll: 旋转/对掌副（控制拇指与其它手指相对旋转）
        # - abad: 外展/内收副（Abduction/Adduction，手指左右并拢与张开）
        # - mcp:  掌指关节弯曲（Metacarpophalangeal Joint）
        # - pip:  近端指间关节弯曲（Proximal Interphalangeal Joint）
        left_names = [
            "L_thumb_roll_joint",   # 拇指内旋/对掌
            "L_thumb_abad_joint",   # 拇指张开/侧摆
            "L_thumb_mcp_joint",    # 拇指掌指弯曲
            "L_index_abad_joint",   # 食指侧摆
            "L_index_pip_joint",    # 食指弯曲
            "L_middle_pip_joint",   # 中指弯曲
            "L_ring_abad_joint",    # 无名指侧摆
            "L_ring_pip_joint",     # 无名指弯曲
            "L_pinky_abad_joint",   # 小指侧摆
            "L_pinky_pip_joint"     # 小指弯曲
        ]
        # 右手电机名称：将 'L_' 替换为 'R_' 即可
        right_names = [n.replace("L_", "R_", 1) for n in left_names]

        def build_cmd(name):
            """
            构建单个自由度电机的控制命令结构体
            :param name: 电机关节名称
            :return: HandCommand 对象
            """
            cmd = HandCommand()
            cmd.name = name
            cmd.position = 0.0      # 目标位置 (单位: rad 弧度，0.0 表示完全伸展张开状态)
            cmd.velocity = 0.1      # 最大期望角速度 (单位: rad/s，平缓运动速度)
            cmd.acceleration = 0.0  # 期望加速度 (0 表示使用系统默认平滑加减速)
            cmd.deceleration = 0.0  # 期望减速度 (0 表示使用系统默认平滑加减速)
            cmd.effort = 0.0        # 期望前馈力矩限制 (单位: Nm，0 表示使用默认限流)
            return cmd

        # 分别组装左手与右手的 10 个自由度命令列表（总计 20 个电机）
        message.left_hands = [build_cmd(n) for n in left_names]
        message.right_hands = [build_cmd(n) for n in right_names]

        # 发布手部控制命令到 /aima/hal/joint/hand/command
        self.publisher_.publish(message)
        self.get_logger().info("Published hand command")


def main(args=None):
    """
    程序主入口：初始化 ROS 2、创建节点并启动事件循环
    """
    rclpy.init(args=args)
    node = HandStateSubscriber()
    try:
        # 进入事件循环，持续处理消息接收与定时器触发
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获用户在终端按下的 Ctrl+C，优雅中断退出
        pass
    finally:
        # 销毁节点对象并安全关闭 ROS 2 通信上下文
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
