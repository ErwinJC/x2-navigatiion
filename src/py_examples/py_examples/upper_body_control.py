#!/usr/bin/env python3
"""
上肢控制示例程序。

使用方法:
  ros2 run py_examples upper_body_control <mode>

  mode 可选模式:
    head    — 头部居中姿态，            需要先切换到 HEAD_ONLY (仅头部) 模式
    claw    — 夹爪半开，                需要先切换到 UPPERBODY_REMOTE_SPLIT (上肢分体遥控) 模式
    joint   — 灵巧手各关节半开，        需要先切换到 UPPERBODY_REMOTE_SPLIT (上肢分体遥控) 模式
    gesture — 灵巧手手势 1，            需要先切换到 UPPERBODY_REMOTE_SPLIT (上肢分体遥控) 模式

运行前请先切换到目标运控 (MC) 模式，例如控制头部前执行:
  ros2 run py_examples set_mc_action SD     # → 切换为默认站立模式 (STAND_DEFAULT)
  ros2 run py_examples set_mc_action HO     # → 切换为仅头部模式 (HEAD_ONLY)
若控制夹爪 (claw) / 灵巧手关节 (joint) / 手势 (gesture) 前执行:
  ros2 run py_examples set_mc_action URS    # → 切换为上肢分体遥控模式 (UPPERBODY_REMOTE_SPLIT)
"""

import sys
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import UpperBodyCommandArray, MessageHeader

# 各控制模式对应的参数配置
# 元组结构: (hand_sub_mode 手部子模式, head_pos 头部位置, arm_pos 双臂位置, hand_pos 手部位置)
MODES = {
    # 头部偏航 head_yaw=0, 头部俯仰 head_pitch=0 (头部居中姿态)
    'head':    (0, [0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                []),
    # 左夹爪开度 left_open=0.5, 右夹爪开度 right_open=0.5 (均为半开状态，范围 0.0~1.0)
    'claw':    (UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE,  # 夹爪开合模式 (1)
                [0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.5, 0.5]),
    # 左手 10 个关节 + 右手 10 个关节（单位：弧度 rad），全部处于半开状态
    # （各关节均处于其 URDF 限位范围的一半）。右手关节参数为左手的镜像对称。
    # 单手关节顺序: [大拇指旋转 thumb_roll, 大拇指外展内收 thumb_abad, 大拇指掌指关节 thumb_mcp, 食指外展内收 index_abad, 食指近端指间关节 index_pip,
    #                中指近端指间关节 middle_pip, 无名指外展内收 ring_abad, 无名指近端指间关节 ring_pip, 小指外展内收 pinky_abad, 小指近端指间关节 pinky_pip]
    'joint':   (UpperBodyCommandArray.HAND_DEXTEROUS_JOINT,  # 灵巧手关节角度控制模式 (2)
                [0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [-0.6, 0.8, -0.4, 0.1, 0.7, 0.7, -0.1, 0.7, -0.1, 0.7,
                 0.6, -0.8, 0.4, -0.1, 0.7, 0.7, 0.1, 0.7, 0.1, 0.7]),
    # 左手手势编号=1, 左手开合度=1.0, 右手手势编号=1, 右手开合度=1.0
    'gesture': (UpperBodyCommandArray.HAND_DEXTEROUS_GESTURE,  # 灵巧手手势控制模式 (3)
                [0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [1.0, 1.0, 1.0, 1.0]),
}


class UpperBodyControlNode(Node):
    """上肢控制 ROS 2 节点类"""

    def __init__(self, mode: str):
        super().__init__('upper_body_control')
        self.hand_sub_mode, self.head_pos, self.arm_pos, self.hand_pos = MODES[mode]
        # 创建发布者，向运控上肢指令话题发布消息，队列长度为 10
        self.pub = self.create_publisher(
            UpperBodyCommandArray, '/mc/upper_body_command', 10)
        # 创建定时器，以 50 Hz 频率周期性调用 publish 发布控制指令
        self.timer = self.create_timer(0.02, self.publish)  # 50 Hz (每 20ms 一次)
        self._seq = 0  # 消息序列号计数器
        self.get_logger().info(
            f'mode={mode}  hand_sub_mode={self.hand_sub_mode}')

    def publish(self):
        """周期性发布上肢控制指令"""
        msg = UpperBodyCommandArray()
        now = self.get_clock().now()
        # 填充消息头部时间戳与坐标系信息
        msg.header = MessageHeader()
        msg.header.stamp.sec = now.nanoseconds // 1_000_000_000
        msg.header.stamp.nanosec = now.nanoseconds % 1_000_000_000
        msg.header.frame_id = 'mc_upper_body'
        msg.header.sequence = self._seq
        self._seq += 1
        # 设置指令来源标识
        msg.source = 'upper_body_example'
        # 填充各部位控制数据
        msg.hand_sub_mode = self.hand_sub_mode
        msg.head_pos = self.head_pos
        msg.arm_pos = self.arm_pos
        msg.hand_pos = self.hand_pos
        # 发布消息
        self.pub.publish(msg)


def main(args=None):
    """主程序入口"""
    rclpy.init(args=args)

    argv = sys.argv[1:]
    # 参数校验：如果未提供模式参数或参数不在支持列表中，则打印提示信息并退出
    if not argv or argv[0] not in MODES:
        print(f'Usage: upper_body_control <{"│".join(MODES)}>')
        rclpy.shutdown()
        return

    node = UpperBodyControlNode(argv[0])
    try:
        # 进入 ROS 2 事件循环
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭 ROS 2 客户端库
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
