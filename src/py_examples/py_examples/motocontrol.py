#!/usr/bin/env python3
"""
机器人底层关节电机控制示例 (Joint Motor Control)

主要功能：
1. 通过 ROS 2 直接向硬件抽象层（HAL，Hardware Abstraction Layer）发布关节指令
2. 支持四个身体部位的独立控制：头部 (HEAD)、双臂 (ARM)、腰部 (WAIST)、腿部 (LEG)
3. 结合 Ruckig 轨迹规划算法（支持限制最大速度、加速度与 Jerk 加加速度），实现平滑平稳的关节运动
4. 支持设定关节软限位、目标位置、目标速度以及 PID 阻抗参数（stiffness 刚度 / damping 阻尼）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from aimdk_msgs.msg import JointCommandArray, JointStateArray, JointCommand
from std_msgs.msg import Header
import ruckig  # 第三方高阶平滑轨迹插值库（常用于机器人关节平滑启停）
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict
from threading import Lock

# ==================== 1. ROS 2 通信服务质量 (QoS) 配置 ====================
# 订阅端 QoS：BEST_EFFORT（尽力传输，低延迟）、队列深度 10、VOLATILE（不保存历史旧数据）
subscriber_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

# 发布端 QoS：RELIABLE（可靠传输，保证指令不轻易丢包）、队列深度 10
publisher_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)


# ==================== 2. 关节组枚举与关节配置数据结构 ====================
class JointArea(Enum):
    """机器人身体关节分组枚举"""
    HEAD = 'HEAD'    # 头部关节 (2 自由度: pitch 俯仰, yaw 偏摆)
    ARM = 'ARM'      # 双臂关节 (14 自由度: 左臂 7 + 右臂 7)
    WAIST = 'WAIST'  # 腰部关节 (3 自由度: yaw 偏摆, pitch 俯仰, roll 翻滚)
    LEG = 'LEG'      # 腿部关节 (12 自由度: 左腿 6 + 右腿 6)


@dataclass
class JointInfo:
    """
    单个关节的物理信息与控制增益配置
    name: 关节名称（需与 URDF / HAL 驱动中的名称完全一致）
    lower_limit: 关节物理/软限位下界（单位：弧度 rad）
    upper_limit: 关节物理/软限位上界（单位：弧度 rad）
    kp: 关节位置控制刚度比例系数（N·m/rad，值越大越硬抗外力）
    kd: 关节速度控制阻尼微分系数（N·m·s/rad，值越大越平抑震荡）
    """
    name: str
    lower_limit: float
    upper_limit: float
    kp: float
    kd: float


# ==================== 3. 机器人模型关节物理限位与参数字典 ====================
robot_model: Dict[JointArea, List[JointInfo]] = {
    # 腿部 12 关节配置（每条腿 6 自由度：髋 3 + 膝 1 + 踝 2）
    JointArea.LEG: [
        # 左腿关节
        JointInfo("left_hip_pitch_joint", -2.4871, 2.4871, 40.0, 4.0),    # 左髋俯仰 (前后摆动)
        JointInfo("left_hip_roll_joint", -0.12217, 2.9059, 40.0, 4.0),    # 左髋翻滚 (内外展)
        JointInfo("left_hip_yaw_joint", -1.6842, 3.4296, 30.0, 3.0),     # 左髋偏摆 (水平旋转)
        JointInfo("left_knee_joint", 0.026179, 2.1206, 80.0, 8.0),       # 左膝屈伸 (刚度设得较高 80.0)
        JointInfo("left_ankle_pitch_joint", -0.80285, 0.45378, 40.0, 4.0),# 左踝俯仰
        JointInfo("left_ankle_roll_joint", -0.2618, 0.2618, 20.0, 2.0),   # 左踝翻滚
        # 右腿关节
        JointInfo("right_hip_pitch_joint", -2.4871, 2.4871, 40.0, 4.0),   # 右髋俯仰
        JointInfo("right_hip_roll_joint", -2.9059, 0.12217, 40.0, 4.0),   # 右髋翻滚
        JointInfo("right_hip_yaw_joint", -3.4296, 1.6842, 30.0, 3.0),    # 右髋偏摆
        JointInfo("right_knee_joint", 0.026179, 2.1206, 80.0, 8.0),      # 右膝屈伸
        JointInfo("right_ankle_pitch_joint", -0.80285, 0.45378, 40.0, 4.0),# 右踝俯仰
        JointInfo("right_ankle_roll_joint", -0.2618, 0.2618, 20.0, 2.0),  # 右踝翻滚
    ],
    # 腰部 3 关节配置
    JointArea.WAIST: [
        JointInfo("waist_yaw_joint", -3.4296, 2.3824, 20.0, 4.0),        # 腰部水平旋转
        JointInfo("waist_pitch_joint", -0.17453, 0.17453, 20.0, 4.0),    # 腰部前后弯曲
        JointInfo("waist_roll_joint", -0.48869, 0.48869, 20.0, 4.0),     # 腰部左右侧弯
    ],
    # 双臂 14 关节配置（左臂 7 + 右臂 7）
    JointArea.ARM: [
        # 左臂 7 自由度
        JointInfo("left_shoulder_pitch_joint", -2.5569, 2.5569, 20.0, 2.0), # 左肩前后抬起
        JointInfo("left_shoulder_roll_joint", -0.06108, 3.3598, 20.0, 2.0), # 左肩内外展开
        JointInfo("left_shoulder_yaw_joint", -2.5569, 2.5569, 20.0, 2.0),  # 左大臂旋转
        JointInfo("left_elbow_pitch_joint", -2.4435, 0.0, 20.0, 2.0),       # 左肘关节屈伸
        JointInfo("left_elbow_roll_joint", -2.5569, 2.5569, 20.0, 2.0),    # 左前臂自转
        JointInfo("left_wrist_pitch_joint", -0.5585, 0.5585, 20.0, 2.0),   # 左手腕上下俯仰
        JointInfo("left_wrist_yaw_joint", -1.5446, 1.5446, 20.0, 2.0),     # 左手腕左右摆动
        # 右臂 7 自由度
        JointInfo("right_shoulder_pitch_joint", -2.5569, 2.5569, 20.0, 2.0),# 右肩前后抬起
        JointInfo("right_shoulder_roll_joint", -3.3598, 0.06108, 20.0, 2.0),# 右肩内外展开
        JointInfo("right_shoulder_yaw_joint", -2.5569, 2.5569, 20.0, 2.0), # 右大臂旋转
        JointInfo("right_elbow_pitch_joint", 0.0, 2.4435, 20.0, 2.0),      # 右肘关节屈伸
        JointInfo("right_elbow_roll_joint", -2.5569, 2.5569, 20.0, 2.0),   # 右前臂自转
        JointInfo("right_wrist_pitch_joint", -0.5585, 0.5585, 20.0, 2.0),  # 右手腕上下俯仰
        JointInfo("right_wrist_yaw_joint", -1.5446, 1.5446, 20.0, 2.0),    # 右手腕左右摆动
    ],
    # 头部 2 关节配置
    JointArea.HEAD: [
        JointInfo("head_pitch_joint", -0.61087, 0.61087, 20.0, 2.0),       # 头部抬头/低头
        JointInfo("head_yaw_joint", -0.61087, 0.61087, 20.0, 2.0),         # 头部向左/向右扭头
    ],
}


# ==================== 4. 关节控制器核心节点类 ====================
class JointControllerNode(Node):
    """
    关节电机控制节点
    功能：
    1. 订阅对应部位的反馈状态话题（/aima/hal/joint/<area>/state）
    2. 使用 Ruckig 插值算法进行平滑的 S 曲线轨迹规划
    3. 向硬件驱动发布控制命令（/aima/hal/joint/<area>/command）
    """

    def __init__(self, node_name: str, sub_topic: str, pub_topic: str, area: JointArea, dofs: int):
        """
        初始化关节控制器
        :param node_name: ROS 2 节点名称
        :param sub_topic: 状态订阅话题（如 /aima/hal/joint/arm/state）
        :param pub_topic: 控制命令发布话题（如 /aima/hal/joint/arm/command）
        :param area: 所属部位（JointArea 枚举）
        :param dofs: 该部位的自由度数量（如腿部 12，手臂 14）
        """
        super().__init__(node_name)
        self.lock = Lock()
        self.joint_info = robot_model[area]
        self.dofs = dofs

        # 初始化 Ruckig 轨迹规划器（自由度数，控制周期 0.002s 即 500Hz 控制循环）
        self.ruckig = ruckig.Ruckig(dofs, 0.002)
        self.input = ruckig.InputParameter(dofs)
        self.output = ruckig.OutputParameter(dofs)
        self.ruckig_initialized = False

        # 初始化规划器的当前位置、速度与加速度向量
        self.input.current_position = [0.0] * dofs
        self.input.current_velocity = [0.0] * dofs
        self.input.current_acceleration = [0.0] * dofs

        # 运动安全约束限制（防止电机转动过猛导致过冲或打齿）
        self.input.max_velocity = [1.0] * dofs       # 最大角速度：1.0 rad/s
        self.input.max_acceleration = [1.0] * dofs   # 最大角加速度：1.0 rad/s^2
        self.input.max_jerk = [25.0] * dofs          # 最大加加速度 (Jerk)：25.0 rad/s^3

        # 创建 ROS 2 订阅者与发布者
        self.sub = self.create_subscription(
            JointStateArray,
            sub_topic,
            self.joint_state_callback,
            subscriber_qos
        )
        self.pub = self.create_publisher(
            JointCommandArray,
            pub_topic,
            publisher_qos
        )

    def joint_state_callback(self, msg: JointStateArray):
        """
        接收电机反馈的实际关节状态
        """
        # 首次接收到硬件状态后，置位初始化标志
        self.ruckig_initialized = True

    def control_callback(self, joint_idx):
        """
        执行轨迹规划循环并向硬件持续发布控制指令
        :param joint_idx: 本次主要调节的目标关节索引
        """
        # 循环调用 ruckig.update()，逐步生成插值点，直至轨迹规划完成（Result.Finished）
        while self.ruckig.update(self.input, self.output) in [ruckig.Result.Working, ruckig.Result.Finished]:
            # 用新算出的插值更新当前状态，作为下一个微元周期的起始点
            self.input.current_position = self.output.new_position
            self.input.current_velocity = self.output.new_velocity
            self.input.current_acceleration = self.output.new_acceleration

            # 目标到达检查（误差小于容差 1e-6 视为已精准到达）
            tolerance = 1e-6
            current_p = self.output.new_position[joint_idx]
            if abs(current_p - self.input.target_position[joint_idx]) < tolerance:
                break

            # 组装并发布包含该部位所有关节的控制消息
            cmd = JointCommandArray()
            for i, joint in enumerate(self.joint_info):
                j = JointCommand()
                j.name = joint.name
                j.position = self.output.new_position[i]     # 目标角度 (rad)
                j.velocity = self.output.new_velocity[i]     # 前馈速度 (rad/s)
                j.effort = 0.0                               # 前馈力矩 (N·m)
                j.stiffness = joint.kp                       # 位置刚度增益 (Kp)
                j.damping = joint.kd                         # 速度阻尼增益 (Kd)
                cmd.joints.append(j)

            self.pub.publish(cmd)

    def set_target_position(self, joint_name: str, position: float):
        """
        指定单个关节的目标旋转角度并触发规划运动
        :param joint_name: 关节名称（如 "left_knee_joint"）
        :param position: 目标角度（弧度 rad）
        """
        p_s = [0.0] * self.dofs
        joint_idx = 0
        for i, joint in enumerate(self.joint_info):
            if joint.name == joint_name:
                p_s[i] = position
                joint_idx = i

        # 设置 Ruckig 规划目标
        self.input.target_position = p_s
        self.input.target_velocity = [0.0] * self.dofs
        self.input.target_acceleration = [0.0] * self.dofs

        # 开始平滑插值并发送电机控制流
        self.control_callback(joint_idx)


# ==================== 5. 主执行函数 (Main) ====================
def main(args=None):
    """
    程序入口：
    演示如何控制机器人指定部位电机的往复运动
    """
    rclpy.init(args=args)

    # 1. 示例默认演示：创建腿部控制器节点（共 12 个关节）
    leg_node = JointControllerNode(
        "leg_node",
        "/aima/hal/joint/leg/state",
        "/aima/hal/joint/leg/command",
        JointArea.LEG,
        12
    )

    # 如需控制腰部，可取消以下注释：
    # waist_node = JointControllerNode(
    #     "waist_node",
    #     "/aima/hal/joint/waist/state",
    #     "/aima/hal/joint/waist/command",
    #     JointArea.WAIST,
    #     3
    # )

    # 如需控制双臂，可取消以下注释：
    # arm_node = JointControllerNode(
    #     "arm_node",
    #     "/aima/hal/joint/arm/state",
    #     "/aima/hal/joint/arm/command",
    #     JointArea.ARM,
    #     14
    # )

    # 如需控制头部，可取消以下注释：
    # head_node = JointControllerNode(
    #     "head_node",
    #     "/aima/hal/joint/head/state",
    #     "/aima/hal/joint/head/command",
    #     JointArea.HEAD,
    #     2
    # )

    position = 0.8

    def timer_callback():
        """
        定时器回调函数：
        每隔 3 秒周期性改变目标角度，控制关节正反向往复摆动
        """
        nonlocal position
        position = -position
        target_pos = 1.3 + position

        # 示例：让左膝关节 (left_knee_joint) 在 0.5 rad 和 2.1 rad 之间摆动
        leg_node.set_target_position("left_knee_joint", target_pos)

        # 若控制其他部位，例如：
        # arm_node.set_target_position("left_shoulder_pitch_joint", target_pos)
        # waist_node.set_target_position("waist_yaw_joint", target_pos)
        # head_node.set_target_position("head_pitch_joint", target_pos)

    # 每隔 3.0 秒触发一次关节目标更新
    leg_node.create_timer(3.0, timer_callback)

    # 使用多线程执行器驱动节点事件循环
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(leg_node)

    # 如果启用了其他部位节点，需一并添加到执行器中：
    # executor.add_node(waist_node)
    # executor.add_node(arm_node)
    # executor.add_node(head_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 退出清理与销毁节点
        leg_node.destroy_node()
        # waist_node.destroy_node()
        # arm_node.destroy_node()
        # head_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
