"""T800 ROS2 Topic 缓存与运动状态切换 Service 客户端。

运行环境：Docker3 / ROS2 Humble。

职责：
  - 保留原有 controller / localization 的 Topic 订阅与缓存。
  - 新增 /my_ros2/motion/motion_state 的标准 String 订阅。
  - 新增两个 std_srvs/srv/Trigger 客户端，供 FastAPI 调用。

安全约束：
  - 本节点不发布任何机器人运动指令，也不直接发布厂商 MotionStateRequest。
  - 状态切换只通过 my_switch_robotState.py 中已有的 Service 执行。
  - 不创建 /my_ros1/localization/relocalize 客户端（保持当前仓库行为）。
"""

import json
from functools import partial

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .controller_status import parse_controller_status
from .state import LatestMessageCache


# 格式：缓存键 -> (ROS Topic, ROS 消息类型, 类型名称, 缓存过期阈值/秒)
# 原有四个订阅保持不变，只新增最后一个运动状态 Topic。
TOPICS = {
    'controller/status': (
        '/my_ros2/controller/status', String, 'std_msgs/msg/String', 3.0
    ),
    'localization/status': (
        '/my_ros1/localization/status', String, 'std_msgs/msg/String', 4.0
    ),
    'localization/pose_status': (
        '/my_ros1/localization/pose_status', String, 'std_msgs/msg/String', 4.0
    ),
    'localization/base_footprint_pose': (
        '/my_ros1/localization/base_footprint_pose',
        PoseStamped, 'geometry_msgs/msg/PoseStamped', 2.0
    ),
    'motion/motion_state': (
        '/my_ros2/motion/motion_state', String, 'std_msgs/msg/String', 2.0
    ),
}


# ROS2 服务的完整名称与 HTTP POST API 一一对应。
MOTION_SWITCH_SERVICES = {
    'set_to_walking': '/my_ros2/switch_motion_srv/set_to_walking',
    'set_to_lowerBodyBalance': '/my_ros2/switch_motion_srv/set_to_lowerBodyBalance',
}


class RosGateway(Node):
    def __init__(self):
        super().__init__('t800_ros_api_gateway')
        self.cache = LatestMessageCache()

        # BEST_EFFORT 订阅可与 RELIABLE / BEST_EFFORT 发布者兼容。
        # 保留已有订阅的 QoS，以免影响其他 Topic。
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 必须保存订阅对象引用，避免 Python GC 将订阅回收。
        self._api_subscriptions = []
        for key, (topic, msg_cls, msg_type, _) in TOPICS.items():
            subscription = self.create_subscription(
                msg_cls,
                topic,
                partial(self._on_message, key, topic, msg_type),
                qos,
            )
            self._api_subscriptions.append(subscription)

        # create_client 仅建立客户端，不会改变或调用机器人状态。
        self.motion_clients = {
            key: self.create_client(Trigger, name)
            for key, name in MOTION_SWITCH_SERVICES.items()
        }

        self.get_logger().info(
            'Listening to %d topics; %d motion switch clients are ready for discovery'
            % (len(TOPICS), len(self.motion_clients))
        )

    def _on_message(self, key, topic, msg_type, msg):
        """缓存原始 ROS 内容；若是运动状态，额外解析 String 中的 JSON。"""
        try:
            converted = dict(message_to_ordereddict(msg))
            parsed = None

            if key == 'controller/status':
                # 保留旧代码对状态字符串的诊断解析。
                parsed = parse_controller_status(msg.data)
            elif key == 'motion/motion_state':
                # my_switch_robotState.py 发布的是 JSON 字符串，形如：
                # {"current_motion_task": "rl_basic",
                #  "available_transition_motions": ["lower_body_balance"]}
                # 保留未经修改的 data.data，同时返回 parsed 方便前端使用。
                try:
                    decoded = json.loads(msg.data)
                    if isinstance(decoded, dict):
                        parsed = decoded
                except (ValueError, TypeError):
                    self.get_logger().warning(
                        'Motion state String is not valid JSON; returning raw data only'
                    )

            self.cache.update(key, topic, msg_type, converted, parsed)
        except Exception as exc:
            self.get_logger().error('Could not serialize %s: %s' % (topic, exc))

    def call_motion_switch(self, action):
        """仅允许调用白名单中的两个 Trigger Service，返回 ROS Future。"""
        client = self.motion_clients[action]
        return client.call_async(Trigger.Request())

    def motion_switch_is_ready(self, action):
        """非阻塞检查：service 端是否已经被 ROS2 发现。"""
        return self.motion_clients[action].service_is_ready()
