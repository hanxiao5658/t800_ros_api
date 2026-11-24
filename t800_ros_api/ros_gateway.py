"""T800 ROS2 Topic 缓存与运动状态切换 Service 客户端。

运行环境：Docker3 / ROS2 Humble。

职责：
  - 保留原有 controller / localization 的 Topic 订阅与缓存。
  - 订阅 /my_ros2/motion/motion_state（JSON 格式的标准 String）。
  - 新增 /my_ros1/navigation/state（经 Docker2 ros1_bridge 转发的标准 String）。
  - 保留两个 std_srvs/srv/Trigger 客户端供 FastAPI 调用。

安全边界：
  - 只读订阅状态，不发布机器人运动指令。
  - 运动模式切换由 my_switch_robotState.py 的原有 Service 执行。
  - 本文件不创建定位重定位 Service 客户端。
"""

import json
import re
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


# 缓存键 -> (ROS Topic, 消息类, ROS 类型字符串, 缓存过期提示阈值/秒)。
# 前五个 Topic 与当前 GitHub 版本保持一致，只新增最后一项导航状态。
# /my_ros1/navigation/state 为 ROS1 锁存式状态发布（事件驱动，不是心跳）。
# 因此这里设置 300 秒作为「缓存很久没更新」的提示阈值，而非 2~4 秒。
# stale=False 仅表示接收时间在阈值内，并不证明导航安全/通信持续健康。
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
    'navigation/state': (
        '/my_ros1/navigation/state', String, 'std_msgs/msg/String', 300.0
    ),
}


# HTTP POST API 会映射到以下两个已有 ROS2 Trigger Service。
MOTION_SWITCH_SERVICES = {
    'set_to_walking': '/my_ros2/switch_motion_srv/set_to_walking',
    'set_to_lowerBodyBalance': '/my_ros2/switch_motion_srv/set_to_lowerBodyBalance',
}


# 只匹配被空白分隔的 key= 起点；即使末尾 fault 值中包含空格，
# 也能保留其完整内容，不必猜测故障原因的字段长度。
_NAV_FIELD_START = re.compile(r'(?<!\S)([A-Za-z_][A-Za-z0-9_]*)=')


def parse_navigation_state(raw):
    """解析 waypoint_controller.py 发布的导航状态字符串。

    ROS1 原文格式：
      state=WAIT_FOR_NEXT waypoint=2/5 id=wp2 obstacle=CLEAR
      localization=TRACKING_FIXED_GICP fault=NONE

    返回字典中的原始字段保持字符串类型；对 waypoint=N/M 额外提供
    waypoint_index 和 waypoint_total 两个整数，方便后端展示进度。
    解析仅供展示，不能用于替代控制器自身的安全联锁判断。
    """
    matches = list(_NAV_FIELD_START.finditer(raw))
    parsed = {}
    for i, match in enumerate(matches):
        end = matches[i + 1].start() if i + 1 < len(matches) else len(raw)
        parsed[match.group(1)] = raw[match.end():end].strip()

    waypoint = parsed.get('waypoint', '')
    parts = waypoint.split('/')
    if len(parts) == 2:
        try:
            index, total = int(parts[0]), int(parts[1])
        except ValueError:
            pass
        else:
            if index >= 0 and total > 0:
                parsed['waypoint_index'] = index
                parsed['waypoint_total'] = total
    return parsed


class RosGateway(Node):
    def __init__(self):
        super().__init__('t800_ros_api_gateway')
        self.cache = LatestMessageCache()

        # 与仓库原版一致：BEST_EFFORT 订阅可以匹配 RELIABLE 或
        # BEST_EFFORT 发布者；VOLATILE 不请求持久化历史数据。
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 用列表保存订阅对象，避免 Python 垃圾回收影响订阅存活。
        self._api_subscriptions = []
        for key, (topic, msg_cls, msg_type, _) in TOPICS.items():
            subscription = self.create_subscription(
                msg_cls,
                topic,
                partial(self._on_message, key, topic, msg_type),
                qos,
            )
            self._api_subscriptions.append(subscription)

        # 创建客户端不等于调用 Service，不会切换机器人状态。
        self.motion_clients = {
            key: self.create_client(Trigger, name)
            for key, name in MOTION_SWITCH_SERVICES.items()
        }

        self.get_logger().info(
            'Listening to %d topics; %d motion switch clients are ready for discovery'
            % (len(TOPICS), len(self.motion_clients))
        )

    def _on_message(self, key, topic, msg_type, msg):
        """保留 ROS 原始消息，并为指定的 String Topic 增加 parsed 字段。"""
        try:
            converted = dict(message_to_ordereddict(msg))
            parsed = None

            if key == 'controller/status':
                # 保持原有控制器状态解析逻辑不变。
                parsed = parse_controller_status(msg.data)

            elif key == 'motion/motion_state':
                # 运动状态 Topic 的 String.data 是 JSON 文本。
                # 原始字符串仍保留在响应的 data.data 中。
                try:
                    decoded = json.loads(msg.data)
                    if isinstance(decoded, dict):
                        parsed = decoded
                except (ValueError, TypeError):
                    self.get_logger().warning(
                        'Motion state String is not valid JSON; returning raw data only'
                    )

            elif key == 'navigation/state':
                # 导航状态由 Docker1 的 waypoint_controller.py 发布，
                # 经 Docker2 ros1_bridge 进入 Docker3。
                # 它不是 JSON，而是空格分隔的 key=value 文本。
                parsed = parse_navigation_state(msg.data)

            self.cache.update(key, topic, msg_type, converted, parsed)

        except Exception as exc:
            self.get_logger().error('Could not serialize %s: %s' % (topic, exc))

    def call_motion_switch(self, action):
        """只允许调用预先登记的 Trigger Service，返回 ROS Future。"""
        client = self.motion_clients[action]
        return client.call_async(Trigger.Request())

    def motion_switch_is_ready(self, action):
        """非阻塞检查对应 ROS2 Service 是否已经被发现。"""
        return self.motion_clients[action].service_is_ready()
