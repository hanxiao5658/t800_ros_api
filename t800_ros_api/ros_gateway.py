"""ROS subscriptions and ROS2 Trigger client: no motion publishers."""

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
#from std_srvs.srv import Trigger

from .controller_status import parse_controller_status
from .state import LatestMessageCache


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
}

#RELOCALIZE_SERVICE = '/my_ros1/localization/relocalize'


class RosGateway(Node):
    def __init__(self):
        super().__init__('t800_ros_api_gateway')
        self.cache = LatestMessageCache()
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            # BEST_EFFORT subscribers can receive RELIABLE or BEST_EFFORT publishers.
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._api_subscriptions = []
        for key, (topic, msg_cls, msg_type, _) in TOPICS.items():
            subscription = self.create_subscription(
                msg_cls,
                topic,
                partial(self._on_message, key, topic, msg_type),
                qos,
            )
            self._api_subscriptions.append(subscription)
        #self.relocalize_client = self.create_client(Trigger, RELOCALIZE_SERVICE)
        self.get_logger().info(
            'Listening to %d topics ' % (len(TOPICS))
        )

    def _on_message(self, key, topic, msg_type, msg):
        try:
            # Standard ROS representation, without losing PoseStamped headers.
            converted = dict(message_to_ordereddict(msg))
            parsed = (parse_controller_status(msg.data)
                      if key == 'controller/status' else None)
            self.cache.update(key, topic, msg_type, converted, parsed)
        except Exception as exc:
            self.get_logger().error('Could not serialize %s: %s' % (topic, exc))

    #def start_relocalization(self):
    #    return self.relocalize_client.call_async(Trigger.Request())
