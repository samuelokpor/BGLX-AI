
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def install(node, original):
    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    paths = node.create_publisher(
        Path, "/supervised_gap/audit_candidate", qos)
    markers = node.create_publisher(
        MarkerArray, "/supervised_gap/audit_candidates", qos)
    history = []

    def publish():
        message = MarkerArray()
        message.markers = list(history)
        markers.publish(message)

    def audit(path, target, passage):
        marker = Marker()
        marker.header = path.header
        marker.ns = "candidate_geometry"
        marker.id = len(history)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = .025
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.a = .85
        marker.points = [p.pose.position for p in path.poses]
        history.append(marker)
        paths.publish(path)
        publish()
        try:
            result = original(path, target, passage)
        except Exception:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            publish()
            raise
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            publish()
            return result

    # Keep publishers alive for the lifetime of the stage.
    audit.publishers = (paths, markers)
    return audit
