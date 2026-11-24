"""Record perception status events without storing RGB or depth frames."""
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusRecorder(Node):
    def __init__(self, path):
        super().__init__("vira_visual_status_recorder")
        self.output = Path(path).open("x")
        self.create_subscription(String,"/perception/status",self.receive,10)

    def receive(self, message):
        try:
            record = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(record,dict):
            print(json.dumps(record),file=self.output,flush=True)


def main():
    rclpy.init()
    node = StatusRecorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
