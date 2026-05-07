from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from epuck2_sim_real_control.phase1_architecture import build_phase1_architecture
from epuck2_sim_real_control.phase2_architecture import build_phase2_architecture
from epuck2_sim_real_control.session_manifest import DEFAULT_MANIFEST, load_manifest


class ProjectInfoNode(Node):
    def __init__(self) -> None:
        super().__init__('epuck2_project_info')
        self.declare_parameter('mode', DEFAULT_MANIFEST.mode)
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('publish_topic', '/epuck2/control_mode')

        manifest_path = str(self.get_parameter('manifest_path').value).strip()
        if manifest_path:
            manifest = load_manifest(manifest_path)
        else:
            manifest = DEFAULT_MANIFEST

        mode_override = str(self.get_parameter('mode').value).strip()
        if mode_override:
            manifest.mode = mode_override

        phase1 = build_phase1_architecture(manifest)
        phase2 = build_phase2_architecture(manifest)
        self.publisher = self.create_publisher(String, str(self.get_parameter('publish_topic').value), 10)
        message = String()
        message.data = json.dumps(
            {
                'manifest': manifest.to_dict(),
                'phase1_architecture': phase1.to_dict(),
                'phase2_architecture': phase2.to_dict(),
            },
            sort_keys=True,
        )
        self.publisher.publish(message)

        self.get_logger().info('epuck2 sim-real project scaffold ready')
        self.get_logger().info(f'project info: {message.data}')

        self.create_timer(0.2, self._shutdown_once)

    def _shutdown_once(self) -> None:
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ProjectInfoNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
