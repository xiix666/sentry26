#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import threading
import os

from rm_interfaces.msg import StatusMsg
from geometry_msgs.msg import PoseStamped

class AllAudioNode(Node):
    def __init__(self):
        super().__init__('all_audio_node')

        self.fight_audio = "/home/rps/sentry26/src/yuyin/fight.mp3"
        self.retreat_audio = "/home/rps/sentry26/src/yuyin/retreat.mp3"
        self.go_audio = "/home/rps/sentry26/src/yuyin/go_signal.mp3"

        self.last_stance = -1

        self.status_sub = self.create_subscription(
            StatusMsg,
            '/status_pack',
            self.status_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.get_logger().info('哨兵嗓子启动完成！')

    def _play(self, audio_file):
        if not os.path.exists(audio_file):
            self.get_logger().warn(f'文件不存在: {audio_file}')
            return

        try:
            threading.Thread(
                target=subprocess.run,
                args=(['mpg123', '-q', audio_file],),
                kwargs={"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL},
                daemon=True
            ).start()
        except:
            pass

    def status_callback(self, msg):
        current = msg.sentry_stance

        # 状态不变 → 不播
        if current == self.last_stance:
            return

        # 状态变化才播放
        if current == 1:
            self._play(self.fight_audio)
        elif current == 2:
            self._play(self.retreat_audio)

        self.last_stance = current

    def goal_callback(self, msg):
        self.get_logger().info('收到目标点！出发！')
        self._play(self.go_audio)

def main(args=None):
    rclpy.init(args=args)
    node = AllAudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()