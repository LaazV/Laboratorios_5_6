import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose as TurtlePose
# from nav_msgs.msg import Odometry
from math import sqrt, pow
from random import choice
import time


class TurtleGoal(Node):
    def __init__(self):
        super().__init__("turtle_goal")

        self.goal_set = [
            Pose2D(x=8.3, y=3.7),
            Pose2D(x=7.4, y=3.4),
            Pose2D(x=2.6, y=7.1),
            Pose2D(x=7.6, y=2.5),
            Pose2D(x=5.8, y=2.7),
            Pose2D(x=2.5, y=3.0),
            Pose2D(x=9.1, y=9.1),
            Pose2D(x=8.0, y=3.3),
            Pose2D(x=3.4, y=6.2),
            Pose2D(x=4.0, y=5.6),
            Pose2D(x=0.8, y=6.3),
            Pose2D(x=1.1, y=4.3),
            Pose2D(x=9.9, y=2.0),
            Pose2D(x=6.6, y=6.4),
            Pose2D(x=3.2, y=6.8),
        ]
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.x = 0.0
        self.y = 0.0
        self.drift = 0.0
        self.MAX_DISTANCE = 0.5

        # Inicializa o publisher de novos objetivos
        self.goal_publisher = self.create_publisher(
            msg_type=Pose2D,
            topic='/goal',
            qos_profile=10
        )

        # Inicializa o subscriber de posição
        self.position_subscriber = self.create_subscription(
            # msg_type=Odometry,
            # topic='/odom',
            msg_type=TurtlePose,
            topic='/turtle1/pose',
            callback=self.position_callback,
            qos_profile=10
        )

        # Inicializa timer para monitorar o status
        INTERVALO_SEGUNDOS = 2
        self.timer = self.create_timer(INTERVALO_SEGUNDOS, self.timer_callback)

        self.publish_new_goal()

    def timer_callback(self):
        self.get_logger().info(
            f"Atual: x={self.x:.1f},y={self.y:.1f}, Objetivo: "
            f"x={self.current_goal_x},y={self.current_goal_y}, "
            f"Distância: {self.drift}")
        self.goal_publisher.publish(self.goal)

    def position_callback(self, msg):
        # self.get_logger().info(f"Posição recebida {msg}")
        self.x = msg.x
        self.y = msg.y

        diff_x = self.current_goal_x - self.x
        diff_y = self.current_goal_y - self.y

        self.drift = sqrt(pow(diff_x, 2) + pow(diff_y, 2))
        # self.get_logger().info(f"{self.drift = }")

        if self.drift <= self.MAX_DISTANCE:
            self.get_logger().info("Objetivo atingido. Gerando novo objetivo.")
            time.sleep(1)
            self.publish_new_goal()

    def publish_new_goal(self):
        self.goal = choice(self.goal_set)
        self.current_goal_x = self.goal.x
        self.current_goal_y = self.goal.y
        self.get_logger().info(f"Novo objetivo: x={self.goal.x},y={self.goal.y}")
        self.goal_publisher.publish(self.goal)


def main():
    rclpy.init()
    turtle_goal = TurtleGoal()
    rclpy.spin(turtle_goal)
    turtle_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()