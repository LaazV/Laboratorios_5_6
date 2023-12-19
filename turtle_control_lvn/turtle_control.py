import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String
from turtlesim.msg import Pose as TurtlePose
from math import atan2, sqrt, pow

class Controle(Node):
    """Nó principal do sistema de controle"""

    def __init__(self):
        super().__init__("turtle_control_lvn")

        # Apesar de ter pedido para definir as variáveis no
        # método 'init_variables', defini no __init__ por ser
        # uma boa prática em python, e permitir o gerenciamento
        # mais eficiente de memória.
        # https://peps.python.org/pep-0412/#split-table-dictionaries
        self.MAX_DISTANCE = 0.1
        self.THETA_TOLERANCE = 0.1
        self.atingido = True
        self.x = float
        self.y = float
        self.goal = None
        self.old_goal = None
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta = float
        self.theta_goal = float

        self.vel_publisher = self.init_publisher()
        self.pose_subscriber, self.goal_subscriber = self.init_subscribers()

    def init_publisher(self):
        # print("Publisher")
        return self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )

    def init_subscribers(self):
        """Inicia os subscribers de pose e objetivo"""

        pose_subscriber = self.create_subscription(
            msg_type=TurtlePose,
            topic='/turtle1/pose',
            callback=self.pose_callback,
            qos_profile=10
        )
        # print("Subscriber Pose")
        goal_subscriber = self.create_subscription(
            msg_type=Pose2D,
            topic='/goal',
            callback=self.goal_callback,
            qos_profile=10
        )
        # print("Subscriber Goal")

        return pose_subscriber, goal_subscriber

    def init_variables(self):
        # Ver comentário em __init__
        raise NotImplementedError

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        # print(f"Pose Callback: {self.x = }, {self.y = }, {self.theta = }")
        self.pub_callback()

    def goal_callback(self, msg):
        if self.old_goal is None:
            self.old_goal = msg

        if self.old_goal != msg:
            print(f"Novo objetivo: x={self.x_goal}, y={self.y_goal}")

        self.x_goal = msg.x
        self.y_goal = msg.y
        self.old_goal = msg

    def pub_callback(self):
        msg = Twist()

        diff_x = self.x_goal - self.x
        diff_y = self.y_goal - self.y
        self.theta_goal = atan2(diff_y, diff_x)

        theta_erro = self.theta_goal - self.theta
        distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2))

        if distance <= self.MAX_DISTANCE:
            if not self.atingido:
                print("Objetivo atingido")
                # self.get_logger().info("Objetivo atingido")
                self.atingido = True
            msg.angular.z = 0.0
            msg.linear.x = 0.0
        else:
            self.atingido = False
            if theta_erro > self.THETA_TOLERANCE:
                msg.angular.z = 1.0
            elif theta_erro < -self.THETA_TOLERANCE:
                msg.angular.z = -1.0
            else:
                msg.linear.x = 1.0

        self.vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    turtle_control = Controle()
    rclpy.spin(turtle_control)
    turtle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
