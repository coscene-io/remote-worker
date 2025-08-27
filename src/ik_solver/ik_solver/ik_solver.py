#导入库
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration
import math


class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver')

        # 设置关节名称
        self.joint_names = [
            'base_link_to_link1',
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_gripper_link'
        ]

        #初始化参数
        self.init_gripper_angle = None
        self.max_gripper_angle = 1.5
        self.ik_done = True

        # 创建初始化位姿订阅者
        self.initialization_sub = self.create_subscription(
            PoseStamped,
            '/initialization_pose',
            self.init_callback,
            10
        )

        # 创建位姿订阅者
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/sirius_pose',
            self.pose_callback,
            10
        )

        # 创建发布者
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # 创建IK服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待moveit/compute_ik服务 ……')

        self.get_logger().info('ik_solver节点启动完成，等待输入 ……')


    #计算四元数中的自旋（roll）角度
    def extract_roll(self, quat):
        ox, oy, oz, ow = quat
        sinr_cosp = 2 * (ow * ox + oy * oz)
        cosr_cosp = 1 - 2 * (ox**2 + oy**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return roll


    # 初始校准回调：修正自旋角度
    def init_callback(self,msg: PoseStamped):
        ox, oy, oz, ow = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        self.get_logger().info(f"收到初始化四元数: x={ox:.3f}, y={oy:.3f}, z={oz:.3f}, w={ow:.3f}")
        
        self.init_gripper_angle = self.extract_roll((ox, oy, oz, ow))


    # 位姿接受回调：发送IK请求
    def pose_callback(self, msg: PoseStamped):
        if self.ik_done:
            self.ik_done = False

            # 读取目标位置
            x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ox, oy, oz, ow = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
            
            # 构造 IK 请求
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'hand'
            req.ik_request.ik_link_name = 'hand_tcp'
            req.ik_request.pose_stamped.header.frame_id = 'base_link'
            req.ik_request.pose_stamped.pose.position.x = x
            req.ik_request.pose_stamped.pose.position.y = y
            req.ik_request.pose_stamped.pose.position.z = z
            req.ik_request.pose_stamped.pose.orientation.x = ox
            req.ik_request.pose_stamped.pose.orientation.y = oy
            req.ik_request.pose_stamped.pose.orientation.z = oz
            req.ik_request.pose_stamped.pose.orientation.w = ow
            req.ik_request.timeout = Duration(sec=0, nanosec=200000000)#0.2s

            # 异步发送请求
            future = self.ik_client.call_async(req)
            future.add_done_callback(
                lambda f: self.handle_ik_response(f, (ox, oy, oz, ow))
            )
        else:
            pass


    # 获取IK结果，发布关节数据
    def handle_ik_response(self, future, orientation):
        try:
            resp = future.result()
            if resp.error_code.val != 1:
                self.get_logger().warn(f"IK 解算失败，错误码: {resp.error_code.val}")
                self.ik_done = True
                return

            joint_state = resp.solution.joint_state

            # base_link_to_link1 方向
            if 'base_link_to_link1' in joint_state.name:
                idx = joint_state.name.index('base_link_to_link1')

            # 用 orientation.roll 自旋映射到 gripper_angle
            real_angle = abs(self.extract_roll(orientation)-self.init_gripper_angle)*3/3.1415926
            if real_angle >= 3:
                diff_angle = (abs(6-real_angle))
            elif real_angle >= 0:
                diff_angle = (real_angle)

            if diff_angle <= 3/6:
                ex_angle = 0
            elif 3/6 < diff_angle  <= 3/2:
                ex_angle =(diff_angle - 3/6) * 3
            elif diff_angle > 3/2:
                ex_angle = 3
            
            gripper_angle = abs(self.max_gripper_angle - ex_angle/2)

            # 更新 gripper joint 值
            if 'link3_to_gripper_link' in joint_state.name:
                idx = joint_state.name.index('link3_to_gripper_link')
                joint_state.position[idx] = gripper_angle
            else:
                joint_state.name.append('link3_to_gripper_link')
                joint_state.position.append(gripper_angle)

            # 补齐 velocity 和 effort
            joint_state.velocity = [0.0] * len(joint_state.name)
            joint_state.effort = [0.0] * len(joint_state.name)
            joint_state.header.stamp = self.get_clock().now().to_msg()

            self.ik_done = True
            self.joint_pub.publish(joint_state)

        except Exception as e:
            self.get_logger().error(f"处理IK响应失败：{str(e)}。")
            self.ik_done = True


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()