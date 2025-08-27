# 导入库
import tkinter as tk
import threading
import asyncio
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from SiriusCeptionBin import AsyncCeptionController, SiriusCeptionLib


class PopupManager:
    def __init__(self):
        self.root = tk.Tk()
        self.root.withdraw()  # 隐藏主窗口
        self.popup = None
        self._lock = threading.Lock()

        # Tk 事件循环放在线程里跑
        threading.Thread(target=self.root.mainloop, daemon=True).start()

    def show_popup(self, text="sdk接口卡顿，等待更新恢复中……"):
        with self._lock:
            if self.popup is None or not self.popup.winfo_exists():
                self.popup = tk.Toplevel(self.root)
                self.popup.title("提示")
                self.popup.attributes("-topmost", True)
                tk.Label(self.popup, text=text, padx=20, pady=20).pack()
                self.popup.update()

    def close_popup(self):
        with self._lock:
            if self.popup is not None and self.popup.winfo_exists():
                self.popup.destroy()
                self.popup = None


class MotionCap(Node):
	def __init__(self):
		super().__init__('motion_cap')	

		self.popup_manager = PopupManager()
		
		#初始化参数
		self.scale = 0.98
		self.length1 = 0.1231*self.scale
		self.length2 = 0.2369*self.scale
		self.length3 = 0.28*self.scale
		self.length23 = self.length2+self.length3
		self.length = self.length1+self.length2+self.length3
		self.serial_port = '/dev/ttyUSB0'
		self.base_frame = 'base_link'
		self.MPUReader = None
		self.sirius = None
		self.h_comp_matrix = None
		self.yaw_offset_calibrated = None
		self.max_length = None
		self.min_z = None
		self.latter_positions = [0,1,2,3,4,5,6,7,8,9]
		self.is_send_message = -100
		self.get_logger().info(f"串口：{self.serial_port}； 基准：{self.base_frame}。")
		
		# Create QoS profile compatible with joint_state_publisher
		qos_profile = QoSProfile(
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.VOLATILE,
			depth=10
		)

		# 创建初始校准数据发布者
		self.initialization_publisher = self.create_publisher(
            PoseStamped,
            'initialization_pose',
            qos_profile
        )
		
		# 创建位姿发布者
		self.pose_publisher = self.create_publisher(
			PoseStamped,
			'sirius_pose',
			qos_profile
		)


	# 读取串口数据，处理旋转矩阵,返回位姿
	async def get_pos_and_quat(self):
		try:
            # 用 asyncio.wait_for 加上超时
			pos, rot = await asyncio.wait_for(self.sirius.getEndPos(), timeout=0.2)
			del self.latter_positions[0]
			self.latter_positions.append(pos)
		except asyncio.TimeoutError:
			raise TimeoutError("sirius.getEndPos() 超时")
		except Exception as e:
			# 把原始异常向上抛出，由调用处处理并记录
			raise

		if all(np.array_equal(j, self.latter_positions[0]) for j in self.latter_positions):
				self.is_send_message += 25
				if self.is_send_message > 100:
					self.is_send_message = 100
		else:
			self.is_send_message -= 1
			if self.is_send_message < -100:
				self.is_send_message = -100

		p = np.array([float(pos[0,0]), float(pos[0,1]), float(pos[0,2])]).reshape(3,1)
		t = -np.matmul(rot, p)
		h_matrix = np.vstack((np.hstack((rot, t)), np.array([0,0,0,1]).reshape(1,4)))
		h_new = np.matmul(h_matrix, self.h_comp_matrix)
		new_p = -np.matmul(h_new[0:3,0:3].T, h_new[0:3, 3])

		# compensate for the end effector heading
		r_end = R.from_euler('z', -self.yaw_offset_calibrated, degrees=True).as_matrix()
		h_end_comp_matrix = np.vstack((np.hstack((np.array(r_end), np.zeros(3).reshape(3,1))), np.array([0, 0, 0, 1]).reshape(1,4)))
		h_end = np.matmul(h_end_comp_matrix, h_matrix)

		q = R.from_matrix(h_end[0:3, 0:3]).as_quat()

		return [new_p.reshape(1,3)[0,0],new_p.reshape(1,3)[0,1],new_p.reshape(1,3)[0,2]],q


	#发送初始校准位姿
	def publish_initial_pose(self,pos,quat):
		msg = PoseStamped()
		msg.header.frame_id = "base_link"
		msg.header.stamp = self.get_clock().now().to_msg()

        # 设置位置
		msg.pose.position.x = pos[0]
		msg.pose.position.y = pos[1]
		msg.pose.position.z = pos[2]

        # 设置四元数
		msg.pose.orientation.x = quat[0]
		msg.pose.orientation.y = quat[1]
		msg.pose.orientation.z = quat[2]
		msg.pose.orientation.w = quat[3]

		self.initialization_publisher.publish(msg)


	# 初始校准方法
	async def initialize_motion_tracking(self):
		# 串口连接姿态传感器
		self.get_logger().info("连接姿态传感器串口 ……")
		self.MPUReader = AsyncCeptionController(self.serial_port)
		await self.MPUReader.connect()
		self.sirius = SiriusCeptionLib(self.MPUReader)
		
		# 初始校准
		# 校准步骤一
		await asyncio.sleep(0.5)
		self.get_logger().info("校准 1, 手臂竖直下垂，完全伸直 ……")
		await asyncio.sleep(2.5)
		await self.sirius.calibration()

		# 校准步骤二
		await asyncio.sleep(0.5)
		self.get_logger().info("校准 2, 手臂完全伸直，平举指向正前方，掌心竖直向下 ……")
		await asyncio.sleep(3.5)

		# 偏航角补偿/补偿矩阵
		yaw_angle = []
		for _ in range(100):
			pos, rot = await self.sirius.getEndPos()
			yaw_angle.append(math.atan(pos[0,1]/pos[0,0]))

		# 计算偏航角补偿
		self.yaw_offset_calibrated = math.degrees(np.mean(yaw_angle))
		if pos[0,0] > 0 and pos[0,1] > 0:
			self.yaw_offset_calibrated = self.yaw_offset_calibrated
		elif pos[0,0] < 0 and pos[0,1] > 0:
			self.yaw_offset_calibrated = 90.0 + self.yaw_offset_calibrated
		elif pos[0,0] < 0 and pos[0,1] < 0:
			self.yaw_offset_calibrated = 90.0 + self.yaw_offset_calibrated
		elif pos[0,0] > 0 and pos[0,1] < 0:
			self.yaw_offset_calibrated = self.yaw_offset_calibrated

		# 创建补偿矩阵
		r_matrix = R.from_euler('z', self.yaw_offset_calibrated, degrees=True).as_matrix()
		self.h_comp_matrix = np.vstack((np.hstack((np.array(r_matrix), np.zeros(3).reshape(3,1))), np.array([0, 0, 0, 1]).reshape(1,4)))
		
		# 采集数据
		pos0,rot0 = await self.get_pos_and_quat()

		#发布初始校准位姿
		self.publish_initial_pose(pos0,rot0)

		#校准步骤三
		await asyncio.sleep(0.5)
		self.get_logger().info("校准 3, 手臂完全伸直，手掌平放桌面 ……")
		await asyncio.sleep(3.5)

		#采集数据
		pos1,rot1 = await self.get_pos_and_quat()

		# 处理初始校准数据
		self.max_length = (pos0[0]**2 + pos0[1]**2 + pos0[2]**2)**0.5
		self.min_z = pos1[2]

		self.get_logger().info("初始校准结果：")
		self.get_logger().info(f"偏航角补偿：{self.yaw_offset_calibrated:.2f}度。")
		self.get_logger().info(f"最低z轴坐标: {self.min_z:.2f}。")
		self.get_logger().info(f"最大传感器距离: {self.max_length:.2f}。")
		self.get_logger().info("初始校准完毕，启动主循环 ……")
		await asyncio.sleep(1.5)


	# 映射、打包、发送位姿信息
	async def set_pose(self, pos, rot):
		msg = PoseStamped()
		msg.header.frame_id = self.base_frame
		msg.header.stamp =self.get_clock().now().to_msg()

		# 限制最低活动位置
		if pos[2] < self.min_z + 0.01:
			pos[2] = self.min_z + 0.01

		# 传感器坐标映射机械臂坐标
		if pos[2] == self.max_length:
			msg.pose.position.z = self.length
			msg.pose.position.x = 0
			msg.pose.position.y = 0
		else:
			msg.pose.position.z = (pos[2]-self.min_z)*self.length/(self.max_length-self.min_z)
			if msg.pose.position.z < self.length1:
				msg.pose.position.x = pos[0]*(self.length2+(self.length3**2-self.length1**2)**0.5+(self.length3-(self.length3**2-self.length1**2)**0.5)*msg.pose.position.z/self.length1)/(self.max_length**2-pos[2]**2)**0.5
				msg.pose.position.y = pos[1]*(self.length2+(self.length3**2-self.length1**2)**0.5+(self.length3-(self.length3**2-self.length1**2)**0.5)*msg.pose.position.z/self.length1)/(self.max_length**2-pos[2]**2)**0.5
			else:
				msg.pose.position.x = pos[0]*(self.length23**2-(msg.pose.position.z-self.length1)**2)**0.5/(self.max_length**2-pos[2]**2)**0.5
				msg.pose.position.y = pos[1]*(self.length23**2-(msg.pose.position.z-self.length1)**2)**0.5/(self.max_length**2-pos[2]**2)**0.5

		msg.pose.orientation.x = rot[0]
		msg.pose.orientation.y = rot[1]
		msg.pose.orientation.z = rot[2]
		msg.pose.orientation.w = rot[3]

		# 发送位姿
		self.pose_publisher.publish(msg)


	async def run_motion_tracking(self):
		while rclpy.ok():
			try:
				new_pos,new_rot = await self.get_pos_and_quat()
				if self.is_send_message <= 0:
					self.popup_manager.close_popup() 
					await self.set_pose(new_pos,new_rot)
					rclpy.spin_once(self, timeout_sec=0.001)
				else:
					self.popup_manager.show_popup()
			except KeyboardInterrupt: 
				self.get_logger().error("Process interrupted by user.")
				self.get_logger().error("Simulation Ended")
				break 
			except Exception as e: 
				self.get_logger().error(f"An error occurred: {e}")
				await asyncio.sleep(0.01)
				continue


	async def run(self):
		await self.initialize_motion_tracking()
		await self.run_motion_tracking()


async def task(motion_cap):
	try:
		await motion_cap.run()
	finally:
		# 清除节点
		motion_cap.destroy_node()
		rclpy.shutdown()


def main(): 
	try:
		rclpy.init(args=None)
		motion_cap = MotionCap()
		asyncio.run(task(motion_cap))
	except KeyboardInterrupt: 
		print("Program terminated by user.")
	except Exception as e:
		print(f"Program error: {e}")


if __name__ == "__main__": 
	main()
