import rospy
import socket
import json
import threading
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped

class ROSToServer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('ros_to_server_node', anonymous=True)

        # 创建 Socket 客户端
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 服务器的 IP 和端口
        self.host = '120.46.61.21'
        self.port = 12345
         
         # 本地端口
        self.local_port = 50000  # 固定小车端使用的端口

        # 设置 Socket 超时时间（比如 10 秒）
        self.client_socket.settimeout(10)

        # 绑定本地端口
        self.client_socket.bind(('', self.local_port))  # 绑定本地端口，空字符串表示所有可用的网络接口

        # 订阅 ROS 话题
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)        

        # 发布 Pose 消息
        self.odom_pose_publisher = rospy.Publisher('/pose/odom', Pose, queue_size=10)
        self.amcl_pose_publisher = rospy.Publisher('/pose/amcl_pose', Pose, queue_size=10)
        self.goal_publisher = rospy.Publisher('/goal', Pose, queue_size=10)

        # 用于保存位姿数据
        self.odom_position = None
        self.odom_orientation = None
        self.amcl_position = None
        self.amcl_orientation = None

        # 启动接收数据的线程
        self.receive_thread = threading.Thread(target=self.receive_data_from_server)
        self.receive_thread.daemon = True  # 设置为守护线程
        self.receive_thread.start()

        # 连接到服务器
        self.connect_to_server()

    def connect_to_server(self):
        rospy.loginfo(f"Attempting to connect to the server at {self.host}:{self.port}")

        try:
            # 连接服务器
            self.client_socket.connect((self.host, self.port))
            rospy.loginfo("Successfully connected to server")

            self.run_pose_publisher()  # 启动独立的 Pose 发布函数

        except socket.timeout:
            rospy.logerr("Connection timed out after 10 seconds")
        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            rospy.logerr(f"Error type: {type(e)}")
        finally:
            self.client_socket.close()
            rospy.loginfo("Socket closed")

    def imu_callback(self, data):
        # 处理 IMU 数据
        orientation = data.orientation
        self.odom_orientation = {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w
        }

    def odom_callback(self, data):
        # 处理 Odometry 数据
        position = data.pose.pose.position
        self.odom_position = {
            'x': position.x,
            'y': position.y,
            'z': position.z
        }

    def amcl_callback(self, data):
        # 处理 amcl_pose 数据
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        self.amcl_position = {
            'x': position.x,
            'y': position.y,
            'z': position.z
        }        
        self.amcl_orientation = {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w
        }

    def publish_pose(self):
        # 发布 Pose 消息
        if self.odom_position and self.odom_orientation:
            odom_pose_msg = Pose()
            odom_pose_msg.position.x = self.odom_position['x']
            odom_pose_msg.position.y = self.odom_position['y']
            odom_pose_msg.position.z = self.odom_position['z']
            odom_pose_msg.orientation.x = self.odom_orientation['x']
            odom_pose_msg.orientation.y = self.odom_orientation['y']
            odom_pose_msg.orientation.z = self.odom_orientation['z']
            odom_pose_msg.orientation.w = self.odom_orientation['w']

            self.odom_pose_publisher.publish(odom_pose_msg)

            # 将 Pose 信息通过 Socket 发送到服务器
            self.send_odom_pose_to_server(odom_pose_msg)

        if self.amcl_position and self.amcl_orientation:
            amcl_pose_msg = Pose()
            amcl_pose_msg.position.x = self.amcl_position['x']
            amcl_pose_msg.position.y = self.amcl_position['y']
            amcl_pose_msg.position.z = self.amcl_position['z']
            amcl_pose_msg.orientation.x = self.amcl_orientation['x']
            amcl_pose_msg.orientation.y = self.amcl_orientation['y']
            amcl_pose_msg.orientation.z = self.amcl_orientation['z']
            amcl_pose_msg.orientation.w = self.amcl_orientation['w']

            self.amcl_pose_publisher.publish(amcl_pose_msg)

            # 将 Pose 信息通过 Socket 发送到服务器
            self.send_amcl_pose_to_server(amcl_pose_msg)

    def run_pose_publisher(self):
        # 定期发布 Pose 消息的循环
        rate = rospy.Rate(5)  # 设置发布频率为 5 Hz
        while not rospy.is_shutdown():
            self.publish_pose()
            rate.sleep()

    def pose_to_dict(self, pose_msg):
        """将 Pose 消息转换为字典格式"""
        return {
            "position": {
                "x": pose_msg.position.x,
                "y": pose_msg.position.y,
                "z": pose_msg.position.z
            },
            "orientation": {
                "x": pose_msg.orientation.x,
                "y": pose_msg.orientation.y,
                "z": pose_msg.orientation.z,
                "w": pose_msg.orientation.w
            }
        }

    def send_odom_pose_to_server(self, pose_msg):
        try:
            # 将 Pose 消息转换为字典
            odom_pose_data = self.pose_to_dict(pose_msg)
            # 添加数据类型
            odom_pose_data["type"] = "odom_pose"
            odom_pose_json = json.dumps(odom_pose_data)

            # 发送数据
            self.client_socket.send((odom_pose_json + "\n").encode('utf-8'))  # 确保以换行符结束

        except socket.error as e:
            rospy.logerr(f"Socket error while sending data: {e}")

    def send_amcl_pose_to_server(self, pose_msg):
        try:
            # 将 Pose 消息转换为字典
            amcl_pose_data = self.pose_to_dict(pose_msg)
            # 添加数据类型
            amcl_pose_data["type"] = "amcl_pose"            
            amcl_pose_json = json.dumps(amcl_pose_data)

            # 发送数据
            self.client_socket.send((amcl_pose_json + "\n").encode('utf-8'))  # 确保以换行符结束

        except socket.error as e:
            rospy.logerr(f"Socket error while sending data: {e}")

    def receive_data_from_server(self):
        """接收数据并处理来自服务器的命令"""
        while not rospy.is_shutdown():
            try:
                data = self.client_socket.recv(1024)  # 接收数据
                if data:
                    message = data.decode('utf-8').strip()  # 解码并去除多余的空白字符
                    rospy.loginfo(f"Received message: {message}")

                    # 解析 JSON 数据
                    message_data = json.loads(message)

                    # 根据数据类型进行相应操作
                    if message_data.get("type") == "goal":
                        # 这里你可以根据目标数据控制小车行为
                        rospy.loginfo(f"Received goal data: {message_data}")
                        self.update_goal_position(message_data)
                    else:
                        rospy.logwarn(f"Unknown message type: {message_data.get('type')}")
                else:
                    rospy.logwarn("No data received from server.")
            except socket.error as e:
                rospy.logerr(f"[GOAL] Socket error while receiving data: {e}")
            except json.JSONDecodeError as e:
                rospy.logerr(f"[GOAL] JSON Decode Error: {e}")
            except Exception as e:
                rospy.logerr(f"[GOAL] Unexpected error: {e}")

    def update_goal_position(self, goal_data):
        """处理从服务器接收到的目标数据并发布目标位姿"""
        goal_pose = Pose()
        goal_pose.position.x = goal_data["x"]
        goal_pose.position.y = goal_data["y"]
        goal_pose.position.z = goal_data["z"]
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0

        self.goal_publisher.publish(goal_pose)
        rospy.loginfo(f"Goal updated: {goal_pose}")


if __name__ == '__main__':
    try:
        ros_to_server = ROSToServer()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error in main: {e}")

