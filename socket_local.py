import rospy
import socket
import json
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
# from tf2_msgs.msg import TFMessage

class ROSToServer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('ros_to_server_node', anonymous=True)

        # 创建 Socket 客户端
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 服务器的 IP 和端口
        self.host = '120.46.61.21'
        self.port = 12345

        # 设置 Socket 超时时间（比如 10 秒）
        self.client_socket.settimeout(10)

        # 订阅 ROS 话题
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # rospy.Subscriber('/tf', TFMessage, self.tf_callback)        

        # 发布 Pose 消息
        self.pose_publisher = rospy.Publisher('/pose', Pose, queue_size=10)

        # 用于保存位姿数据
        self.position = None
        self.orientation = None

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
        self.orientation = {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w
        }

    def odom_callback(self, data):
        # 处理 Odometry 数据
        position = data.pose.pose.position
        self.position = {
            'x': position.x,
            'y': position.y,
            'z': position.z
        }

    # def tf_callback(self, data):
    #     # 处理 TF 数据
    #     for transform in data.transforms:
    #         # 示例：解析 transform 的位置和姿态
    #         translation = transform.transform.translation
    #         rotation = transform.transform.rotation

    #         # 更新位置和姿态
    #         self.position = {
    #             'x': translation.x,
    #             'y': translation.y,
    #             'z': translation.z
    #         }
    #         self.orientation = {
    #             'x': rotation.x,
    #             'y': rotation.y,
    #             'z': rotation.z,
    #             'w': rotation.w
    #         }

    def publish_pose(self):
        # 发布 Pose 消息
        if self.position and self.orientation:
            pose_msg = Pose()
            pose_msg.position.x = self.position['x']
            pose_msg.position.y = self.position['y']
            pose_msg.position.z = self.position['z']
            pose_msg.orientation.x = self.orientation['x']
            pose_msg.orientation.y = self.orientation['y']
            pose_msg.orientation.z = self.orientation['z']
            pose_msg.orientation.w = self.orientation['w']

            self.pose_publisher.publish(pose_msg)
            #rospy.loginfo(f"Published Pose: {pose_msg}")

            # 将 Pose 信息通过 Socket 发送到服务器
            self.send_pose_to_server(pose_msg)

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
    
    def send_pose_to_server(self, pose_msg):
        try:
            # 将 Pose 消息转换为字典
            pose_data = self.pose_to_dict(pose_msg)
            pose_json = json.dumps(pose_data)
            
            # 发送数据
            self.client_socket.send((pose_json + "\n").encode('utf-8'))  # 确保以换行符结束
            #rospy.loginfo(f"Sent Pose data to server: {pose_json}")

        except socket.error as e:
            rospy.logerr(f"Socket error while sending data: {e}")

if __name__ == '__main__':
    try:
        ros_to_server = ROSToServer()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error in main: {e}")

