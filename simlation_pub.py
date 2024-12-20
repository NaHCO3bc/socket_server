import rospy
import json
import socket
from geometry_msgs.msg import Pose
import random

class SimulatedDataPublisher:
    def __init__(self):
        rospy.init_node('simulated_data_publisher', anonymous=True)
        
        # 设置服务器地址
        self.host = '127.0.0.1'  # 使用 localhost 进行测试
        self.port = 12345

        # 创建 Socket 客户端
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))

        # 发布模拟的 Pose 消息
        self.pose_publisher = rospy.Publisher('/pose', Pose, queue_size=10)
        self.rate = rospy.Rate(1)  # 设置发布频率为1Hz

    def generate_random_pose(self):
        # 生成随机的位姿数据
        pose = Pose()
        pose.position.x = random.uniform(-10, 10)
        pose.position.y = random.uniform(-10, 10)
        pose.position.z = random.uniform(-10, 10)
        pose.orientation.x = random.uniform(-1, 1)
        pose.orientation.y = random.uniform(-1, 1)
        pose.orientation.z = random.uniform(-1, 1)
        pose.orientation.w = random.uniform(-1, 1)
        return pose

    def send_pose_to_server(self, pose):
        try:
            # 将 Pose 消息转换为字典，然后转换为 JSON
            pose_dict = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w,
                }
            }
            pose_json = json.dumps(pose_dict)

            # 发送数据到服务器
            self.client_socket.send((pose_json + "\n").encode('utf-8'))
            rospy.loginfo(f"Sent simulated Pose data to server: {pose_json}")
        except socket.error as e:
            rospy.logerr(f"Socket error while sending data: {e}")

    def publish_simulated_data(self):
        while not rospy.is_shutdown():
            simulated_pose = self.generate_random_pose()

            # 发布模拟的 Pose 消息
            self.pose_publisher.publish(simulated_pose)
            rospy.loginfo(f"Published simulated Pose: {simulated_pose}")

            # 将模拟的 Pose 数据发送到服务器
            self.send_pose_to_server(simulated_pose)

            # 等待下一次发布
            self.rate.sleep()

if __name__ == '__main__':
    try:
        simulator = SimulatedDataPublisher()
        simulator.publish_simulated_data()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
