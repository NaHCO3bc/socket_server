import socket
import json
import rospy
import mysql.connector
from geometry_msgs.msg import Pose

def save_to_mysql(pose_data):
    try:
        # 连接MySQL数据库
        connection = mysql.connector.connect(
            host='localhost',       # 或者服务器的 IP 地址
            user='debian-sys-maint',     # 用户名
            password='UVDtNMmYAmF1N5MR',       
            database='pose'         # 数据库名称
        )
        cursor = connection.cursor()

        #  # 使用 REPLACE INTO 来插入或更新数据
        sql = """
        REPLACE INTO pose_data (id, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
        VALUES (1, %s, %s, %s, %s, %s, %s, %s)
        """
        val = (
            pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z'],
            pose_data['orientation']['x'], pose_data['orientation']['y'], pose_data['orientation']['z'], pose_data['orientation']['w']
        )
        
        # 使用 INSERT INTO 来插入新数据
        # sql = """
        # INSERT INTO pose_data (position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
        # VALUES (%s, %s, %s, %s, %s, %s, %s)
        # """
        # val = (
        #     pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z'],
        #     pose_data['orientation']['x'], pose_data['orientation']['y'], pose_data['orientation']['z'], pose_data['orientation']['w']
        # )
        cursor.execute(sql, val)
        connection.commit()

    except mysql.connector.Error as err:
        rospy.logerr(f"MySQL error: {err}")
    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()

def save_and_publish_data(data):
    try:
        # 处理接收到的数据
        pose_data = json.loads(data)
        save_to_mysql(pose_data)

        #print(f"data: {pose_data}")
        
        # 发布到 ROS 话题
        pub = rospy.Publisher('/pose', Pose, queue_size=10)
        rospy.init_node('pose_publisher_node', anonymous=True)
        rate = rospy.Rate(10)  # 10 Hz

        pose_msg = Pose()
        pose_msg.position.x = pose_data.get('position', {}).get('x', 0)
        pose_msg.position.y = pose_data.get('position', {}).get('y', 0)
        pose_msg.position.z = pose_data.get('position', {}).get('z', 0)
        pose_msg.orientation.x = pose_data.get('orientation', {}).get('x', 0)
        pose_msg.orientation.y = pose_data.get('orientation', {}).get('y', 0)
        pose_msg.orientation.z = pose_data.get('orientation', {}).get('z', 0)
        pose_msg.orientation.w = pose_data.get('orientation', {}).get('w', 1)

        #rospy.loginfo(f"Publishing pose: {pose_msg}")
        pub.publish(pose_msg)
        rate.sleep()
    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e}")

def main():
    # 创建一个 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 获取本地主机名
    host = '0.0.0.0'
    port = 12345

    # 绑定端口号
    server_socket.bind((host, port))

    # 设置最大连接数，超过后排队
    server_socket.listen(5)

    print("Server is listening...")

    buffer = ""  # 初始化缓冲区

    while True:
        # 建立客户端连接
        client_socket, addr = server_socket.accept()
        print(f"Got connection from {addr}")

        while True:
            try:
                # 接收数据
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                # 将接收到的数据添加到缓冲区
                buffer += data

                # 以换行符分割数据
                lines = buffer.split("\n")
                buffer = lines[-1]  # 保留最后一行可能不完整的部分

                # 处理完整的 JSON 数据
                for line in lines[:-1]:
                    if line.strip():  # 确保行不是空的
                        # 打印和处理接收到的数据
                        save_and_publish_data(line)

            except Exception as e:
                print(f"Error: {e}")
                break

        client_socket.close()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Unexpected error: {e}")
