import socket
import json
import rospy
import mysql.connector
from geometry_msgs.msg import Pose

# 连接 MySQL 数据库
def get_db_connection():
    connection = mysql.connector.connect(
        #使用sudo cat /etc/mysql/debian.cnf命令查看
        host='localhost',       
        user='debian-sys-maint',    
        password='SkrcGTjcUD720PCb',       
        database='pose'       
    )
    return connection

def save_odom_to_mysql(pose_data):
    try:
        # 连接MySQL数据库
        connection = get_db_connection()
        cursor = connection.cursor()

        #  # 使用 REPLACE INTO 来插入或更新数据
        sql = """
        REPLACE INTO odom_pose (id, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
        VALUES (1, %s, %s, %s, %s, %s, %s, %s)
        """
        val = (
            pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z'],
            pose_data['orientation']['x'], pose_data['orientation']['y'], pose_data['orientation']['z'], pose_data['orientation']['w']
        )
        cursor.execute(sql, val)
        connection.commit()

    except mysql.connector.Error as err:
        rospy.logerr(f"MySQL error: {err}")
    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()

def save_amcl_to_mysql(pose_data):
    try:
        # 连接MySQL数据库
        connection = get_db_connection()
        cursor = connection.cursor()

        #  # 使用 REPLACE INTO 来插入或更新数据
        sql = """
        REPLACE INTO amcl_pose (id, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
        VALUES (1, %s, %s, %s, %s, %s, %s, %s)
        """
        val = (
            pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z'],
            pose_data['orientation']['x'], pose_data['orientation']['y'], pose_data['orientation']['z'], pose_data['orientation']['w']
        )
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
        if pose_data.get("type")=="odom_pose":
            save_odom_to_mysql(pose_data)
        if pose_data.get("type")=="amcl_pose":
            save_amcl_to_mysql(pose_data)        
        
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

