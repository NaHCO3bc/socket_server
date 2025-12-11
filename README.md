# 项目介绍

该项目为广工电子信息工程课程设计内容，下面介绍文件的大概内容和作用，同时介绍程序启动的顺序。

## 文件介绍

### 1. `socket_service.py`
`socket_service.py`为云服务器端运行的程序，它包含了与小车或电脑的socket通信，负责接受信息并存入MySQL数据库。同时，它也有将MySQL数据库中的信息发送到小车端的接口。

### 2. `socket_local.py`
`socket_local.py`为小车或电脑端本地的运行程序，使用socket接口与云服务器通信，具备话题信息的推送功能和信息接收功能。

### 3. `tcp_server.py`
`tcp_server.py`为云服务器上的前后端程序，负责将MySQL中的位置信息推送到前端，进行轨迹图或数据的展示。同时，它也有前端输入坐标的功能。其中index.html为前端文件

### 4. `web_code.txt`
这个文件中是云服务器的相关信息，会随着该项目的使用，阶段性更新。

## 程序启动顺序
1. 启动 `socket_service.py` 在云服务器上运行，处理与小车或电脑的通信，并存储信息到MySQL数据库。
2. 启动 `socket_local.py` 在小车或电脑端运行，处理与云服务器的通信，发送话题信息和接收云端的数据。
3. 启动 `tcp_server.py` 负责前端展示，推送MySQL中的位置数据到前端，并处理前端输入的坐标信息。

**Licensed under the BSD 3-Clause License. See the LICENSE file for details.**
