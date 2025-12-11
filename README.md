# Project Introduction

This project is part of the Electronic Information Engineering course design at Guangdong University of Technology. The following section introduces the general content and function of the files, as well as the program startup sequence.

## Document Introduction

### 1. socket_service.py
This is a program that runs on a cloud server. It includes socket communication with the vehicle or computer, and is responsible for receiving information and storing it in a MySQL database. It also has an interface for sending information from the MySQL database to the vehicle.

### 2. socket_local.py
This is a local program that runs on a vehicle or computer, using a socket interface to communicate with a cloud server, and has the functions of pushing topic information and receiving information.

### 3. tcp_server.py
This is a local program that runs on a vehicle or computer, using a socket interface to communicate with a cloud server, and has the functions of pushing topic information and receiving information.

### 4. web_code.txt
This file contains information about the cloud server and will be updated periodically as the project is used.

## Program startup order
1. Start `socket_service.py` to run on the cloud server, handling communication with the vehicle or computer and storing information in a MySQL database.
2. Start `socket_local.py` to run on the vehicle or computer, handling communication with the cloud server, sending topic information and receiving data from the cloud.
3. Start `tcp_server.py` to handle the front-end display, pushing location data from MySQL to the front-end, and processing coordinate information input from the front-end.

**Licensed under the BSD 3-Clause License. See the LICENSE file for details.**
