# Onboard Config file
################################################################################
# Define the Connection configuration
################################################################################

UAV_cfg = {
    "UAV_ID": 0,
    "CONNECTION": 
    {
        "FCU_CONNECTION": {
            "FCU_PROTOCOL": "mavlink_uart",
            "FCU_PORTS": "/dev/ttyS0",
            "FCU_BAUD":  921600
        },
        "GCS_CONNECTION": {
            "GCS_PROTOCOL": "udp",
            "LOCAL_IP": "192.168.151.202",
            "GCS_IP":   "192.168.151.101",
            "MSG_IN_PORT_GCS": 52001,
            "MSG_IN_PORT_LOCAL": 52000,
            "GCS_UART_PORT": "/dev/ttyUSB1",
            "GCS_UART_BAUD": 57600
        }
    },
    # "UAV_TASK_SEQ":
    # {
    #     "description": "UAV task sequence",
    #     "UAV_CMD_TASK_WAITE"  : -1,
    #     "UAV_CMD_TASK_BEGIN"  : 0,
    #     "UAV_CMD_TASK_END"    : 99
    # },
    "movement":
    {
        "UAV_MAX_SPEED"   : 0.25,
        "UAV_MAX_SPEED_XYZ" : [0.25, 0.25, 0.25],
        "UAV_MAX_XYZ" : [None, None, None],
        "UAV_MIN_XYZ" : [None, None, None]
    }
}

UAV_cfg = {
    "UAV_ID": 0,
    "CONNECTION": 
    {
        "FCU_CONNECTION": {
            "FCU_PROTOCOL": "mavlink_udp",
            "FCU_PORTS": "/dev/ttyUSB0",
            "FCU_BAUD":  500000
        },
        "GCS_CONNECTION": {
            "GCS_PROTOCOL": "udp",
            "LOCAL_IP": "127.0.0.1",
            "GCS_IP":   "127.0.0.1",
            "MSG_IN_PORT_GCS": 52001,
            "MSG_IN_PORT_LOCAL": 52000,
            "GCS_UART_PORT": "/dev/ttyUSB1",
            "GCS_UART_BAUD": 57600,
            "VIDEO_ON": False,
            "VIDEO_PROTOCOL": "udp",
            "VIDEO_IP": None,
            "VIDEO_PORT": 52002
        }
    },
    # "UAV_TASK_SEQ":
    # {
    #     "description": "UAV task sequence",
    #     "UAV_CMD_TASK_WAITE"  : -1,
    #     "UAV_CMD_TASK_BEGIN"  : 0,
    #     "UAV_CMD_TASK_END"    : 99
    # },
    "movement":
    {
        "UAV_MAX_SPEED"   : 0.25,
        "UAV_MAX_SPEED_XYZ" : [0.25, 0.25, 0.25],
        "UAV_MAX_XYZ" : [None, None, None],
        "UAV_MIN_XYZ" : [None, None, None]
    }
}

################################################################################
# Define the UAV's configuration
################################################################################
# import socket
# UAV_TASK_SEQ:
UAV_CMD_TASK_WAIT   = 0
UAV_CMD_TASK_BEGIN  = 1

UAV_CMD_TASK_END    = 99

import time
class DetectResult:
    def __init__(self) -> None:
        self.detect_num = 0
        # left top right bottom
        self.target_pos = [0,0,0,0]
        self.est_pos = [0,0]
        self.detect_time = 0
        pass

    def get_dict(self):
        return {
            "detect_num": self.detect_num,
            "target_pos": self.target_pos,
            "detect_time": self.detect_time,
            "est_pos": self.est_pos
        }
        
    def set_result(self, detect_num, target_pos, est_pos=None):
        self.detect_num = detect_num
        self.target_pos = target_pos
        self.est_pos = est_pos if est_pos is not None else [0,0]
        self.detect_time = time.time()
        
        
class DrvCmd:
    CMD_VERSION                 = "v0.0.1"
    # mavlink mission now with 7 params
    # 0: task index 1-6: params
    # tasks
    DRV_CMD_TASK_WAIT           = 0
    DRV_CMD_TASK_BEGIN          = 1
    DRV_CMD_TASK_ILLEGAL_AREA   = 2
    DRV_CMD_TASK_GOTO_WAIT      = 10  # x, y, yaw
    DRV_CMD_TASK_FOLLOW_PERSON  = 12
    DRV_CMD_TASK_TEST_GOTO      = 80  # x, y, yaw
    DRV_CMD_TASK_END            = 99
    DRV_CMD_EMERGENCY_STOP      = 100
    # about target
    DRV_INFO_TARGET_POS_GLOBAL = 101  # x, y
    DRV_INFO_TARGET_POS_EST    = 102  # x, y
    DRV_INFO_TARGET_POS_IMG    = 103  # left, top, right, bottom
    # about agent
    DRV_INFO_AGENT_POS_GLOBAL  = 110  # x, y, id_
    DRV_INFO_AGENT_TASK_NOW    = 111  # task_now, id_
    
    
def get_local_ip():
    import socket
    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)
    return ip

# get_local_ip()
# # UAV mode
# WAITING      = 0
# ARM         = 1
# TAKEOFF     = 2
# LAND        = 3
# TRACK       = 4
# HOVER       = 5
# DISARM      = 6

# RIGHT_GOING = 10
# LEFT_GOING  = 11
# TURN_LEFT   = 20
# TURN_RIGHT  = 21
# TURN_BACK   = 22
# TURN_AHEAD  = 23

# MODE_DIC = {WAITING      :"WATING", 
#             ARM         :"ARM",
#             TAKEOFF     :"TAKEOFF",
#             LAND        :"LAND   ",
#             TRACK       :"TRACK  ",
#             HOVER       :"HOVER  ",
#             DISARM      :"DISARM",
#             RIGHT_GOING :"RIGHT_GOING",
#             LEFT_GOING  :"LEFT_GOING",
#             }

# def udp_server():
#     # Localhost IP
#     host = '127.0.0.1'
#     # Port to listen on
#     port = 5001

#     # Create a UDP socket
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     # Bind the socket to the address and port
#     server_socket.bind((host, port))

#     print(f"UDP server up and listening on {port}")

#     # Listen for incoming datagrams
#     while True:
#         data, address = server_socket.recvfrom(1024)
#         print(f"Message from {address}: {data.decode()}")

#         # Optionally, send a response back to the client
#         response = "Message received"
#         server_socket.sendto(response.encode(), address)
        
# def udp_client_send(message, target_port):
#     # Localhost IP
#     host = '127.0.0.1'

#     # Create a UDP socket
#     client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#     try:
#         # Send the message to the server
#         client_socket.sendto(message.encode(), (host, target_port))

#         # Receive response from the server
#         data, server = client_socket.recvfrom(1024)
#         print(f"Response from server: {data.decode()}")
#     finally:
#         # Close the socket
#         client_socket.close()

# if __name__ == "__main__":
#     import threading
#     t = threading.Thread(target=udp_server)
#     t.start()
#     # Example message
#     message = "Hello, UDP Server!"
#     # Target port (the server's listening port)
#     target_port = 5001

#     # Call the client function
#     udp_client_send(message, target_port)