#include "../include/UavOnboard.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unistd.h>
#include <string>
#ifdef __linux__
    #include <socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
#else
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
#endif
//#include <mavlink.h>


Position::Position(float x, float y, float z, float roll, float pitch, float yaw)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
}

Position::Position()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->roll = 0;
    this->pitch = 0;
    this->yaw = 0;
}

int UavOnboard::decode_msg(char *msg) {
    std::cout << "Decoding message: " << msg << std::endl;
    return 0;

}

void UavOnboard::ListenThread() {
    WSADATA wsaData;
    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKADDR_IN service;

    // 初始化Winsock
    int iResult;
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        return;
    }

    // 创建套接字
    ListenSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ListenSocket == INVALID_SOCKET) {
        std::cerr << "socket failed with error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return;
    }

    // 准备 sockaddr_in 结构
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr(this->config.local_ip);
    service.sin_port = htons(this->config.local_udp_port);

    // 绑定套接字
    if (bind(ListenSocket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR) {
        std::cerr << "bind failed with error: " << WSAGetLastError() << std::endl;
        closesocket(ListenSocket);
        WSACleanup();
        return;
    }

    // 监听传入的数据
    char *recv_buffer = new char[1024];
    int addrlen = sizeof(service);
    while (true) {
        int recvLen = recvfrom(ListenSocket, recv_buffer, sizeof(recv_buffer), 0, (SOCKADDR*)&service, &addrlen);
        if (recvLen == SOCKET_ERROR) {
            std::cerr << "recvfrom failed with error: " << WSAGetLastError() << std::endl;
            break;
        }
        this->decode_msg(recv_buffer);
        std::cout << "Received message: " << recv_buffer << " from " << inet_ntoa(service.sin_addr) << std::endl;

    }

    // 清理
    closesocket(ListenSocket);
    WSACleanup();
}

void UavOnboard::send_msg(char * msg) {
    WSADATA wsaData;
    SOCKET SendSocket = INVALID_SOCKET;

    // 初始化Winsock
    WSAStartup(MAKEWORD(2,2), &wsaData);

    // 创建套接字
    SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (SendSocket == INVALID_SOCKET) {
        std::cerr << "socket failed with error: " << WSAGetLastError() << std::endl;
        return;
    }

    SOCKADDR_IN dest;
    int sendLen;
//    char message[] = "Hello, UDP Server";

    // 准备目标地址
    dest.sin_family = AF_INET;
    dest.sin_port = htons(this->config.gcs_udp_port);
    inet_pton(AF_INET, this->config.gcs_ip, &dest.sin_addr);

    sendLen = sendto(SendSocket, msg, sizeof(msg), 0, (SOCKADDR*)&dest, sizeof(dest));
    if (sendLen == SOCKET_ERROR) {
        std::cerr << "sendto failed with error: " << WSAGetLastError() << std::endl;
    }

    std::cout << "Sent message to " << this->config.gcs_ip << ":" << this->config.gcs_udp_port << std::endl;

//    Sleep(1000); // 等待1秒

    // 清理
    closesocket(SendSocket);
    WSACleanup();
}

UavOnboard::UavOnboard(config_t config)
{
    this->config = config;
    if(config.protocol == "udp")
    {
        // udp
        // code: bind to local_ip:local_udp_port and listen the port
        this->gcs_listen_thread = std::thread(&UavOnboard::ListenThread, this);

    }
    else
    {
        // serial
    }
}

UavOnboard::~UavOnboard()
{
}