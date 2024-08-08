#include <string>
#ifdef __linux__
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
#else   
    #include <winsock2.h>
#include <thread>

#endif
class Position
{
    public:
        Position();
        Position(float x, float y, float z, float roll, float pitch, float yaw);
//        ~Position();
        float x, y, z;
        float roll, pitch, yaw;

    private:

};

typedef struct config_t{
    std::string protocol = "udp";
    std::string gcs_port = "/dev/ttyUSB1";
    int gcs_baudrate = 115200;
    const char *local_ip = "127.0.0.1";
    const CHAR *gcs_ip = "127.0.0.1";
    int local_udp_port = 14550;
    int gcs_udp_port = 14550;
    int system_id = 1;
    std::string fcu_port = "/dev/ttyUSB0";
    int fcu_baudrate = 500000;
} config_t;

#define POS_SRC_NONE        -1
#define POS_SRC_VICON       0
#define POS_SRC_LOCAL       1
#define POS_SRC_GPS         2
#define POS_SRC_CALCULATED  3

//UAV_TASK_SEQ:
#define UAV_CMD_TASK_WAITE    -1
#define UAV_CMD_TASK_BEGIN    0
#define UAV_CMD_TASK_END      99

// UAV setting
#define UAV_MAX_SPEED           0.25  //  m/s;
#define UAV_MAX_SPEED_X         0.25  //  m/s;
#define UAV_MAX_SPEED_Y         0.25  //  m/s;
#define UAV_MAX_SPEED_Z         0.25  // m/s;
#define UAV_MAX_X               0
#define UAV_MIN_X               0
#define UAV_MAX_Y               0
#define UAV_MIN_Y               0
#define UAV_MAX_Z               2.0
#define UAV_SPEED_X_DEAD_ZONE   0.05  // X轴死区m/s;
class UavOnboard
{
public:
    UavOnboard(config_t config);
    UavOnboard(UavOnboard &&) = default;
    UavOnboard(const UavOnboard &) = default;
    UavOnboard &operator=(UavOnboard &&) = default;
    UavOnboard &operator=(const UavOnboard &) = default;
    ~UavOnboard();
    config_t config;
    Position pos;
    Position vel;
    void ListenThread();
    void set_pose(const Position& pos_){this->pos = pos_;};  // TODO
    void set_vel(const Position& vel_){this->vel = vel_;}; //
    int mission_state = UAV_CMD_TASK_WAITE;
    bool pos_got_ever = false;
    bool ctrl_block_flag = true;
    bool takeoff_confirm_without_position = false;
    int udp_out_sockfd, udp_in_sockfd;
    struct sockaddr_in gcs_addr, local_addr;
    void send_msg(char * msg);
    int decode_msg(char * msg);
private:
    std::thread gcs_listen_thread;
};

