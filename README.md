# 29所比赛核心仓库
- ROS2仓库
- 文件说明：
    - communicator：负责飞机之间的通讯
    - detector：负责目标检测
    - hardware：负责与硬件进行通讯
        - gport2ros2：云台<->ROS2
        - uav2ros2：无人机<->ROS2
        - uwb2ros2：uwb->ROS2
    - localization：负责定位
    - msg：各种自定义消息
    - planner：核心规划器
        - 基站
        - 编队
        - 测试等
    - scripts：一些bash文件
        - comp29run.sh：目前只启动硬件节点，先编译后执行
        - setup_all_cats.bash：在工作站（PC）（Linux）上将现有代码`强行`同步到所有的无人机
    - configs：无人机配置文件
        - UAV_configs：151网段配置文件
        - UAV_configs：136（自组网）网段配置文件
    - weights：存放权重文件
# Important
- 执行setup命令之后会覆盖原有文件！
- 必须注意相机的安装角度！