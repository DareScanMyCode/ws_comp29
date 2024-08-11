#!/bin/bash
# 定义颜色
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
MAGENTA='\033[35m'
CYAN='\033[36m'
WHITE='\033[37m'
NC='\033[0m' # 没有颜色（重置）

# 检查参数
BUILD_FLAG=false
DEBUG_FLAG=false

for arg in "$@"
do
    if [ "$arg" == "--build" ]; then
        BUILD_FLAG=true
        break
    fi
    if [ "$arf" == "--debug" ]; then
        DEBUG_FLAG=true
        export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
        break
    fi
done


echo ">>> 检查环境变量"

# 检查UAV_ID和ROS_DOMAIN_ID环境变量
check_and_set_env() {
    local env_var_name=$1
    local env_var_value=$(eval echo \$$env_var_name)

    if [ -z "$env_var_value" ]; then
        echo -e ">>> ${RED}环境变量 $env_var_name 未设置，请在~/.bashrc中设置: ${NC}"
        echo -e "echo \"export $env_var_name=xxx\" >> ~/.bashrc"
    else
        echo -e "$env_var_name 当前值为 ${GREEN} $env_var_value ${NC}"
    fi
}

# 调用函数检查UAV_ID和ROS_DOMAIN_ID
check_and_set_env "UAV_ID"
check_and_set_env "ROS_DOMAIN_ID"

# 比较UAV_ID和ROS_DOMAIN_ID
if [ "$UAV_ID" != "$ROS_DOMAIN_ID" ]; then
    echo -e ">>> ${RED}UAV_ID ($UAV_ID) 和 ROS_DOMAIN_ID ($ROS_DOMAIN_ID) 不同，请确保它们一致。${NC}"
fi

# 设置ros2的logger格式
echo ">>> 设置ROS 2 日志格式"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}]: {message}"

# 重新加载 .bashrc 以应用更改
# source ~/.bashrc

echo ">>> ROS 2 日志格式已设置为仅包括节点名称、时间和消息类型。"

echo ">>> 构建与source"
if [ "$BUILD_FLAG" = true ]; then
    echo -e ">>> ${YELLOW}Compiling...${NC}"
    cd ~/ws_comp29
    colcon build
fi

export DISPLAY=:0

echo ">>>启动主节点"
source ~/ws_comp29/install/setup.bash
ros2 launch comp29planner comp29test_fsm.py
