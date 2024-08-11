#!/bin/bash
echo ">>>检查环境变量"

# 检查UAV_ID和ROS_DOMAIN_ID环境变量
check_and_set_env() {
    local env_var_name=$1
    local env_var_value=$(eval echo \$$env_var_name)

    if [ -z "$env_var_value" ]; then
        read -p "环境变量 $env_var_name 未设置，请输入值: " env_var_value
        echo "export $env_var_name=$env_var_value" >> ~/.bashrc
        export $env_var_name=$env_var_value
        echo "$env_var_name 设置为 $env_var_value 并已写入 .bashrc"
    else
        echo "$env_var_name 当前值为 $env_var_value"
    fi
}

# 调用函数检查UAV_ID和ROS_DOMAIN_ID
check_and_set_env "UAV_ID"
check_and_set_env "ROS_DOMAIN_ID"

# 比较UAV_ID和ROS_DOMAIN_ID
if [ "$UAV_ID" != "$ROS_DOMAIN_ID" ]; then
    echo "UAV_ID ($UAV_ID) 和 ROS_DOMAIN_ID ($ROS_DOMAIN_ID) 不同，请确保它们一致。"
fi

# 设置ros2的logger格式
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}]: {message}"

# 重新加载 .bashrc 以应用更改
# source ~/.bashrc

echo "ROS 2 日志格式已设置为仅包括节点名称、时间和消息类型。"

echo ">>>构建与source"
cd ~/ws_comp29
colcon build
source ~/ws_comp29/install/setup.bash

export DISPLAY=:0
# ros2 run comp29planner test_formation
ros2 launch comp29planner comp29test_form.py