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


# 检查UAV_ID和ROS_DOMAIN_ID环境变量
check_and_set_env() {
    local env_var_name=$1
    local env_var_value=$(eval echo \$$env_var_name)

    if [ -z "$env_var_value" ]; then
        read -p ">>> ${YELLOW}环境变量 $env_var_name 未设置，请输入值: ${NC}" env_var_value
        echo -e "export $env_var_name=$env_var_value" >> ~/.bashrc
        export $env_var_name=$env_var_value
        echo -e ">>> ${YELLOW}$env_var_name 设置为 $env_var_value 并已写入 .bashrc"
    else
        echo -e "$env_var_name 当前值为 ${GREEN} $env_var_value ${NC}"
    fi
}

echo ">>> 检查环境变量"
check_and_set_env "UAV_ID"
check_and_set_env "ROS_DOMAIN_ID"

# 关闭ssh的信息提示
echo -e ">>> 检查ssh的信息提示"
read -p ">>> ${YELLOW}是否关闭 打开ssh时的信息提示${NC}[y/n]: " ssh_info_echo
if [ "$ssh_info_echo" == "y" ]; then
    echo -e ">>> ${YELLOW}关闭ssh的信息提示${NC}"
    sudo chmod -x /etc/update-motd.d/*
else
    echo -e ">>> ${YELLOW}保留ssh的信息提示${NC}"
fi

# 检查colcon是否安装
echo ">>> 检查colcon是否安装"
if ! command -v colcon &> /dev/null
then
    echo -e ">>> ${RED}colcon 未安装${NC}"
    read -p ">>> ${YELLOW}是否安装colcon${NC}[y/n]: " install_colcon
    if [ "$install_colcon" == "y" ]; then
        echo -e ">>> ${GREEN}将要安装colcon${NC}"
        sudo apt install python3-colcon-common-extensions
    else
        echo -e ">>> ${YELLOW}未安装colcon${NC}"
    fi
else
    echo -e ">>> ${GREEN}colcon 已安装${NC}"
fi

# 永久添加串口权限
echo ">>> 永久添加串口权限"
read -p ">>> ${YELLOW}是否永久添加串口权限${NC}[y/n]: " add_serial_permission
if [ "$add_serial_permission" == "y" ]; then
    echo -e ">>> ${GREEN}将永久添加串口权限${NC}"
    sudo touch /etc/udev/rules.d/70-ttyusb.rules
    echo 'KERNEL=="ttyS*",MODE="0666"' > /etc/udev/rules.d/70-ttyusb.rules
    echo 'KERNEL=="ttyUSB*",MODE="0666"' >> /etc/udev/rules.d/70-ttyusb.rules
    echo -e ">>> ${GREEN}串口权限重启生效${NC}"

else
    echo ">>> ${YELLOW}未添加串口权限${NC}"
fi

# 设置shutdown命令
echo ">>> 设置shutdown命令为 sudo shutdown -h now"
echo 'alias shutdown="sudo shutdown -h now"' >> ~/.bashrc
