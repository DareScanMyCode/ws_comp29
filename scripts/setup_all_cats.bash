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

# 本机编译
echo -e ">>> ${GREEN}Compiling...${NC}"
cd ~/ws_comp29
colcon build

# 检查目录是否存在
echo ">>> Trying to pack the directory ~/ws_comp29/src"
cd ~/
if [ -d "ws_comp29" ]; then
    # 打包文件夹
    tar -czf ws_comp29_src.tar.gz -C ~/ws_comp29/ src
else
    echo -e ">>> ${RED}Directory ~/ws_comp29 does not exist.${NC}"
    exit 1
fi

deploy(){
    target_ip=$1
    echo ">>> Trying to ping $target_ip"
    # 尝试ping
    ping -c 1 $target_ip &> /dev/null
    
    if [ $? -eq 0 ]; then
        echo ">>> $target_ip is reachable, proceeding with SCP"
        
        # 将打包好的文件传输到目标机器
        sshpass -p 'cat' scp ws_comp29_src.tar.gz cat@$target_ip:~/
        
        echo ">>> $target_ip is reachable, proceeding with SSH"
        # 通过SSH登录并删除原有文件夹，解压新文件
        sshpass -p 'cat' ssh cat@$target_ip << EOF
rm -rf ~/ws_comp29/build
rm -rf ~/ws_comp29/install
rm -rf ~/ws_comp29/src
tar -xzf ~/ws_comp29_src.tar.gz -C ~/ws_comp29/
# rm ~/ws_comp29_src.tar.gz
echo ">>> $target_ip: Done."
exit
EOF
        echo -e ">>> ${GREEN} TARGET $target_ip COMPLETED.${NC}"
    else
        echo -e ">>> ${RED} TARGET $target_ip is not reachable.${NC}"
    fi
}

# 目标IP地址范围
for i in {202..202}; do
    IP="192.168.151.$i"
    IP="192.168.151.202"
    echo -e ">>> Trying to deploy on ${GREEN} $IP ${NC}"
    deploy $IP
    
done

echo "Script execution completed."
