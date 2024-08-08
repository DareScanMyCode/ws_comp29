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

# 进入工作目录
cd ~/

# 检查目录是否存在
echo ">>> Trying to pack the directory ~/ws_comp29"
if [ -d "ws_comp29" ]; then
    # 打包文件夹
    tar -czf ws_comp29.tar.gz -C ~/ ws_comp29
else
    echo -e ">>> ${RED}Directory ~/ws_comp29 does not exist.${NC}"
    exit 1
fi

# 目标IP地址范围
for i in {201..208}; do
    IP="192.168.151.$i"
    
    echo ">>> Trying to ping $IP"
    # 尝试ping
    ping -c 1 $IP &> /dev/null
    
    if [ $? -eq 0 ]; then
        echo "$IP is reachable, proceeding with SCP and SSH"
        
        # 将打包好的文件传输到目标机器
        sshpass -p 'cat' scp ws_comp29.tar.gz cat@$IP:~/
        
        # 通过SSH登录并删除原有文件夹，解压新文件
        sshpass -p 'cat' ssh cat@$IP << EOF
        rm -rf ~/ws_comp29
        tar -xzf ~/ws_comp29.tar.gz -C ~/
        # rm ~/ws_comp29.tar.gz
        echo "$IP: Done."
        exit
EOF
        echo -e "${GREEN}Operations on $IP completed.${NC}"
    else
        echo -e "${RED}$IP is not reachable.${NC}"
    fi
done

echo "Script execution completed."
