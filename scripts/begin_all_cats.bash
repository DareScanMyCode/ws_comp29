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

begin_task(){
    target_ip=$1
    echo ">>> Trying to ping $target_ip"
    # 尝试ping
    ping -c 1 $target_ip &> /dev/null
    
    if [ $? -eq 0 ]; then
        echo ">>> $target_ip is reachable, proceeding with SCP and SSH"
        
        # 将打包好的文件传输到目标机器
        sshpass -p 'cat' scp ws_comp29.tar.gz cat@$target_ip:~/
        
        # 通过SSH登录并删除原有文件夹，解压新文件
        sshpass -p 'cat' ssh cat@$target_ip << EOF
        source ~/.bashrc
        bash ./ws_comp29/src/scripts/comp29run.sh
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
    echo ">>> Trying to deploy on ${GREEN} $target_ip ${NC}"
    begin_task $IP
    
done

echo "Script execution completed."
