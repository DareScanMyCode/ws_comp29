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

# 读取参数：--build_local, --rm
BUILD_FLAG=false
RM_FLAG=false
for arg in "$@"
do
    if [ "$arg" == "--build_local" ]; then
        BUILD_FLAG=true
        break
    fi
    if [ "$arg" == "--rm" ]; then
        RM_FLAG=true
        break
    fi
done

if [ "$RM_FLAG" = true ]; then
    echo -e ">>> Existing files will be ${RED}REMOVED${NC}"
else
    echo -e ">>> ${YELLOW}Skipping removal of existing files on target machines...${NC}"
fi

# 本机编译
if [ "$BUILD_FLAG" = true ]; then
    echo -e ">>> ${GREEN}Compiling LOCAL...${NC}"
    cd ~/ws_comp29
    colcon build
    source ~/ws_comp29/install/setup.bash
else
    echo -e ">>> ${YELLOW}Skipping LOCAL build...${NC}"
fi

# 检查目录是否存在
echo ">>> Trying to pack the directory ~/ws_comp29/src"
cd ~/
if [ -d "ws_comp29" ]; then
    # 打包文件夹
    tar --exclude='src/.git' -czf ws_comp29_src.tar.gz -C ~/ws_comp29/ src
else
    echo -e ">>> ${RED}Directory ~/ws_comp29 does not exist.${NC}"
    exit 1
fi

# 定义版本文件路径
version_file="/home/${USER}/ws_comp29/src/scripts/version.txt"
echo $version_file
# 检查文件是否存在
if [ -f "$version_file" ]; then

    # 读取版本号并只提取数字部分，忽略空行
    version=$(grep -oE '^[0-9]+' "$version_file")
    echo -e ">>> READ CODE VERSION : ${version}."
    
    # 将版本号加一
    new_version=$(($version + 1))

    # 写回新版本号
    echo $new_version > "$version_file"
    echo -e ">>> NEW ${GREEN} CODE VERSION : ${new_version}. ${NC}"

else
    echo -e ">>> ${RED} NO VERSION FILE FOUND at: ${version_file}. ${NC}"
    
    # 如果文件不存在，初始化版本号为0并写入文件
    echo "0" > "$version_file"
    version=0
fi


deploy(){
    target_ip=$1
    # 尝试ping
    ping -c 1 $target_ip &> /dev/null
    
    if [ $? -eq 0 ]; then
        echo ">>> Copying file to $target_ip and deploying..."
        
        # 将打包好的文件传输到目标机器
        sshpass -p 'cat' scp ws_comp29_src.tar.gz cat@$target_ip:~/
        
        # 通过SSH登录并删除原有文件夹，解压新文件
        if [ "$RM_FLAG" = true ]; then
        sshpass -p 'cat' ssh cat@$target_ip << EOF

rm -rf ~/ws_comp29/build
rm -rf ~/ws_comp29/install
rm -rf ~/ws_comp29/src
tar -xzf ~/ws_comp29_src.tar.gz -C ~/ws_comp29/
# rm ~/ws_comp29_src.tar.gz
# version_local=$(cat ~/ws_comp29/src/scripts/version.txt)
echo -e "<<< CODE VERSION on ${target_ip} is: \033[32m $(cat ~/ws_comp29/src/scripts/version.txt) \033[0m"
echo "<<< $target_ip: Done."
exit
EOF
        else
        sshpass -p 'cat' ssh cat@$target_ip << EOF
rm -rf ~/ws_comp29/src
tar -xzf ~/ws_comp29_src.tar.gz -C ~/ws_comp29/
# rm ~/ws_comp29_src.tar.gz
# version_local=$(cat ~/ws_comp29/src/scripts/version.txt)
echo -e "<<< CODE VERSION on ${target_ip} is: \033[34m $(cat ~/ws_comp29/src/scripts/version.txt) \033[0m"
echo "<<< $target_ip: Done."
exit
EOF
        echo -e ">>> ${GREEN}TARGET $target_ip COMPLETED.${NC}"
        fi
    else
        echo -e ">>> ${RED}TARGET $target_ip is not reachable.${NC}"
    fi
}

# 目标IP地址范围
for i in {203..204}; do
    IP="192.168.151.$i"
    # IP="192.168.151.202"
    echo -e ">>> Trying to deploy on ${YELLOW}$IP ${NC}"
    deploy $IP
    
done

echo ">>> Script execution completed."
