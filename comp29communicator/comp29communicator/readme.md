### 总体概述
  随着探索的的进行，不断选取无人机作为基站，实现对其他无人机的定位

坐标系：
(0,0)右下角，(300,300)左上角

### 基站选取
  1. 初始化：
   随机选取两个，布置在(100,0)与(0,0)位置
  2. 找到目标之后：
    1. 如果编队之中存在对应编号，则发配一架到那里去，定在上面
    2. 如不存在对应编号，发配剩余编号最多的到那里去
  3. 任何时刻都需要两架距离在(100m)以上的两个基站
  
### 额外要求
  1. 定点控制需要加强
  2. 一直发送心跳信号，确保每个飞机是否alive
  3. 飞机保持在一定的高度范围
  4. 避碰有两种思路
     1. 在接近时(2m)分配不同的高度
     2. 飞机轨迹搜索有不同的优先级
        1. 高优先级的先搜索出一条轨迹
        2. 低优先级的再去搜索


### 具体实现
1. 属性
   1. 本机的状态
      1. 初始化
      2. 基站
      3. 悬空(无任务)
      4. 执行
   2. 其他无人机的状态
      1. 初始化
      2. 基站
      3. 悬空(无任务)
      4. 执行
   3. 总体的基本状态
      1. 通过heartbeat确定每个无人机是否在线
   4. tag_list
      1. 数字
      2. 位置(假设tag之间距离较大，可以相互区分)
   5. fsm_state
      1. 指示状态机要进行什么操作
2. Timer(定时触发)
   1. FSM(状态机)：主要部分
      在其他回调函数，udp的接收会导致fsm_state的改变
      fsm就根据其状态执行具体的操作
   2. 无人机运动控制
   3. 云台控制(大概率不在这里)
3. 消息的传递
   总体要求是在能接收消息的无人机都能接收到消息的条件下，最小化通信量
   由于目前使用的星型网络结构，只需要与“中心”判断就好。每隔一段时间向中心发送heartbeat，然后heartbeat回传哪些飞机是active的。
   星型网络不允许构建连通图
   鉴于上面的说明，什么消息都清楚能够发给谁，还没发给谁。
   因此最终的策略：**你发现了tag，就由你负责发送，你状态变了，就有由你发送给其他人**
   额外要求：**中心**飞机需要尽量在中心
4. 具体的消息类型及发送策略
   1. tag
   2. bind
    检测到tag的无人机除了发送tag信息外，还发送bind；然后同组的无人机给出bind，最后再回传最终确定的基站无人机
    或者直接检测到的做基站
   3. 规划的轨迹：用于避碰
   4. heartbeat
   5. 自身状态
      pos,vel,state
      其中state指示是否是基站
5. 接收到消息的处理
  1. tag(bind)
    1. 记录
    2. 如果是同组的，回传bind
  2. bind
    1. 检测到tag的进行处理，确定最终谁成为基站
    2. 发送确定消息给同组
  3. station_set
    说明该机被设置为station，则需要到达tag，然后根据已有的基站确定位置后，向其他无人机发送**自己成为基站，以及位置**
  4. trajectory
    1. 检测是否与自己的有冲突，有冲突进一步交流
  5.   