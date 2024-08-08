# Onboard python code
## Important
- [240806] 修改了SETGROUP的CMDID，尚未与GCS同步！！
## How to use
1. modify settings in `config.py` or `config.json`

2. import `UavOnBoard`
```python
import config as cfg
from UavOnBoard import UavOnBoard
Uav = UavOnBoard(config_file="config.py")
```
or
```python
import config as cfg
from UavOnBoard import UavOnBoard
Uav = UavOnBoard(config_file=config_dict)  # config_dict should be in the same format with `UAV_cfg` in `config.py`
```
3. use `Uav` in your code
```python
if Uav.mission_state == cfg.UAV_CMD_TASK_BEGIN: 
    # begin task
    pass
elif Uav.mission_state == cfg.UAV_CMD_TASK_END: 
    # end task
    pass
elif Uav.mission_state == cfg.YOUR_TASK_NUM:
    pass
```

4. set `YOUR_TASK_NUM`:
    - define `MISSION_CURRENT` code like `UAV_CMD_TASK_END` in `config` file(both gcs and onboard), code between 0 and 99(0 for begin, 99 for end)
    ```python
        UAV_CMD_MISSION_CURRENT_1 = 1
        UAV_CMD_MISSION_CURRENT_2 = 2
    ```
    - send `MAV_CMD_DO_SET_MISSION_CURRENT` with `param1` as the `MISSION_CURRENT` number

## About
- UDP mavlink transition between uav computer and gcs
    - GCS -> UAV
        - `COMMAND_LONG` with param list [...]
            - `MAV_CMD_DO_SET_MISSION_CURRENT` [mission_state]
            - `MAV_CMD_NAV_TAKEOFF` [takeoff_height]
            - `MAV_CMD_NAV_LAND`
            - `MAV_CMD_COMPONENT_ARM_DISARM` [arm_flag]: 1 for arm, 0 for disarm
            - `MAV_CMD_DO_PAUSE_CONTINUE` [continue_flag]: 1 for continue, 0 for pause
        - `VICON_POSITION_ESTIMATE`
    - UAV -> GCS (FCU -> UAV -> GCS)
        - `LOCAL_POSITION_NED`
        - `ATTITUDE`
- UART mavlink transition between uav computer and fcu
    - FCU -> UAV
        - `HEARTBEAT`
        - `LOCAL_POSITION_NED`
        - `ATTITUDE`
    - UAV -> FCU
        - `SET_LOCAL_POSITION_NED`
            - Send FLU spd
        - `COMMAND_LONG`
            - `MAV_CMD_COMPONENT_ARM_DISARM`