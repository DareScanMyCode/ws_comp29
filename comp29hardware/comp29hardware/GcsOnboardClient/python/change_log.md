# UavOnboard python version
## 2024.4.11
- **ADD** 
- UDP mavlink transition between uav computer and gcs
    - GCS -> UAV
        - `MAV_CMD_DO_SET_PARAMETER`
## Until 2024.4.10
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