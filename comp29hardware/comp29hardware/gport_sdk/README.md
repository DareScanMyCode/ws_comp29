# gport_sdk
## Requirements
- pyserial

## How to use
```python
gimbal = GportGimbal("path to your port") 
gimbal.set_angle_degree(0, 10, 10)  # can not control roll.
gimbal.set_speed_degree(0, 10, 10)  # can not control roll.

gimbal.should_stop = True

```
- It's essential to set `gimbal.should_stop = True` when program is end since a sub-thread is running to listen the UART and it should be stop.

## about gport
- 相机正置坐标系为FLU，倒置坐标系为FRD
- 相机倒置为例：
- 向前yaw为零，  向右yaw为正，  向左yaw为负
- 水平pitch为零，向上pitch为正，向下pitch为负
- 使用 hall_rpy 属性获取当前相机的角度
