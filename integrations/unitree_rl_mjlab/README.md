# 接入官方 `unitree_rl_mjlab` 速度策略

官方 G1 部署代码中的 `velocity_commands` 默认从遥控器读取：

```text
[joystick ly, -joystick lx, -joystick rx] -> [vx, vy, wz]
```

视觉程序不应该假装成关节控制器。正确的改法是增加一个
`vision_velocity_commands` observation，让同一个速度跟踪 policy 改从本机 UDP 读取
`vx/vy/wz`。

## 1. 把接收代码加入 G1 部署程序

把 `vision_velocity_observation.cpp` 复制到：

```text
unitree_rl_mjlab/deploy/robots/g1/src/vision_velocity_observation.cpp
```

再用本目录的 `State_RLBase_vision.cpp` 替换部署目录中的
`src/State_RLBase.cpp`。里面的空函数调用用于确保静态链接器保留两个新增 observation
的注册代码。接收端会：

- 非阻塞监听 UDP 15001；
- 读取 ASCII `vx vy wz`；
- 按 `deploy.yaml` 中的训练范围限幅；
- 超过 0.30 秒没有新包时返回全零命令。

## 2. 修改策略 observation 名称

可以直接把本目录的 `deploy_vision.yaml` 复制到：

```text
deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml
```

它在官方文件的基础上把：

```yaml
velocity_commands:
```

改为：

```yaml
vision_velocity_commands:
```

其余 `params/clip/scale/history_length` 不变。

同一文件里的官方 `gait_phase` 会再次读取遥控器命令，因此还要把：

```yaml
gait_phase:
```

改成：

```yaml
vision_gait_phase:
```

其余 `period/clip/scale/history_length` 保持不变。漏掉这一步时，视觉已经发送
`vx`，但 gait phase 仍可能因为遥控器为零而不启动。

## 3. 重新编译

```bash
cd ~/unitree_ws/unitree_rl_mjlab/deploy/robots/g1
rm -rf build
mkdir build && cd build
export CPATH=~/unitree_ws/unitree_sdk2/include:~/unitree_ws/unitree_sdk2/thirdparty/include/ddscxx:~/unitree_ws/unitree_sdk2/thirdparty/include/iceoryx/v2.0.2:~/cyclonedds/install/include
export LIBRARY_PATH=~/unitree_ws/unitree_sdk2/lib/x86_64:~/unitree_ws/unitree_sdk2/thirdparty/lib/x86_64:~/cyclonedds/install/lib
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
```

## 4. 联调

先不要启动 G1。检查接收端是否能获得数据：

```bash
echo '0.20 0.00 0.05' | nc -u -w1 127.0.0.1 15001
```

随后按顺序启动：

1. `run_unitree_camera_sim.py`
2. 官方 G1 policy deploy 程序
3. 在仿真中进入 RL/Velocity 状态

第一次必须把机器人架空或只在仿真中测试。确认图像中赛道中心在右侧时，
发送的 `wz` 为负；若你的 policy 坐标约定相反，只改控制器输出的一个符号，
不要交换左右白线。
