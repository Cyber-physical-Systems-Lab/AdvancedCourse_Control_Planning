# Nova5 + Robotiq 2F-85 仿真（Gazebo Classic）

模型来自实验室仓库 `All_Robot/robots/nova5_2f85/urdf/nova5_2f85.urdf`（Xuezhi 组装），
控制部分沿用 Dobot 的 ros2_control 配置，所以手臂的接口和以前完全一样。

## 文件

```
description/nova5_2f85_sim.urdf   模型：实验室的手臂+夹具 + world 固定 + 关重力 + ros2_control + 插件
description/controllers.yaml      控制器：nova5_group_controller（手臂 JTC）、joint_state_broadcaster、gripper_controller（夹具）
description/meshes/               网格文件（nova5 7 个、robotiq_2f85 10 个）
description/shelf.sdf             架子 + 3 个箱子（拉出来会耷拉、卡住，不会掉）
description/tools/make_shelf.py   生成 shelf.sdf 的脚本（尺寸参数都在文件顶部）
launch/sim.launch.py              启动 Gazebo、生成机器人和架子、按顺序激活 3 个控制器
scripts/start_sim.sh              启动仿真
scripts/go_home.sh                手臂回 home / 零位
scripts/gripper.sh                夹具 open / close / 指定角度
```

## 用法（在 pulldrawer_dev 容器里，旧的 dobot_dev 也能用）

```bash
cd /dobot_ws/shared/ros_ws/PullDrawerProject/scripts
./start_sim.sh                 # 终端 1：启动仿真
./go_home.sh                   # 终端 2：手臂回 home
./gripper.sh close             # 夹具合上
./gripper.sh open              # 夹具张开
./gripper.sh 0.4               # 开合到 0.4 rad（0.0 全开 ~ 0.725 全合）
```

## 接口

| 用途 | 名字 | 类型 |
|---|---|---|
| 手臂轨迹 | `/nova5_group_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory`（和以前一样） |
| 夹具 | `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand`，`command.position` = finger_joint 角度 |
| 关节状态 | `/joint_states` | 多了 finger_joint 和 5 个联动关节，joint1~6 按名字取不受影响 |

## 你的代码要改的一处

FK/IK 必须用同一个模型：实验室模型的 joint1、joint4 转轴是 +z（Dobot 的 Gazebo 模型是 −z）。

```bash
cd /dobot_ws/shared/ros_ws/PullDrawerProject
mv assets/nova5_gazebo.urdf assets/nova5_gazebo_dobot_old.urdf
cp description/nova5_2f85_sim.urdf assets/nova5_gazebo.urdf
```

之后 IK 算出的 joint1、joint4 会和以前差一个负号（正常）。例：T_pre 从 home 出发
→ `[-0.1271, -0.2572, -1.7468, 0.4332, 1.5708, -1.6979]`。

## 夹具几何（以后算目标位姿要用）

- 手指沿 Link6 的 **x 轴** 开合
- 指尖离法兰（Link6 原点）沿 Link6 z 轴约 **159 mm（张开）/ 170 mm（合上）**
- 张开时两指垫间距约 78 mm（按网格估算）

## 架子和箱子（description/shelf.sdf，启动时自动加载）

不要架子：`ros2 launch .../launch/sim.launch.py shelf:=false`

- 架子在机器人 +x 方向：前沿 x = 0.70，深 0.45，宽 0.50，中心 y = −0.20（world）
- 3 层，层板上表面 z = 0.10 / 0.30 / 0.50，顶板 0.70；每层一个开口朝上的蓝色箱子：
  外形 0.40（x）× 0.34（y）× 0.15（高），壁厚 1 cm，0.8 kg
- **箱子上沿到上一层层板底面只留 3 cm**
- **只有最下层的箱子（bin_0）伸出架子 10 cm**，是要抓的目标；上面两层和架子前沿齐平
  （上层箱子要是也伸出来，就正好挡在从上往下抓时手腕和夹具的位置）

### 箱子怎么动（真实物理，没有隐藏导轨）

箱子就放在层板上，靠接触和摩擦（μ = 0.5）支撑。三个关节只把它限制在 x–z 平面里
（能前后滑、上下动、前后倾），不会左右滑或打转：

1. 往外拉时正常滑动，拉力约 μ·m·g ≈ 4 N
2. 拉出约 **0.10 m** 后，箱子重心越过层板前沿，**前端开始往下耷拉**
3. 后沿翘起来，顶到上一层层板的底面（只有 3 cm 空隙），箱子就**卡住**了，不会掉
4. 只要拉出量不超过约 **0.20 m**，松开夹具、拿走机械臂，箱子都会保持卡住的状态

关节的范围（拉出 ≤ 0.32 m、倾斜 ≤ 34°）只是保险，正常情况下碰不到。

`ros2 topic echo /shelf/joint_states`：`drawer_0` = bin_0 往外移动了多少（m），
`bin_0_tilt` = 倾斜角（rad，前端朝下是负值）

### 目标箱子 bin_0 和 Link6 目标位姿

bin_0 前壁（world）：外侧 x = 0.60，内侧 x = 0.61，y = −0.20，上沿 z = 0.25（= base_link 0.22）

夹具**张开**、从上往下插、里面那根手指贴在前壁内侧后面约 3.5 mm，指尖到上沿下 2.5 cm：

| 位姿 | Link6 位置（base_link），姿态都是朝下 R = diag(1, −1, −1) |
|---|---|
| 预抓取 | (0.574, −0.20, 0.454) |
| 插进去（下降 10 cm） | (0.574, −0.20, 0.354) |
| 拉出 0.16 m | (0.414, −0.20, 0.354) |
| 抬起离开 | (0.414, −0.20, 0.454) |

我用你的 IK 和直线函数从 home 把这一套都算过：全部收敛，σ_min ≥ 0.18，
手臂和夹具的大致外形不碰架子、上层箱子和层板。

预期：拉完 `drawer_0` ≈ 0.156（手指先空走约 3.5 mm）；抬起离开后，箱子前端耷拉约 14°
（前沿下降约 6 cm，`bin_0_tilt` ≈ −0.25），卡在那里不掉。

改尺寸：编辑 `description/tools/make_shelf.py` 顶部的参数，运行 `python3 make_shelf.py` 重新生成 shelf.sdf。
它会打印新的箱子位置，以及"拉多少开始耷拉、拉多少以内能卡住"。

## 如果夹具控制器没起来

启动日志里 `gripper_controller` 报找不到类型时，说明镜像里没装 gripper_controllers。
新镜像 pulldrawer:humble 已经装好了，只有旧的 dobot_dev 可能缺。在主机上用 root 进容器装一次：

```bash
docker exec -u root -it dobot_dev bash
apt update && apt install -y ros-humble-gripper-controllers
```

手臂不受影响，照常可用。
