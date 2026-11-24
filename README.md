# t800_ros_api

ROS 2 Humble + FastAPI REST 网关。**只读 Topic，唯一 POST 只调用重定位 Service**；不启动或修改任何机器人速度控制器。

## 1. 数据来源（已核对源码）

- `/my_ros2/controller/status`：`std_msgs/msg/String`。根据用户提供的 `body_vel_twist_to_vendor(6).py`，默认以 10 Hz 发布 `|` 分隔的诊断字符串；API 同时返回原字符串和解析后的 `parsed`。
- `/my_ros1/localization/status`：`std_msgs/msg/String`。
- `/my_ros1/localization/pose_status`：`std_msgs/msg/String`。
- `/my_ros1/localization/base_footprint_pose`：`geometry_msgs/msg/PoseStamped`（包含 `header.frame_id`、时间戳、三维位置与四元数）。
- `/my_ros1/localization/relocalize`：`std_srvs/srv/Trigger`，Request `{}`，Response `{success: bool, message: string}`；它是 ROS1 中的 `std_srvs/Trigger`，必须能经 `ros1_bridge` 从 ROS2 调用。

以上定位相关接口来自 `hanxiao5658/t800_localization` 仓库 `main` 分支。

## 2. 在现有 ROS2 Humble Docker3 中安装

将整个 `t800_ros_api/` 文件夹复制到 ROS2 工作区 `~/ros2_ws/src/` 下：

```bash
# 以下命令在已进入 Docker3 的终端执行
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 验证这些值与你此前已调通的 ROS2 环境一致
echo "$ROS_DOMAIN_ID"              # 预期 69
echo "$RMW_IMPLEMENTATION"        # rmw_cyclonedds_cpp
echo "$CYCLONEDDS_URI"            # 厂家/项目已有配置，不能擅自覆盖

# FastAPI 依赖只需安装一次，不需要升级 ROS 或 Jetson 系统
python3 -m pip install -r ~/ros2_ws/src/t800_ros_api/requirements.txt

cd ~/ros2_ws
colcon build --symlink-install --packages-select t800_ros_api
source install/setup.bash

# 检查接口（只读，不会触发重定位）
ros2 topic info /my_ros2/controller/status -v
ros2 topic info /my_ros1/localization/base_footprint_pose -v
ros2 service list -t | grep '/my_ros1/localization/relocalize'
ros2 service type /my_ros1/localization/relocalize

# 手动启动 REST 网关：前台运行，Ctrl+C 停止
ros2 run t800_ros_api api_server
```

如容器内 `pip` 不存在，先安装 `python3-pip`；请避免覆盖系统 ROS Python 环境。如果使用自建 Python venv，必须能访问 `/opt/ros/humble` 的 Python 包（包括 `rclpy`），且与其 Python 版本一致。

默认绑定 **`127.0.0.1:8000`**，适合同容器后端使用。若后端需要从另一个容器/主机访问，可在启动前显式设置：

```bash
export T800_API_HOST=0.0.0.0
export T800_API_PORT=8000
ros2 run t800_ros_api api_server
```

需同时确保 Docker 容器网络和端口可达；如果容器使用 `--network host`，通常可以直接通过 Jetson 的对应 IP 访问。

## 3. REST 路由

| 方法 | 地址 | ROS 对应项 |
| --- | --- | --- |
| GET | `/api/v1/controller/status` | `/my_ros2/controller/status` |
| GET | `/api/v1/localization/status` | `/my_ros1/localization/status` |
| GET | `/api/v1/localization/pose_status` | `/my_ros1/localization/pose_status` |
| GET | `/api/v1/localization/base_footprint_pose` | `/my_ros1/localization/base_footprint_pose` |
| POST | `/api/v1/localization/relocalize` | `/my_ros1/localization/relocalize` |
| GET | `/api/v1/health` | 网关启动及服务可用性检查 |
| GET | `/docs` | FastAPI 交互式文档 |

### GET 返回格式

Topic 接收时就缓存最新 ROS 消息；后端按 1 Hz 请求 GET 即可，不需要每次重新 ROS 订阅。示例：

```json
{
  "topic": "/my_ros2/controller/status",
  "msg_type": "std_msgs/msg/String",
  "received_at": "2026-11-19T10:30:00.000Z",
  "age_ms": 150,
  "stale": false,
  "data": {
    "data": "state=ARMED|reason=ARMED|motion_allowed=1|..."
  },
  "parsed": {
    "state": "ARMED",
    "reason": "ARMED",
    "motion_allowed": true
  }
}
```

上面仅示意部分字段；`data.data` 是未经改写的 ROS String 字段。`parsed` 可能包含 `event`、`last_disarm`、`gamepad_connected`、`sticks_neutral`、`rx_twist`、`tx_twist`、`tx_topic`、`twist_age_s`，以实际收到的字段为准。**这些诊断内容不能替代机器人的安全联锁状态检查。**

位姿响应的 `data` 与 ROS2 的 `PoseStamped` 字段对应，例如：

```json
{
  "topic": "/my_ros1/localization/base_footprint_pose",
  "msg_type": "geometry_msgs/msg/PoseStamped",
  "received_at": "2026-11-19T10:30:00.000Z",
  "age_ms": 150,
  "stale": false,
  "data": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "pose": {
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
```

位姿数值和时间戳都是**格式示例**。响应中的 `received_at` 是网关收到消息的墙上时间，`data.header.stamp` 是 ROS 原始时间戳；两者不要混淆。`stale=true` 表示该缓存样本的接收时间超过阈值（控制器 3s、定位状态 4s、位姿 2s）；不会删除历史样本。首次尚未收到消息时返回 HTTP 503。

### POST 返回格式

```bash
# 只有确认机器人处于安全状态、允许重定位时才执行
curl -X POST http://127.0.0.1:8000/api/v1/localization/relocalize
```

正常调用会返回 HTTP 200，示例：

```json
{
  "service": "/my_ros1/localization/relocalize",
  "success": true,
  "message": "source=1234 map=5678 fitness_mse=0.030 overlap=0.850 translation_correction=0.120m yaw_correction=1.500deg"
}
```

- `success=false`：ROS Service 正常响应，但 FastGICP 拒绝配准/缺少数据，HTTP 仍为 200。
- HTTP 503：ROS Service 不可用，先检查 `ros1_bridge` 与 ROS2 服务列表。
- HTTP 409：之前的重定位仍在进行（包括 HTTP 超时后 ROS 仍未结束）。
- HTTP 504：超过等待时限（默认 30s）；**这并不保证 ROS 侧运算已取消**，请勿直接反复重试。

可用 `T800_API_SERVICE_TIMEOUT_S=60` 增加等待秒数，需根据 Jetson 实际 GICP 耗时调整。

**仓库已知限制**：`t800_localization` 的 `base_footprint_path.py` 默认 `only_first_gicp=true`。再次调用重定位即使 `success=true`，后续位姿节点也可能继续使用第一次的对齐结果。第一版网关不会擅自修改定位算法逻辑。

## 4. Windows 或容器内测试

```bash
curl http://127.0.0.1:8000/api/v1/health
curl http://127.0.0.1:8000/api/v1/controller/status
curl http://127.0.0.1:8000/api/v1/localization/status
curl http://127.0.0.1:8000/api/v1/localization/pose_status
curl http://127.0.0.1:8000/api/v1/localization/base_footprint_pose
```

直接在浏览器打开 `http://127.0.0.1:8000/docs` 可看到接口文档和响应。Windows 与 Jetson 不在同一容器时可以使用之前熟悉的 SSH 本地转发：

```powershell
ssh -L 8001:127.0.0.1:8000 ubuntu@Jetson_IP
```

但 **SSH 隧道要转发到实际监听 8000 的网络命名空间**：如果 FastAPI 只监听 Docker 容器内 `127.0.0.1`、容器不是 host 网络，Jetson 主机上的 SSH 隧道将无法直接接通。此时应让容器监听 `0.0.0.0` 并发布端口，或使用 host 网络。

## 5. 限制与安全约定

- 当前仅包含上述固定路由，没有附加 IP 白名单或身份认证；假设只在受控公司内网使用。请不要把开放的 POST 接口直接暴露到公网。
- 程序只订阅状态与位姿 Topic；**绝不发布速度指令或加载机器人运动控制脚本**。
- 只启动一个 Uvicorn worker；多 worker 将产生多个独立的 ROS 节点和缓存。
- 本包不会启动 `ros1_bridge`、ROS1 定位节点或厂商程序，请先验证它们已正常运行。
- 这份代码已经完成 Python 语法和非 ROS 纯函数单元测试，但尚未在真实 Jetson/ROS2 Humble + ROS1 Bridge 现场联调。

## 6. 离线单元测试

不需要 ROS 环境即可执行纯 Python 测试：

```bash
cd ~/ros2_ws/src/t800_ros_api
python3 -m unittest discover -s tests -v
```
