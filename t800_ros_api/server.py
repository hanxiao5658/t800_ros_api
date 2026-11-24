"""T800 ROS2 Humble 的 FastAPI REST 网关。

运行环境：Docker3 / ROS2 Humble。

保留现有四个定位/控制 GET、运动状态 GET、两个运动状态切换 POST；
新增：
  GET /api/v1/navigation/state
       读取 /my_ros1/navigation/state（Docker2 桥接到 ROS2 的标准 String）

本文件只负责 HTTP 请求和 ROS2 Topic 缓存查询；
控制类 POST 仍调用已有的运动状态切换 Service，不发布运动指令。
"""

import asyncio
import os
import threading
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException

from .ros_gateway import MOTION_SWITCH_SERVICES, TOPICS

# 等待 ROS2 Trigger 返回的 HTTP 超时时间（默认 6 秒）。
# 上游 my_switch_robotState.py 通常最多等待 3 秒状态确认。
# HTTP 超时不保证 ROS Service 已取消；应先核实实际状态再重试。
MOTION_SWITCH_TIMEOUT_S = float(os.getenv('T800_API_MOTION_TIMEOUT_S', '6.0'))
if MOTION_SWITCH_TIMEOUT_S <= 0:
    raise ValueError('T800_API_MOTION_TIMEOUT_S must be > 0')


@asynccontextmanager
async def lifespan(app):
    """API 启停期间创建一个 ROS2 节点及后台 Executor 线程。"""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from .ros_gateway import RosGateway

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=2)
    gateway = None
    worker = None
    try:
        gateway = RosGateway()
        executor.add_node(gateway)
        worker = threading.Thread(
            target=executor.spin,
            daemon=True,
            name='t800_ros_executor',
        )
        worker.start()
        app.state.gateway = gateway
        # 避免上一次 ROS Service 仍在执行时再次发起运动切换。
        app.state.motion_pending = None
        yield
    finally:
        executor.shutdown(timeout_sec=5.0)
        if worker is not None:
            worker.join(timeout=5.0)
        if gateway is not None:
            executor.remove_node(gateway)
            gateway.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


app = FastAPI(
    title='T800 ROS REST API',
    description='Read ROS2 states and invoke allowed motion-switch Trigger services.',
    version='0.2.0',
    lifespan=lifespan,
)


def _gateway():
    gateway = getattr(app.state, 'gateway', None)
    if gateway is None:
        raise HTTPException(503, 'ROS gateway is not started')
    return gateway


def _get_topic(key):
    """查询由 ROS2 订阅回调缓存的最新消息；不在 HTTP 回调内阻塞等 Topic。"""
    _, _, _, max_age_s = TOPICS[key]
    result = _gateway().cache.latest(key, max_age_s)
    if result is None:
        raise HTTPException(503, 'No ROS message has been received for this topic')
    # 与旧版接口一致：即使缓存过期也返回样本，并通过 stale=true 标记。
    return result


# ============================================================
# 原有 GET 接口：路径与返回格式完全保留
# ============================================================

@app.get('/api/v1/controller/status', tags=['Controller'])
def controller_status():
    return _get_topic('controller/status')


@app.get('/api/v1/localization/status', tags=['Localization'])
def localization_status():
    return _get_topic('localization/status')


@app.get('/api/v1/localization/pose_status', tags=['Localization'])
def localization_pose_status():
    return _get_topic('localization/pose_status')


@app.get('/api/v1/localization/base_footprint_pose', tags=['Localization'])
def localization_base_footprint_pose():
    return _get_topic('localization/base_footprint_pose')


@app.get('/api/v1/health', tags=['Diagnostics'])
def health():
    gateway = _gateway()
    return {
        'gateway': 'running',
        'topics_received': gateway.cache.seen(),
    }


# ============================================================
# 已有运动状态 GET 接口：保持原路径
# ============================================================

@app.get('/api/v1/my_ros2/motion/motion_state', tags=['Motion'])
def motion_state():
    """获取最近一次机器人运动状态（String.data 为 JSON）。"""
    return _get_topic('motion/motion_state')


# ============================================================
# 新增导航状态 GET：只读，不调用导航 start/pause 等控制服务
# ============================================================

@app.get('/api/v1/navigation/state', tags=['Navigation'])
def navigation_state():
    """获取最近一次 ROS1 导航状态。

    ROS2 订阅源：/my_ros1/navigation/state，std_msgs/msg/String。

    响应字段包含原始 data.data、解析后的 parsed，以及：
      received_at：网关接收到消息的 UTC 时间；
      age_ms：距离网关收到该消息的时间差（毫秒）；
      stale：缓存年龄是否超过本 Topic 的提示阈值。

    注意：导航状态是锁存式事件消息，不是连续心跳；
    stale=False 不代表传感器、定位或机器人一定安全。
    """
    return _get_topic('navigation/state')


# ============================================================
# 原有 POST 接口：通过 ROS2 Trigger Service 切换机器人运动模式
# ============================================================

async def _call_motion_switch(action):
    """异步调用已有 ROS2 Trigger Service，不阻塞 FastAPI 和 Executor。

    - 由 my_switch_robotState.py 核对机器人原始状态及反馈。
    - 不允许未完成的请求与新的运动切换请求并发。
    - HTTP 等待超时不代表 ROS 请求已取消。
    """
    gateway = _gateway()
    service = MOTION_SWITCH_SERVICES[action]

    # 当前协程在这里没有 await；检查和设置操作不会被其他协程交错。
    previous = getattr(app.state, 'motion_pending', None)
    if previous is not None and not previous.done():
        raise HTTPException(
            409,
            'A previous motion switch request is still pending; '
            'verify robot state before retrying',
        )

    if not gateway.motion_switch_is_ready(action):
        raise HTTPException(503, 'ROS2 service is not available: ' + service)

    loop = asyncio.get_running_loop()
    result_future = loop.create_future()

    try:
        ros_future = gateway.call_motion_switch(action)
    except Exception as exc:
        raise HTTPException(502, 'Could not call ROS2 service %s: %s' % (service, exc))

    app.state.motion_pending = result_future

    def on_ros_completed(completed):
        """ROS Future 在 Executor 线程完成，结果送回 asyncio 主线程。"""
        def deliver_result():
            if result_future.done():
                return
            try:
                result = completed.result()
                if result is None:
                    raise RuntimeError('ROS2 service returned no response')
                result_future.set_result(result)
            except Exception as exc:
                result_future.set_exception(exc)

        try:
            loop.call_soon_threadsafe(deliver_result)
        except RuntimeError:
            # 服务器已关闭，不能往已关闭的事件循环投递结果。
            pass

    ros_future.add_done_callback(on_ros_completed)

    try:
        response = await asyncio.wait_for(
            asyncio.shield(result_future),
            timeout=MOTION_SWITCH_TIMEOUT_S,
        )
    except asyncio.TimeoutError:
        raise HTTPException(
            504,
            'Timed out waiting for %s; ROS request may still be running. '
            'Check /my_ros2/motion/motion_state before retrying.' % service,
        )
    except Exception as exc:
        raise HTTPException(502, 'ROS2 service call failed: %s (%s)' % (service, exc))

    # 即使 ROS success=False，也返回 HTTP 200 并保留原始 message。
    return {
        'service': service,
        'success': bool(response.success),
        'message': str(response.message),
    }


@app.post('/api/v1/my_ros2/switch_motion_srv/set_to_walking', tags=['Motion'])
async def set_to_walking():
    """请求 lower_body_balance -> rl_basic；服务端继续进行状态门控。"""
    return await _call_motion_switch('set_to_walking')


@app.post('/api/v1/my_ros2/switch_motion_srv/set_to_lowerBodyBalance', tags=['Motion'])
async def set_to_lower_body_balance():
    """请求 rl_basic -> lower_body_balance；服务端继续进行状态门控。"""
    return await _call_motion_switch('set_to_lowerBodyBalance')


def main():
    host = os.getenv('T800_API_HOST', '127.0.0.1')
    # 与现有仓库配置保持一致：Docker3 默认端口是 8001。
    port = int(os.getenv('T800_API_PORT', '8001'))
    # 只能启动一个 worker，否则每个 worker 会生成独立 ROS 节点和缓存。
    uvicorn.run(app, host=host, port=port, workers=1, reload=False)


if __name__ == '__main__':
    main()
