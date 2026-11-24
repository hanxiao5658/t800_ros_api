"""T800 ROS2 Humble 的 FastAPI REST 网关。

运行在 Docker3；保留现有四个 GET Topic + health，新增：
  GET  /api/v1/my_ros2/motion/motion_state
  POST /api/v1/my_ros2/switch_motion_srv/set_to_walking
  POST /api/v1/my_ros2/switch_motion_srv/set_to_lowerBodyBalance

状态切换必须由 engineai_pose_controller/my_switch_robotState.py
提供的两个 ROS2 std_srvs/srv/Trigger Service 实现，本文件不发布运动命令。
"""

import asyncio
import os
import threading
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException

from .ros_gateway import MOTION_SWITCH_SERVICES, TOPICS

# HTTP 等待 ROS2 Trigger 返回的上限（默认 6 秒）。
# 上游 my_switch_robotState.py 默认等待厂商状态反馈 3 秒。
# HTTP 超时不保证 ROS Service 已取消；期间禁止重复发起切换。
MOTION_SWITCH_TIMEOUT_S = float(os.getenv('T800_API_MOTION_TIMEOUT_S', '6.0'))
if MOTION_SWITCH_TIMEOUT_S <= 0:
    raise ValueError('T800_API_MOTION_TIMEOUT_S must be > 0')


@asynccontextmanager
async def lifespan(app):
    # 网关只创建一个 ROS2 Node，由独立线程持续处理订阅与 Service 回调。
    # 不通过 FastAPI 同步线程调用 rclpy.spin_once，避免执行器冲突。
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
        # 已发起但尚未返回的运动模式切换：包括 HTTP 超时后的请求。
        # 避免用户重复点击时发生两个并行的 ROS Service 调用。
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
    _, _, _, max_age_s = TOPICS[key]
    result = _gateway().cache.latest(key, max_age_s)
    if result is None:
        raise HTTPException(503, 'No ROS message has been received for this topic')
    # 保留旧版行为：过期数据也返回，但明确标记 stale=true。
    return result


# ============================================================
# 已有 GET 接口：保持路径和返回格式不变
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
# 新增 GET 接口：路径与 ROS2 Topic 名称一致（在前面添加 /api/v1）
# ============================================================

@app.get('/api/v1/my_ros2/motion/motion_state', tags=['Motion'])
def motion_state():
    """获取最新机器人运动状态（ROS2 标准 String，data 中为 JSON）。"""
    return _get_topic('motion/motion_state')


# ============================================================
# 新增 POST 接口：通过 ROS2 Trigger Service 切换机器人运动状态
# ============================================================

async def _call_motion_switch(action):
    """异步调用服务，避免阻塞 FastAPI 和 ROS2 Executor。

    - ROS2 服务端决定前置状态是否合法，HTTP 不擅自跳过检查。
    - 同一时刻只允许有一个未完成的运动切换请求。
    - 若 HTTP 超时，仍锁住后续切换请求，直至原 ROS Future 真正结束。
      这样不会在状态不明时继续叠加运动模式切换命令。
    """
    gateway = _gateway()
    service = MOTION_SWITCH_SERVICES[action]

    # 同一 Uvicorn 事件循环中，此段没有 await，因此检查与设置不会交错。
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
        """ROS Future 在 Executor 线程完成，安全地传回 asyncio 事件循环。"""
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
            # HTTP 服务器正在停止，不能在已关闭事件循环投递回调。
            pass

    ros_future.add_done_callback(on_ros_completed)

    try:
        # shield：HTTP 超时不取消正在执行的 ROS Service。
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

    # 业务层成功/失败完全依赖 my_switch_robotState.py 的 Trigger 返回。
    # ROS2 success=False 时 HTTP 仍返回 200，并原样保留 message。
    return {
        'service': service,
        'success': bool(response.success),
        'message': str(response.message),
    }


@app.post('/api/v1/my_ros2/switch_motion_srv/set_to_walking', tags=['Motion'])
async def set_to_walking():
    """请求 lower_body_balance -> rl_basic。具体检查由 ROS2 服务端完成。"""
    return await _call_motion_switch('set_to_walking')


@app.post('/api/v1/my_ros2/switch_motion_srv/set_to_lowerBodyBalance', tags=['Motion'])
async def set_to_lower_body_balance():
    """请求 rl_basic -> lower_body_balance。具体检查由 ROS2 服务端完成。"""
    return await _call_motion_switch('set_to_lowerBodyBalance')


def main():
    host = os.getenv('T800_API_HOST', '127.0.0.1')
    # 保留当前仓库的默认端口 8001，不擅自改回此前旧版本的 8000。
    port = int(os.getenv('T800_API_PORT', '8001'))
    # 只开一个 worker，否则每个 worker 会有独立 ROS 节点和状态缓存。
    uvicorn.run(app, host=host, port=port, workers=1, reload=False)


if __name__ == '__main__':
    main()
