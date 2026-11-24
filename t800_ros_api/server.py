"""FastAPI front end for the T800 ROS2 gateway."""

import asyncio
import os
import threading
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException

from .ros_gateway import  TOPICS


@asynccontextmanager
async def lifespan(app):
    # Import/init ROS here: this process owns one executor and one ROS node.
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
        worker = threading.Thread(target=executor.spin, daemon=True,
                                  name='t800_ros_executor')
        worker.start()
        app.state.gateway = gateway
        app.state.relocalize_future = None
        app.state.relocalize_lock = asyncio.Lock()
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
    description='Read cached ROS2 topics and trigger T800 localization.',
    version='0.1.0',
    lifespan=lifespan,
)


def _gateway():
    gateway = getattr(app.state, 'gateway', None)
    if gateway is None:
        raise HTTPException(503, 'ROS gateway is not started')
    return gateway


def _get_topic(key):
    _, _, _, default_max_age_s = TOPICS[key]
    gateway = _gateway()
    result = gateway.cache.latest(key, default_max_age_s)
    if result is None:
        raise HTTPException(503, 'No ROS message has been received for this topic')
    # A stale sample is returned with stale=true, not hidden from the caller.
    return result


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
        'topics_received': gateway.cache.seen()
    }




def main():
    host = os.getenv('T800_API_HOST', '127.0.0.1')
    port = int(os.getenv('T800_API_PORT', '8001'))
    # Keep workers=1; each worker would otherwise own a separate ROS node/cache.
    uvicorn.run(app, host=host, port=port, workers=1, reload=False)


if __name__ == '__main__':
    main()
