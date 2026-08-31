"""Public LiDAR/IMU stream configuration and parsing without WebRTC."""

import asyncio
import json

import pytest

from nori_sdk.mock.loopback import loopback_pair
from nori_sdk.teleop import RemoteTeleop, TeleopError
from nori_sdk.types import ImuSample, LidarScan, SensorStreamStatus


class Channel:
    readyState = "open"

    def __init__(self):
        self.sent = []

    def send(self, raw):
        self.sent.append(json.loads(raw))


def session(capabilities=("sensor_streams",)):
    signaling, _robot_side = loopback_pair()
    robot = RemoteTeleop(signaling)
    channel = Channel()
    robot._control = channel
    robot._handle_frame(json.dumps({
        "type": "ack",
        "accepted": True,
        "protocol_version": 1,
        "capabilities": list(capabilities),
    }))
    return robot, channel


@pytest.mark.asyncio
async def test_configure_streams_is_correlated_and_caches_status():
    robot, channel = session()
    pending = asyncio.create_task(robot.configure_sensor_streams(
        lidar_hz=5,
        imu_hz=20,
        lidar_max_points=180,
    ))
    await asyncio.sleep(0)
    request = channel.sent[0]
    assert request == {
        "type": "sensor_stream",
        "request_id": request["request_id"],
        "action": "configure",
        "lidar_hz": 5,
        "imu_hz": 20,
        "lidar_max_points": 180,
    }
    robot._handle_frame(json.dumps({
        "type": "sensor_stream_status",
        "request_id": request["request_id"],
        "ok": True,
        "lidar_hz": 5,
        "imu_hz": 20,
        "lidar_max_points": 180,
        "lidar_available": True,
        "imu_available": False,
    }))
    status = await pending
    assert isinstance(status, SensorStreamStatus)
    assert status.lidar_hz == 5
    assert status.imu_available is False
    assert robot.sensor_stream_status is status


def test_lidar_and_imu_are_typed_stream_events_and_latest_snapshots():
    robot, _channel = session()
    seen = []
    robot.on("lidar_scan", seen.append)
    robot._handle_frame(json.dumps({
        "type": "lidar_scan",
        "stamp": {"sec": 12, "nanosec": 34},
        "frame_id": "laser",
        "angle_min_rad": -1.0,
        "angle_max_rad": 1.0,
        "angle_increment_rad": 0.5,
        "time_increment_s": 0.001,
        "scan_time_s": 0.1,
        "range_min_m": 0.05,
        "range_max_m": 12.0,
        "source_points": 8,
        "ranges_m": [1.2, None, 2.3],
        "intensities": [5.0, None, 8.0],
    }))
    assert isinstance(robot.lidar_scan, LidarScan)
    assert robot.lidar_scan.stamp.sec == 12
    assert robot.lidar_scan.ranges_m == (1.2, None, 2.3)
    assert seen == [robot.lidar_scan]

    robot._handle_frame(json.dumps({
        "type": "imu",
        "stamp": {"sec": 56, "nanosec": 78},
        "frame_id": "imu_link",
        "orientation_xyzw": [0.0, 0.0, 0.5, 0.866],
        "orientation_covariance": list(range(9)),
        "angular_velocity_rad_s": [None, 0.0, 0.3],
        "angular_velocity_covariance": [0.0] * 9,
        "linear_acceleration_m_s2": [0.0, 0.0, 9.81],
        "linear_acceleration_covariance": [1.0] * 9,
    }))
    assert isinstance(robot.imu_sample, ImuSample)
    assert robot.imu_sample.angular_velocity_rad_s == (None, 0.0, 0.3)
    assert robot.imu_sample.linear_acceleration_m_s2[-1] == 9.81


@pytest.mark.asyncio
async def test_invalid_or_explicitly_unsupported_requests_do_not_send():
    robot, channel = session()
    with pytest.raises(ValueError, match="between 0 and 50"):
        await robot.configure_sensor_streams(imu_hz=51)
    assert channel.sent == []

    unsupported, unsupported_channel = session(("record",))
    with pytest.raises(TeleopError, match="sensor_streams"):
        await unsupported.get_sensor_stream_status()
    assert unsupported_channel.sent == []
