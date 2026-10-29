#!/usr/bin/env python3
"""Publish version context and a small world cloud for flight diagnosis (1 Hz)."""
import os
import math
import subprocess
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


rospy.init_node('flight_bag_context')
workspace = os.path.dirname(os.path.abspath(__file__))
version = subprocess.run(['git', '-C', workspace, 'rev-parse', 'HEAD'],
                         capture_output=True, text=True)
context_pub = rospy.Publisher('/flight_bag/version', String, queue_size=1, latch=True)
context_pub.publish(String(data=version.stdout.strip() or 'Git version unavailable'))
cloud_pub = rospy.Publisher('/flight_bag/cloud_sparse', PointCloud2, queue_size=1)
last_cloud_time = 0.0


def cloud_callback(msg):
    global last_cloud_time
    now = time.monotonic()
    if now - last_cloud_time < 1.0:
        return
    last_cloud_time = now
    # 0.20 m voxels are for bag inspection only; the controller receives full resolution.
    voxels = {}
    count = msg.width * msg.height
    step = max(1, (count + 7999) // 8000)
    pixels = [(i % msg.width, i // msg.width) for i in range(0, count, step)]
    for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True, uvs=pixels):
        if not all(math.isfinite(v) for v in point):
            continue
        key = tuple(int(v // 0.20) for v in point)
        if key not in voxels:
            voxels[key] = point
    points = list(voxels.values())
    stride = max(1, (len(points) + 1999) // 2000)
    cloud_pub.publish(pc2.create_cloud_xyz32(msg.header, points[::stride]))


rospy.Subscriber('/cloud_registered', PointCloud2, cloud_callback, queue_size=1)
rospy.spin()
