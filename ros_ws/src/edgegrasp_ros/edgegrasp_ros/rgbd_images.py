"""Decode the pinned registered Gazebo camera without guessing encodings."""

import numpy as np

from edgegrasp.rgbd import ObservationRejected


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def decode_image(message):
    formats = {"rgb8": ("u1", 3), "32FC1": ("f4", 1)}
    if message.encoding not in formats:
        raise ObservationRejected("unsupported_encoding:" + message.encoding)
    dtype, channels = formats[message.encoding]
    dtype = np.dtype((">" if message.is_bigendian else "<") + dtype)
    row_bytes = message.width * channels * dtype.itemsize
    if message.step < row_bytes or len(message.data) != message.height * message.step:
        raise ObservationRejected("invalid_image_stride")
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        message.height, message.step)[:, :row_bytes].copy()
    shape = (message.height, message.width, channels) if channels > 1 else (
        message.height, message.width)
    return raw.view(dtype).reshape(shape)


def matrix(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    t = transform.translation
    result[:3, 3] = [t.x, t.y, t.z]
    return result
