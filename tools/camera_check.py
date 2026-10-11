"""Check the RealSense camera: list device info, grab aligned color + depth, save a snapshot."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

OUT_DIR = Path(__file__).resolve().parent.parent / "captures"


def main() -> int:
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("No RealSense device found.")
        return 1

    dev = devices[0]
    info = {k: dev.get_info(getattr(rs.camera_info, k))
            for k in ("name", "serial_number", "firmware_version", "usb_type_descriptor")
            if dev.supports(getattr(rs.camera_info, k))}
    for k, v in info.items():
        print(f"{k:22}: {v}")
    if info.get("usb_type_descriptor", "").startswith("2"):
        print("WARNING: connected over USB 2 - use a USB 3 port/cable for full resolution.")

    pipeline = rs.pipeline()
    profile = None
    # USB 3 handles 720p; USB 2 only manages lower resolutions/rates
    for w, h, fps in ((1280, 720, 30), (640, 480, 30), (640, 480, 15)):
        config = rs.config()
        config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        try:
            profile = pipeline.start(config)
            print(f"streaming             : {w}x{h} @ {fps} fps")
            break
        except RuntimeError:
            continue
    if profile is None:
        print("Could not start any color+depth stream profile.")
        return 1
    try:
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        for _ in range(30):  # let auto-exposure settle
            pipeline.wait_for_frames()
        frames = align.process(pipeline.wait_for_frames())
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data())
    finally:
        pipeline.stop()

    h, w = depth.shape
    center_mm = depth[h // 2, w // 2] * depth_scale * 1000
    valid = np.count_nonzero(depth) / depth.size * 100
    print(f"color frame           : {color.shape[1]}x{color.shape[0]}")
    print(f"depth at center       : {center_mm:.0f} mm")
    print(f"valid depth pixels    : {valid:.1f} %")

    OUT_DIR.mkdir(exist_ok=True)
    depth_vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imwrite(str(OUT_DIR / "check_color.png"), color)
    cv2.imwrite(str(OUT_DIR / "check_depth.png"), depth_vis)
    print(f"saved snapshots to    : {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
