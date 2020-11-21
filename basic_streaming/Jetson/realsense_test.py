import cv2
import numpy as np
import pyrealsense2 as rs
import time

# Camera Configuration
CONFIG = rs.config()
CONFIG.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
CONFIG.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

def RealSense_test():
    pipeline = rs.pipeline()
    pipeline.start(CONFIG)
    # colorizer
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 2);  # white to black
    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # pyrealsense colorzier
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())


            # Stack both images horizontally
            # images = np.hstack((color_image, depth_colormap))
            images = np.hstack((color_image, colorized_depth))

            # Show images
            cv2.namedWindow('RealSense Camera Test', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)

            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break

    finally:
        # Stop streaming
        pipeline.stop()


if __name__ == "__main__":
    RealSense_test()
