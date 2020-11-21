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
    profile = pipeline.start(CONFIG)
    # align depth2color
    align = rs.align(rs.stream.color)
    # colorizer
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 2);  # white to black
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() 
            aligned_color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not aligned_color_frame:
                continue

            # colorize depth image and get color image
            colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())

            # Stack both color and colorized depth horizontally
            images = np.hstack((color_image, colorized_depth))
            
            print(f'\rcolor shape: {color_image.shape}, colorized_depth shape: {colorized_depth.shape}, images shape: {images.shape}', end='')

            # Show images
            cv2.namedWindow('RealSense Camera Test', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)

            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                print()
                break

    finally:
        # Stop streaming
        pipeline.stop()


if __name__ == "__main__":
    RealSense_test()
