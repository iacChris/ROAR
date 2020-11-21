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
    
    ##### Depth Alignment #####
    # for depth alignment
    # depth_sensor = profile.get_device().first_depth_sensor()
    # depth_scale = depth_sensor.get_depth_scale()
    # print("Depth Scale is: " , depth_scale)
    #
    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    # clipping_distance_in_meters = 1 #1 meter
    # clipping_distance = clipping_distance_in_meters / depth_scale
    # align depth to color
    # align = rs.align(rs.stream.color)
    ###########################

    # colorizer
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 2);  # white to black
    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            ##### Depth Alignment #####
            # Align the depth frame to color frame
            # aligned_frames = align.process(frames)
            # Get aligned frames
            # aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            # aligned_color_frame = aligned_frames.get_color_frame()
            # if not aligned_depth_frame or not aligned_color_frame:
            #     continue
            ###########################

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # pyrealsense colorzier
            colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())


            # Stack both images horizontally
            # images = np.hstack((color_image, depth_colormap))
            # images = np.hstack((color_image, colorized_depth))
            
            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # Render images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((bg_removed, depth_colormap))

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
