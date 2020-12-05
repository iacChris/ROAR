import numpy as np
import cv2
import pyrealsense2 as rs
from multiprocessing import Process

from depth_image_encoding import FloatArrayToRgbImage

# CLIENT_IP='192.168.1.74' # you need to modify this line match the ip addr. of your client
CLIENT_IP='192.168.1.19'

def gstreamer_pipeline(
    capture_width=3280, 
    capture_height=2464, 
    output_width=224, 
    output_height=224, 
    framerate=21, 
    flip_method=0
    ):   
        return (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "nvvidconv ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "appsink" 
            % (
                capture_width, 
                capture_height, 
                framerate, 
                flip_method, 
                output_width, 
                output_height
            )
        )

def send_csi():
    # initialize CSI camera
    cap_send =cv2.VideoCapture(
            gstreamer_pipeline(
                capture_width =1280,
                capture_height =720,
                output_width=1280,
                output_height=720,
                framerate=30,
                flip_method=0),
            cv2.CAP_GSTREAMER)
    # initialize GStreamer Sender pipeline for CSI, using port 5000
    out_send = cv2.VideoWriter(
                    "appsrc ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)BGRx, width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)I420 ! "
                    "omxh264enc control-rate=2 bitrate=8000000 ! "
                    "video/x-h264, stream-format=byte-stream ! "
                    "rtph264pay mtu=1400 ! "
                    "udpsink host=%s port=5000 sync=false async=false"
                    % (
                        CLIENT_IP
                    ),
                    apiPreference=cv2.CAP_GSTREAMER,
                    fourcc=0,
                    fps=30,
                    frameSize=(1280,720), 
                    isColor=True
                )
    # check sender is properly opened
    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened')
        exit(0)
    # actual streaming data
    while True:
        ret, frame = cap_send.read()
        if not ret:
            print('empty frame')
            break
        out_send.write(frame)

        ##### Optional: displaying what is beint sent
        ##### Note: it will cause the streaming to be laggy!
        # cv2.imshow("CSI Camera Streaming", frame)
        # keyCode = cv2.waitKey(30) & 0xFF
        # # Stop the program on the ESC key
        # if keyCode == 27:
        #     break
        ##### Optional End
    # cleaning at end
    cap_send.release()
    out_send.release()

def send_rs():
    # initialize RealSense camera
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # color camera
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)  # depth camera
    pipeline.start(cfg)
    # initialize GStreamer Sender pipeline for RealSense color and depth, using port 5001
    out_send_both = cv2.VideoWriter(
                    "appsrc ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)BGRx, width=(int)1280, height=(int)480, framerate=(fraction)30/1 ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)I420 ! "
                    "omxh264enc control-rate=2 bitrate=4000000 ! "
                    "video/x-h264, stream-format=byte-stream ! "
                    "rtph264pay mtu=1400 ! "
                    "udpsink host=%s port=5001 sync=false async=false"
                    % (
                        CLIENT_IP
                    ),
                    apiPreference=cv2.CAP_GSTREAMER,
                    fourcc=0,
                    fps=30,
                    frameSize=(640*2,480), 
                    isColor=True
                )
    # align depth2color
    align = rs.align(rs.stream.color)
    # actual streaming data
    while True:
        # wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        # align the depth frame to color frame
        aligned_frames = align.process(frames)
        # get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() 
        aligned_color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not aligned_color_frame:
            continue
        # colorize depth image and get color image
        depth_image = np.asanyarray(aligned_depth_frame.get_data())/1000
        colorized_depth = FloatArrayToRgbImage(depth_image)
        color_image = np.asanyarray(aligned_color_frame.get_data())
        # stack both color and colorized depth horizontally
        images = np.hstack((color_image, colorized_depth))
        # save for loss test
        np.savetxt("send_frame.txt", images, delimiter=',')
        # send using GStreamer
        out_send_both.write(images)

        # print(f'\rcolor shape: {color_image.shape}, colorized_depth shape: {colorized_depth.shape}, images shape: {images.shape}', end='')

        ##### Optional: displaying what is beint sent
        ##### Note: it will cause the streaming to be laggy!
        # cv2.imshow("RealSense Camera Streaming", images)
        # keyCode = cv2.waitKey(30) & 0xFF
        # # Stop the program on the ESC key
        # if keyCode == 27:
        #     break
        ##### Optional End
    # cleaning at end
    out_send.release()

if __name__ == '__main__':
    csi_cam = Process(target=send_csi)
    realsense_cam = Process(target=send_rs)
    csi_cam.start()
    realsense_cam.start()
    csi_cam.join()
    realsense_cam.join()
