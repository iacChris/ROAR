import time
import cv2
import numpy as np
from multiprocessing import Process

def receive_csi():
    cap_csi = cv2.VideoCapture("udpsrc port=5000 ! "
                               "application/x-rtp,encoding-name=H264,payload=96 ! "
                               "rtph264depay ! "
                               "h264parse ! "
                               "avdec_h264 ! "
                               "videoconvert ! "
                               "appsink",
                               cv2.CAP_GSTREAMER)
    if not cap_csi.isOpened():
        print("VideoCapture for CSI not opened")
        return

    while True:
        ret, frame = cap_csi.read()
        if not ret:
            print("empty frame")
            break
        cv2.imshow("CSI Camera", frame)
        # Stop the program on the ESC key
        keyCode = cv2.waitKey(30) & 0xFF
        if keyCode == 27:
            break

def receive_rs():
    cap_rs = cv2.VideoCapture("udpsrc port=5001 ! "
                              "application/x-rtp,encoding-name=H264,payload=96 ! "
                              "rtph264depay ! "
                              "h264parse ! "
                              "avdec_h264 ! "
                              "videoconvert ! "
                              "appsink",
                              cv2.CAP_GSTREAMER)
    if not cap_rs.isOpened():
        print("VideoCapture for RealSense not opened")
        return

    for i in range(10):
        ret, frame = cap_rs.read()
        if not ret:
            print("empty frame")
            break
        # save for loss test
        frame_reshaped = frame.reshape(frame.shape[0], -1)
        np.savetxt(f"received_frame{i}.txt", frame_reshaped)
        # display result
        cv2.imshow("RealSense Camera", frame)

        # Depth only
        # print(f"\rframe shape: {frame.shape}", end="")
        # color, depth = np.hsplit(frame, 2)
        # depth = RgbImageToFloatArray(depth)
        # cv2.imshow("RealSense Depth", depth)
        
        # Stop the program on the ESC key
        keyCode = cv2.waitKey(30) & 0xFF
        if keyCode == 27:
            break

if __name__ == '__main__':
    csi_receiver = Process(target=receive_csi)
    rs_receiver = Process(target=receive_rs)
    csi_receiver.start()
    rs_receiver.start()
    csi_receiver.join()
    rs_receiver.join()
