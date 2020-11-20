# Streaming between Jetson and Windows

**On the Jetson Nano**, make sure both your RealSense camera and CSI camera are working properly. You can try to run the following testing scripts.
```
$ python3 realsense_test.py
$ python3 csi_test.py
```
You should see window pops up and displays the video captured by the camera.

---

**On the Windows**, figure out your ip address on the local network. Below is the command I used in Windows command prompt.
```
> ipconfig
```
Record your IPv4 address.

---

**On the Jetson Nano**, modify the `CLIENT_IP` field inside [basic_streaming.py](https://github.com/iacChris/ROAR/blob/basic_streaming/basic_streaming/Jetson/basic_streaming.py#L6) to match the client address (Windows IPv4 address) you just figured out.

Then, run the `basic_streaming.py` to start streaming.
```
$ python3 basic_streaming.py
```

---

**On the Windows**, run the receiver scripts as following.
```
> start port5000_csi.bat
> start port5001_rs.bat
```
Alternatively, if you have built OpenCV with GStreamer, you can simply run the following command.
```
> python basic_receiver.py
```
Then, you should see two video streaming from the front RealSense camera and the rear CSI camera.

# Streaming between Windows
**On the Windows**, make sure the RealSense Camera is connected, and run the following scripts to send and receive on the same Windows.
```
> start send_rs.bat
> start receive_rs.bat
```