# G_Stream
Use gstreamer to stream ROS image as udp stream using H264/H265 encoder



## issues
[segmantion fault](http://github.com/ros2/rclpy/issues/1149)


## gstreamer
- [ ] no need for multicast flag just set the right ip address


---

## Work with remote ssh
From remote mount sshfs to local host workspace

```bash
sshfs user@10.0.0.1:/home/user/workspaces/gst_stream_ws workspace
```

## Run 

```bash title="nvidia"
ros2 run g_stream stream_node.py --ros-args --params-file /home/user/workspace/src/g_stream/config/nvidia.yaml
```

[to read](https://www.theconstruct.ai/how-to-manipulate-parameters-at-runtime-ros2-humble-python-tutorial/)


```
gst-launch-1.0 videotestsrc ! videoconvert ! timeoverlay ! autovideosink

```