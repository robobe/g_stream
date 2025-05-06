# G_Stream
Use gstreamer to stream ROS image as udp stream using H264/H265 encoder


## Workspace
- [g_stream](src/g_stream/README.md) ros package that stream images msg as udp stream allow to control encoding settings
- [g_stream_control]() rqt plugin to control g_stream node
- [g_stream_interface]() ros2 service msg to set the request preset


## issues
[segmantion fault](http://github.com/ros2/rclpy/issues/1149)


## gstreamer
- [ ] no need for multicast flag just set the right ip address



