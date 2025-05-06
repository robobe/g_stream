# g_stream_interface

Add service to set preset

```bash
ros2 service call /stream/set_preset g_stream_interface/srv/Preset "{preset: high}"
ros2 service call /stream/set_preset g_stream_interface/srv/Preset "{preset: low}"
```