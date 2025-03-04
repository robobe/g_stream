#!/bin/bash
colcon clean workspace -y
colcon build
colcon build --packages-up-to g_stream
scp -r /workspace/install user@10.0.0.4:~/workspace