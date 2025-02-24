#!/usr/bin/env python3


import sys
import numpy as np
import traceback
import cv2
import gi
import threading
import time
import queue
from bfid import extract_binary_from_image
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

PIPELINE = """udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 \
    ! rtpjitterbuffer latency=10 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,width=640,height=480,format=BGR  \
    ! appsink name=sink sync=true max-buffers=1 drop=true emit-signals=true
"""




def on_message(bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop):
    mtype = message.type
    if mtype == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

    elif mtype == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(err, debug)
        loop.quit()

    elif mtype == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(err, debug)

    return True

def frame_handler(sink):
    sample = sink.emit("pull-sample")
    caps = sample.get_caps()

    width = caps.get_structure(0).get_value("width")
    height = caps.get_structure(0).get_value("height")
    buf = sample.get_buffer()
    frame = np.ndarray((height, width, 3),
               buffer = buf.extract_dup(0,buf.get_size()),
               dtype=np.uint8)
    sec, nanosec = extract_binary_from_image(frame, bit_size=3)
    cv2.imshow("debug", frame)
    cv2.waitKey(1)
    
    # print(threading.current_thread().getName())
    return Gst.FlowReturn.OK

def gst_runner():
    pipeline = Gst.parse_launch(PIPELINE)
    app_sink = pipeline.get_by_name("sink")
    app_sink.connect("new-sample", frame_handler)
    bus = pipeline.get_bus()
    # allow bus to emit messages to main thread
    bus.add_signal_watch()
    pipeline.set_state(Gst.State.PLAYING)
    loop = GLib.MainLoop()
    bus.connect("message", on_message, loop)



    try:
        loop.run()
    except Exception:
        traceback.print_exc()
        loop.quit()

# Stop Pipeline
    pipeline.set_state(Gst.State.NULL)

cv2.namedWindow("debug", cv2.WINDOW_NORMAL)


if __name__ == "__main__":
    gst_runner()
