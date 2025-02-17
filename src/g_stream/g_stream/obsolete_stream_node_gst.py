import gi
import numpy as np
gi.require_version("Gst", "1.0")

from gi.repository import GLib, Gst

# region gst
Gst.init(None)

app_source = None
pipeline = None

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

def change_bitrate(rate):
    def execute(rate):
        element = pipeline.get_by_name("codec")
        element.set_property("bitrate", rate)

    GLib.idle_add(execute, rate)

def change_codec_prop(prop, value):
    def execute():
        element = pipeline.get_by_name("codec")
        element.set_property(prop, value)

    GLib.idle_add(execute)

def gst_runner(bitrate):
    PIPELINE = "appsrc name=app_src \
        ! video/x-raw,width={width},height={height},format=BGR,framerate={fps}/1 \
        ! videoconvert \
        ! x264enc name=codec bitrate={bitrate} qp-max=50 qp-min=10 speed-preset=ultrafast tune=zerolatency \
        ! rtph264pay mtu={MTU} \
        ! udpsink host={host} port={port} {multicast} sync=false".format(
        width=640,
        height=512,
        fps=9,
        bitrate=300,
        MTU=1500,
        host="127.0.0.1",
        port=5000,
        multicast="auto-multicast=false",
    )

    """
    gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay  ! avdec_h264 ! videoconvert ! fpsdisplaysink
    """

    global pipeline
    pipeline = Gst.parse_launch(PIPELINE)
    global app_source
    app_source = pipeline.get_by_name("app_src")

    bus = pipeline.get_bus()
    # allow bus to emit messages to main thread
    bus.add_signal_watch()
    # Start pipeline
    pipeline.set_state(Gst.State.PLAYING)
    # Init GObject loop to handle Gstreamer Bus Events
    loop = GLib.MainLoop()
    # Add handler to specific signal
    bus.connect("message", on_message, loop)
    try:
        loop.run()
    except Exception:
        loop.quit()


def ndarray_to_gst_buffer(array: np.ndarray) -> Gst.Buffer:
    """Converts numpy array to Gst.Buffer"""
    return Gst.Buffer.new_wrapped(array.tobytes())

def push_image(frame):
    if app_source:
        app_source.emit("push-buffer", ndarray_to_gst_buffer(frame))

#endregion
