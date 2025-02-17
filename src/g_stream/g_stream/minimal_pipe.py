import threading
import time
import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# region pipe name
SRC_ELEMENT = "app_src"
# endregion

class GstPipelineThread:
    def __init__(self, pipeline_desc):
        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.app_src = self.pipeline.get_by_name("app_src")
        self.loop = GLib.MainLoop()
        self.running = False
        self.thread = None

    def start(self):
        """Starts the pipeline in a separate thread"""
        if self.running:
            print("Pipeline already running.")
            return

        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        """Main loop for the pipeline"""
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_message)

        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()  # Start the GMainLoop
        except Exception as e:
            print(f"Error in pipeline loop: {e}")

        # Cleanup when loop stops
        self.pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")

    def stop(self):
        """Stops the pipeline"""
        if not self.running:
            print("Pipeline is not running.")
            return

        self.running = False
        self.loop.quit()  # Stop the GMainLoop

        # Wait for the thread to exit
        if self.thread and self.thread.is_alive():
            self.thread.join()

    def _on_message(self, bus, message):
        """Handles messages from the GStreamer bus"""
        msg_type = message.type

        if msg_type == Gst.MessageType.EOS:
            print("End of stream")
            self.stop()
        elif msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            self.stop()

    def ndarray_to_gst_buffer(self, array: np.ndarray) -> Gst.Buffer:
        """Converts numpy array to Gst.Buffer"""
        return Gst.Buffer.new_wrapped(array.tobytes())

    def push_image(self, frame):
        if self.app_src:
            self.app_src.emit("push-buffer", self.ndarray_to_gst_buffer(frame))


# Example usage with a simple test video pipeline
if __name__ == "__main__":
    pipeline_desc = "videotestsrc ! autovideosink"

    gst_thread = GstPipelineThread(pipeline_desc)
    gst_thread.start()

    time.sleep(5)  # Run for 5 seconds
    print("Stopping pipeline...")
    gst_thread.stop()
