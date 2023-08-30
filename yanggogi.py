import os
import cv2
import time
import Jetson.GPIO as gp
import threading
from queue import Queue

class ArducamController:
    def __init__(self, exit_flag):
        self.exit_flag = exit_flag
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)
        gp.setup(7, gp.OUT)
        gp.setup(11, gp.OUT)
        gp.setup(12, gp.OUT)

    def capture(self, cam):
        cmd = "nvgstcapture-1.0 -A --capture-auto -S 0 --image-res=3 --file-name=capture_%d.jpg" % cam
        os.system(cmd)

    def test_cameras(self):
        print("Start testing the camera A")
        i2c = "i2cset -y 1 0x70 0x00 0x04"
        os.system(i2c)
        gp.output(7, False)
        gp.output(11, False)
        gp.output(12, True)
        self.capture(1)

        print("Start testing the camera B")
        i2c = "i2cset -y 1 0x70 0x00 0x05"
        os.system(i2c)
        gp.output(7, True)
        gp.output(11, False)
        gp.output(12, True)
        self.capture(2)

        print("Start testing the camera C")
        i2c = "i2cset -y 1 0x70 0x00 0x06"
        os.system(i2c)
        gp.output(7, False)
        gp.output(11, True)
        gp.output(12, False)
        self.capture(3)

        print("Start testing the camera D")
        i2c = "i2cset -y 1 0x70 0x00 0x07"
        os.system(i2c)
        gp.output(7, True)
        gp.output(11, True)
        gp.output(12, False)
        self.capture(4)

    def stream_video(self, cam):
        print(f"Start streaming video from camera {cam}")
        width = 1280
        height = 720

        def gstreamer_pipeline(capture_width=1280, capture_height=720, display_width=640, display_height=360, framerate=60, flip_method=0):
            return (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
                )
            )

        def frame_reader(camera, name):
            while self.exit_flag.check():
                _, frame = camera.read()
                while queues:
                    queue = queues.pop()
                    queue.put(frame)

        def get_frame(camera, timeout=None):
            queue = Queue(1)
            queues.append(queue)
            return queue.get(timeout=timeout)

        def previewer(camera, name):
            window_name = "Arducam"
            while self.exit_flag.check():
                cv2.imshow(window_name, get_frame(camera, 2000))
                keyCode = cv2.waitKey(16) & 0xFF
            cv2.destroyWindow(window_name)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0, display_height=720, display_width = 1280), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError("Failed to open camera!")

        queues = []

        frame_reader_thread = threading.Thread(target=frame_reader, args=(cap, ""))
        frame_reader_thread.daemon = True
        frame_reader_thread.start()

        previewer_thread = threading.Thread(target=previewer, args=(cap, ""))
        previewer_thread.daemon = True
        previewer_thread.start()

        while self.exit_flag.check():
            time.sleep(10)

        cap.release()
        cv2.destroyAllWindows()

        gp.cleanup()

if __name__ == "__main__":

    class ExitFlag:
        def __init__(self):
            self.flag = True

        def on(self):
            self.flag = False
        def check(self):
            return self.flag

    ef = ExitFlag()
    import signal, sys

    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        ef.on()
        print('System shutdown in 3 secs')
        time.sleep(3)    
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    controller = ArducamController(ef)
    controller.stream_video(1)
