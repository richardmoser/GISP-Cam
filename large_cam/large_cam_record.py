"""
Author: Richard Moser
Date Modified: 03Dec24
Purpose: Record video from multiple usb cameras simultaneously
"""
import sys
import os
import time
os.environ["OPENCV_LOG_LEVEL"]="FATAL"
import cv2
from gpiozero import Button
# from gpiozero import LED
from gpiozero import PWMLED

# create a button object on GPIO pin 2 to start recording
button = Button(2)

# create an LED object on GPIO pin 17 to indicate recording
led = PWMLED(17)

# if there is a first argument, show = True
if len(sys.argv) > 1:
    show = sys.argv[1]
else:
    show = False


def record(show=False):

    cams = []
    num_cams = 5  # number of possible cameras
    for i in range(num_cams):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera {i} is available")
            cams.append(cap)
        else:
            cap.release()
            cv2.destroyAllWindows()


    # the file name is f"camera{camera_index}_{date}_{time}.avi"
    date_of_rec = time.strftime("%Y-%m-%d")
    time_of_rec = time.strftime("%H-%M-%S")

    # make a directory named after the date and time
    dir_name = f"recordings/{date_of_rec}_{time_of_rec}_captures"
    os.mkdir(dir_name)

    print(f"Recording to {dir_name}")
    led.value = 0.4

    frame_width = int(cams[0].get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cams[0].get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cams[0].get(cv2.CAP_PROP_FPS))


    # Define codec and create VideoWriter objects for each camera
    fourcc = cv2.VideoWriter_fourcc(*'XVID')

    outs = []
    for i in range(len(cams)):
        out = cv2.VideoWriter(f"{dir_name}/camera{i}_{date_of_rec}_{time_of_rec}.avi", fourcc, fps, (frame_width, frame_height))
        outs.append(out)


    while True:
        rets = []  # list of return values from each camera
        frames = []  # list of frames from each camera
        for cam in cams:
            ret, frame = cam.read()  # read frame from camera
            rets.append(ret)  # append return value to list
            frames.append(frame)  # append frame to list

        for i in range(len(cams)):
            if rets[i]:  # if the return value is True
                if show:
                    cv2.imshow(f'Cam {i}', frames[i])  # show the frame
                outs[i].write(frames[i])  # write the frame to the video file

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if button.is_pressed:
            break


    for cam in cams:
        cam.release()
    for out in outs:
        out.release()

    cv2.destroyAllWindows()
    print("Recording stopped")
    led.off()
    return

def blink_until_pressed():
    while True:
        # led.on()
        led.value = 0.2
        time.sleep(0.25)
        led.off()
        time.sleep(2)
        if button.is_pressed:
            for i in range(5):
                led.value = 0.1
                time.sleep(0.05)
                led.value = 0
                time.sleep(0.05)
            break
    return

if __name__ == "__main__":
    # button.wait_for_press()
    blink_until_pressed()
    print('Starting recording')
    record(show)
    led.off()

    exit()