
# sequentially append all videos in a directory to a single video per camera
# clips will start with camera0, then camera2, then camera4

import cv2
import os
import sys
import time

def append_videos(directory):
    """
    append all videos in a directory to a single video per camera
    :param directory: directory containing videos
    :return: None
    """
    # get all files in the directory
    files = os.listdir(directory)
    # sort the files
    files.sort()

    # get the number of cameras from the filenames. not more than 3 cameras
    cameras = []
    for file in files:
        camera = file.split("_")[0]
        if camera not in cameras:
            cameras.append(camera)

    # create a video for each camera
    # get the first record time
    t_0 = files[0].split("_")[1]

    for camera in cameras:
        # get all the videos for the camera
        camera_files = [file for file in files if file.startswith(camera)]
        # sort the files
        camera_files.sort()
        # get the first file
        first_file = camera_files[0]
        # get the first file's date and time
        date = first_file.split("_")[1]
        time = first_file.split("_")[2]
        # create the filename
        filename = f"{camera}_{date}_{t_0}.avi"
        # create the video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))
        # write all the frames to the video
        for file in camera_files:
            cap = cv2.VideoCapture(file)
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
                out.write(frame)
                cv2.imshow('frame', frame)
            cap.release()
        out.release()
        cv2.destroyAllWindows()

    return None

if __name__ == "__main__":
    # dir = the current directory
    dir = os.getcwd()
    append_videos(dir)
    # append_videos(sys.argv[1])

