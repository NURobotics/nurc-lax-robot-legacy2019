# Object Tracking

This is where all the code for finding the ball from camera video.
Also check our team's folder in the NURC Google Drive for more information.

## Building the code
To build, run the following commands after every change to main.cpp:

cd build
cmake ..
make
./cv_nurc_lax


Any issues - try deleting everything in the /build folder first to clear all the cache for CMake

## Changing camera formats

To make sure the cameras are using th correct image format for the fastest frame rate

0. Add this code below opening the VideoCapture (assuming the video captue's name is cap):
	int fps = 60;
	cap.set(CV_CAP_PROP_FPS, fps);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	0b. If you're lucky that's the only step you'll have to do, so test the code to see if it works

1. Plug camera into device
2. Open terminal
3. Run *v4l2-ctl --list-devices*
	3a. The camera's index is the number at the end of /dev/video
4. Run *v4l2-ctl -d <device index from 3a> --list-formats-ext*
	4a. You should now see something like this:
		ioctl: VIDIOC_ENUM_FMT
			Index       : 0
			Type        : Video Capture
			Pixel Format: 'MJPG' (compressed)
			Name        : Motion-JPEG
				Size: Discrete 1920x1080
					Interval: Discrete 0.033s (30.000 fps)
				Size: Discrete 1280x720
					Interval: Discrete 0.017s (60.000 fps)
				Size: Discrete 1024x768
					Interval: Discrete 0.033s (30.000 fps)
				Size: Discrete 640x480
					Interval: Discrete 0.008s (120.101 fps)
				Size: Discrete 800x600
					Interval: Discrete 0.017s (60.000 fps)
				Size: Discrete 1280x1024
					Interval: Discrete 0.033s (30.000 fps)
				Size: Discrete 320x240
					Interval: Discrete 0.008s (120.101 fps)

			Index       : 1
			Type        : Video Capture
			Pixel Format: 'YUYV'
			Name        : YUYV 4:2:2
				Size: Discrete 1920x1080
					Interval: Discrete 0.167s (6.000 fps)
				Size: Discrete 1280x720
					Interval: Discrete 0.111s (9.000 fps)
				Size: Discrete 1024x768
					Interval: Discrete 0.167s (6.000 fps)
				Size: Discrete 640x480
					Interval: Discrete 0.033s (30.000 fps)
				Size: Discrete 800x600
					Interval: Discrete 0.050s (20.000 fps)
				Size: Discrete 1280x1024
					Interval: Discrete 0.167s (6.000 fps)
				Size: Discrete 320x240
					Interval: Discrete 0.033s (30.000 fps)

	4b. We will choose MJPG, 1280x720 for now to get 60fps for high framerate and resolution for this example
5. Run *v4l2-ctl -d 1 -v width=<w>,height=<h>,pixelformat=<pf>* to change the camera's settings
	5a. To choose choose MJPG, 1280x720, run *v4l2-ctl -d 1 -v width=1280,height=720,pixelformat=0* 
	5b. pixelformat is set to 0 since MJPG has its index equal to 0 in the result of 4a
6. Run *v4l2-ctl -d 1 -V* to confirm the change has been made


