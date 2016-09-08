gst-launch-1.0 filesrc location=test.bin ! \
	h264parse ! omxh264dec ! \
	videoconvert ! \
	xvimagesink sync=false -e

