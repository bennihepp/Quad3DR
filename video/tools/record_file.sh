gst-launch-1.0 videotestsrc num-buffers=1500 ! \
	video/x-raw, format=(string)I420,width=(int)640, height=(int)480 ! \
	omxh264enc ! video/x-h264, stream-format=(string)byte-stream ! \
	filesink location=test.bin -e

