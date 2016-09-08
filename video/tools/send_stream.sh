#!/bin/bash

export GST_DEBUG=3
# export GST_DEBUG_DUMP_DOT_DIR="."
#gst-launch-1.0 v4l2src ! video/x-raw,format=YUY2,width=2560,height=720,framerate=60/1 ! videoconvert ! xvimagesink
#gst-launch-1.0 v4l2src ! video/x-raw,format=YV12,width=2560,height=720,framerate=60/1 ! xvimagesink
#gst-launch-1.0 v4l2src ! video/x-raw,width=2560,height=720,framerate=60/1 ! videoconvert ! vp9enc ! vp9dec ! videoconvert ! autovideosink
#gst-launch-1.0 v4l2src ! videoconvert ! vp9enc ! vp9dec ! videoconvert ! autovideosink
#gst-launch-1.0 v4l2src ! video/x-raw,format=YV12 ! autovideosink
#gst-launch-1.0 videotestsrc ! video/x-raw,width=320,height=240,framerate=30/1 ! vp8enc ! vp8dec ! fpsdisplaysink

frame_rate="15/2"
input_frame_rate="30/1"
# format=""
format="format=YUY2,"
pixel_aspect_ratio=""
# pixel_aspect_ratio="pixel-aspect-ratio=1/1,"
interlace_mode=""
# interlace_mode="interlace-mode=progressive"
video_format="video/x-raw,${format}${pixel_aspect_ratio}${interlace_mode}width=1344,height=376,framerate=${input_frame_rate}"
# video_format="video/x-raw,${format}${pixel_aspect_ratio}${interlace_mode}width=2560,height=720,framerate=${input_frame_rate}"

# video_source="videotestsrc ! ${video_format} !"
#video_source="filesrc location=\"UWBTracking_IROS_2016.mp4\" ! decodebin !"
video_source="v4l2src ! ${video_format} !"

# video_rate=""
video_rate="videorate drop-only=true max-rate=30 ! video/x-raw,framerate=${frame_rate} !"

video_scale=""
# video_scale="videoscale method=1 add-borders=false ! video/x-raw,width=2560,height=720,pixel-aspect-ratio=1/1 !"


# PNG
# encoder="pngenc !"
# rtp="!!!!!!!"
# JPEG
# encoder="video/x-raw,format=I420,width=1344,height=376,framerate=30/1 ! jpegenc !"
# rtp ="rtpjpegpay !"
# Nvidia JPEG
# encoder="video/x-raw,format=I420 ! nvjpegenc !"
# rtp ="rtpjpegpay !"
# MPEG4
# encoder="avenc_mpeg4 max-key-interval=10 bitrate=5000000 quantizer=0.01 !"
# rtp="rtpmp4vpay !"
# VP8
# encoder="vp8enc threads=4 cq-level=10 target-bitrate=1000000 ! keyframe-max-dist=10 end-usage=cbr"
# encoder="vp8enc threads=4 cq-level=10 target-bitrate=1000000 !"
# rtp="rtpvp8pay !"
# H264
# encoder="x264enc bitrate=1024 pass=qual quantizer=20 tune=zerolatency speed-preset=medium threads=4 key-int-max=100 !"
# rtp="rtph264pay !"
# OMX H264
# encoder="omxh264enc low-latency=1 bitrate=1024 iframeinterval=15 !"
# encoder="omxh264enc low-latency=0 control-rate=1 bitrate=10000000 ! "
encoder="omxh264enc low-latency=0 control-rate=1 bitrate=1000000 ! "
rtp="rtph264pay mtu=1400 config-interval=5 !"
# gdp="h264parse ! qtmux !"
gdp="video/x-h264,stream-format=byte-stream !"
# gdp="mpegtsmux"
# gdp="avmux_mpegts"
# gdp="${rtp} gdppay"
# gdp="matroskamux !"
# gdp=""
# OMX H265
# encoder="omxh265enc low-latency=1 control-rate=2 bitrate=4000000 iframeinterval=30 ! "
# rtp="h256parse ! rtph265pay !"
# OMX VP8
# encoder="omxvp8enc low-latency=false control-rate=1 temporal-tradeoff=2 bitrate=3000000 !"
# rtp="rtpvp8pay !"
# Defunc
# encoder="avenc_h263 !"
# rtp="rtph263pay !"
# MPEG2
# encoder="avenc_mpeg2video !"
# rtp="rtpmp2tpay !"
#
#encoder="videoconvert !"
#rtp="rtpvrawpay !"


# hostname="localhost"
hostname="ohws05.inf.ethz.ch"
sink="${rtp} udpsink host=${hostname} port=5100 sync=false async=false"
# sink="video/x-h264, stream-format=(string)byte-stream ! ${rtp} udpsink host=${hostname} port=5100 sync=false async=false"
# sink="video/x-h264, stream-format=(string)byte-stream ! ${rtp} tcpserversink host=0.0.0.0 port=5101"
# sink="${gdp} tcpserversink host=0.0.0.0 port=5101"
# sink="${gdp} tcpclientsink host=${hostname} port=5101"
# sink="video/x-h264, stream-format=(string)byte-stream ! filesink location=test.bin -e"

# display_sink="xvimagesink sync=0"
# display_sink="autovideosink sync=0"
#display_sink="fpsdisplaysink sync=0"
# display_sink="fpsdisplaysink name=fpssink text-overlay=false video-sink=xvimagesink signal-fps-measurements=true sync=false"
display_sink="fakesink"

gst-launch-1.0 -v ${video_source} videoconvert ! \
    ${video_rate} ${video_scale} \
    tee name=tp \
    tp. ! queue2 ! ${display_sink} \
    tp. ! queue2 !  ${encoder} ${sink}

    # tp. ! queue2 ! avenc_mpeg4 ! rtpmp4vpay ! udpsink host=127.0.0.1 port=5100 \
    # tp. ! queue2 ! vp8enc keyframe-max-dist=15 threads=4 cq-level=1 ! rtpvp8pay ! udpsink host=127.0.0.1 port=5100 \
# gst-launch-1.0 -v videotestsrc ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! tee name=tp \
#     tp. ! queue2 ! vp8enc keyframe-max-dist=15 threads=4 cq-level=10 ! rtpvp8pay ! udpsink host=127.0.0.1 port=5100 \
#     tp. ! queue2 ! autovideosink sync=false
# gst-launch-1.0 -v v4l2src ! video/x-raw,format=YUY2,width=1344,height=376,framerate=15/1 ! videoconvert ! videoscale ! video/x-raw,width=800 ! tee name=tp \
    # tp. ! queue ! avenc_mpeg4 ! rtpmp4vpay config-interval=3 ! udpsink host=127.0.0.1 port=5100 \
    # tp. ! queue ! autovideosink

#gst-launch-1.0 v4l2src ! xvimagesink
#gst-launch-1.0 v4l2src ! video/x-raw,format=YUY2,width=4416,height=1242,framerate=15/1,pixel-aspect-ratio=1/1,interlace-mode=progressive ! xvimagesink
#gst-launch-1.0 v4l2src ! video/x-raw,format=YUY2,width=4416,height=1242,framerate=15/1,interlace-mode=progressive ! xvimagesink
#gst-launch-1.0 v4l2src ! video/x-raw,format=YUY2,width=4416,height=1242,framerate=15 ! xvimagesink
