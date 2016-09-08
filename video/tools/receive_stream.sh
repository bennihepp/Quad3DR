#!/bin/bash

export GST_DEBUG=3
# export GST_DEBUG_DUMP_DOT_DIR="."

# PNG
# decoder="pngdec !"
# rtp="!!!!!!!"
# JPEG
# decoder="jpegdec !"
# rtp="rtpjitterbuffer ! rtpjpegdepay !"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)JPEG, a-framerate=(string)30.000000, a-framesize=(string)2560-720, payload=(int)96"
# MPEG4
# decoder="avdec_mpeg4 !"
# rtp="rtpmp4vdepay !"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)MP4V-ES, profile-level-id=(string)1, config=(string)000001b001000001b58913000001000000012000c48d8800f550045a1463000001b24c61766335342e33352e30, payload=(int)96"
# VP8
# decoder="vp8dec !"
# rtp="rtpjitterbuffer latency=500 mode=0 ! rtpvp8depay !"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)VP8-DRAFT-IETF-01, payload=(int)96"
# OMX VP8
# decoder="omxvp8dec !"
# rtp="rtpjitterbuffer ! rtpvp8depay !"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)VP8-DRAFT-IETF-01, payload=(int)96"
# H264
# decoder="h264parse ! queue ! avdec_h264 !"
# rtp="rtpjitterbuffer ! rtph264depay !"
# # rtp="rtpjitterbuffer latency=200 mode=0 ! rtph264depay !"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96"
# H264
decoder="h264parse ! identity single-segment=true ! avdec_h264 !"
rtp="rtpjitterbuffer latency=500 mode=0 ! rtph264depay !"
caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96"
# # gdp=""
# # gdp="tsdemux"
# # gdp="gdpdepay ! application/x-rtp, payload=96 ! ${rtp}"
# # gdp="matroskademux !"
# gdp=""
# # caps="application/x-rtp, payload=96"
# libde265 H265
# decoder="h265parse ! queue ! libde265dec: !"
# rtp="rtpjitterbuffer ! rtph264depay !"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265, payload=(int)96"
# OMX H265
# decoder="h265parse ! queue ! omxh264dec !"
#rtp="rtpjitterbuffer ! rtph265depay !"
#caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265, payload=(int)96"
# Defunc
# H263
# decoder="avdec_h263 !"
# rtp="rtph263depay !"
# MPEG2
# decoder="mpeg2dec !"
# rtp="rtpmp2tdepay !"
#
# Decodebin
# decoder="decodebin !"
# # rtp="rtpjitterbuffer latency=1000 mode=0 ! rtpjpegdepay !"
# # rtp="rtpjitterbuffer latency=1000 mode=0 ! rtpvp8depay !"
# rtp="rtpjitterbuffer latency=200 mode=0 ! rtph264depay !"
# # caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96"
# # caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)MP4V-ES, profile-level-id=(string)1, config=(string)000001b001000001b58913000001000000012000c48d8800f550045a1463000001b24c61766335342e33352e30, payload=(int)96"
# caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)VP8-DRAFT-IETF-01, payload=(int)96"
# gdp="qtdemux ! h264parse !"
#
#decoder="videoconvert !"
#rtp="rtpvrawdepay !"
#caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:2, depth=(string)8, width=(string)1344, height=(string)376, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)3319574339, timestamp-offset=(uint)563672079, seqnum-offset=(uint)57031"

# hostname="localhost"
hostname="jetson1.inf.ethz.ch"
src="udpsrc port=5100 caps = \"${caps}\" ! ${rtp}"
# src="tcpclientsrc host=${hostname} port=5101 ${gdp} !"
# src="tcpserversrc host=0.0.0.0 port=5101 ! ${gdp}"
# src="filesrc location=test.bin !"

# sink="xvimagesink sync=false async=false -e"
#sink="autovideosink sync=0 -e"
# sink="fpsdisplaysink sync=0 -e"
sink="fpsdisplaysink name=fpssink text-overlay=false video-sink=xvimagesink signal-fps-measurements=true sync=false -e"
# sink="fakesink -e"
# sink="filesink location=test.bin"

gst-launch-1.0 -m -v ${src} tee name=tp \
    tp. ! queue2 ! ${decoder} videoconvert ! ${sink} \
    tp. ! queue2 ! h264parse ! mpegtsmux ! filesink location=test.mp4 buffer-mode=2 -e
# gst-launch-1.0 udpsrc port=12345 ! \
#     "application/x-rtp, payload=127" ! \
#     rtph264depay ! \
#     avdec_h264 ! \
#     videoconvert  ! \
#     xvimagesink sync=false
