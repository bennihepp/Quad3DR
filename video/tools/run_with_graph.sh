#!/bin/bash

export GST_DEBUG=5
export GST_DEBUG_DUMP_DOT_DIR="."
./video_streamer $@

