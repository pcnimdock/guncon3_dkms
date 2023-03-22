#!/bin/bash
res=$(xdpyinfo | awk '/dimensions/ {print $2}')
python calibrate.py -r $res
