#!/bin/bash
ffmpeg -r 96 -i $1/%07d.png -pix_fmt yuv420p $2
