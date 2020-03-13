#!/bin/sh

ffmpeg -r 20 -f image2 -i snap%d0.png -vb 20M -s 1000x1000 -y simulation.avi
