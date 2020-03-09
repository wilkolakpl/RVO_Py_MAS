#!/bin/sh

ffmpeg -r 7 -f image2 -i snap%d.0.png -vb 20M -s 1000x1000 -y simulation.avi
