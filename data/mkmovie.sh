#!/bin/sh

ffmpeg -r 20 -f image2 -i broadcast_4_way_crossing/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/broadcast_4_way_crossing.avi
ffmpeg -r 20 -f image2 -i broadcast_blocking/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/broadcast_blocking.avi
ffmpeg -r 20 -f image2 -i broadcast_head_on/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/broadcast_head_on.avi
ffmpeg -r 20 -f image2 -i broadcast_moshpit/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/broadcast_moshpit.avi

ffmpeg -r 20 -f image2 -i honest_4_way_crossing/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/honest_4_way_crossing.avi
ffmpeg -r 20 -f image2 -i honest_head_on/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/honest_head_on.avi
ffmpeg -r 20 -f image2 -i honest_moshpit/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/honest_moshpit.avi

ffmpeg -r 20 -f image2 -i inference_4_way_crossing/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/inference_4_way_crossing.avi
ffmpeg -r 20 -f image2 -i inference_blocking/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/inference_blocking.avi
ffmpeg -r 20 -f image2 -i inference_head_on/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/inference_head_on.avi
ffmpeg -r 20 -f image2 -i inference_moshpit/snap%d.png -vb 20M -s 1000x1000 -y /mnt/c/Users/wilko/Desktop/Robotics\ Miniproject/vids/inference_moshpit.avi
