#!/bin/sh

find ./broadcast_4_way_crossing -type f -name snap\* -delete
find ./broadcast_blocking -type f -name snap\* -delete
find ./broadcast_head_on -type f -name snap\* -delete
find ./broadcast_moshpit -type f -name snap\* -delete

find ./honest_4_way_crossing -type f -name snap\* -delete
find ./honest_head_on -type f -name snap\* -delete
find ./honest_moshpit -type f -name snap\* -delete

find ./inference_4_way_crossing -type f -name snap\* -delete
find ./inference_blocking -type f -name snap\* -delete
find ./inference_head_on -type f -name snap\* -delete
find ./inference_moshpit -type f -name snap\* -delete