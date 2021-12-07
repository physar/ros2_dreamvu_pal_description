#!/bin/bash

cd ../..

colcon build --packages-select dreamvu_pal_camera_description --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
