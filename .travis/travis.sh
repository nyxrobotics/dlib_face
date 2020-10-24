#!/bin/bash
docker run -itd --name build_container -v `pwd`:/opt/workspace/src/dlib_face -w /opt/workspace/src/dlib_face tacha/dlib_face:${ROS_DISTRO}

docker exec -it build_container .travis/clang-format.sh

docker exec -it build_container .travis/build.sh

docker exec -it build_container .travis/clang-tidy.sh
