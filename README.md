# dlib_face 

[![Build Status](https://travis-ci.com/Tacha-S/dlib_face.svg?branch=main)](https://travis-ci.com/Tacha-S/dlib_face)

ROS package for face recognition using [dlib](https://github.com/davisking/dlib).

## Installation

### Install dlib

Dlib can be GPU accelerated.
If you have a GPU and CUDA and cuDNN are not installed, please install them first.

``` bash
git clone git@github.com:davisking/dlib.git
cd dlib && mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON ..
cmake --build . --config Release
sudo checkinstall # or make install
```

The dlib pkgconfig file is installed in `/usr/local/lib/pkgconfig`, so if the path is not specified in `PKG_CONFIG_PATH`, execute the following command.

``` bash
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig
```

### Install dlib_face

``` bash
cd /path/to/your/workspace/src
git clone git@github.com:Tacha-S/dlib_face.git
rosdep install -y --from-paths dlib_face -i --rosdistro melodic
cd dlib_face
catkin bt
```

### Download pre-trained model

A network model is required to use dlib's face detection and face recognition, so please clone it [here](https://github.com/davisking/dlib-models).

The models used in this package are:
- shape_predictor_5_face_landmarks.dat
- dlib_face_recognition_resnet_model_v1.dat

Please give the correct path to these files as rguments when `roslaunch`.
It is recommended to place it in the default path `~/.ros/dlib/models`.

## Benchmark

Simulation on CPU:i7-3770K, RAM 16GB, GPU:GTX1660-Ti(VRAM 6GB)

Image topic: 640x480, 30Hz

| \ | CPU usage | MEM usage | GPU usage | VRAM usage |
| ------------- | ------------- | ------------- | ------------- | ------------- |
| Detection only | about 40% | 9.1% | about 20% | 602MiB |
| Detection & Recognition | about 50% | 10.0% | about 20% | 785MiB |

## Contribute

Contributions are always welcome!

## License

MIT
