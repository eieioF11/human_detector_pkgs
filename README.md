# human_detector_pkgs
Human Detection Package. \
Tested on Ubuntu 22.04,ros2 humble.

# Installation
## ros2_common_tools Installation
```
cd ros2_ws/src
git clone https://github.com/eieioF11/ros2_common_tools.git
cd ros2_common_tools
git submodule update --init --recursive
```

## Clone human_detector_pkgs
```bash
cd ros2_ws/src
git clone --recursive https://github.com/eieioF11/human_detector_pkgs.git
```

## Python dependency installation
```bash
cd human_detector_pkgs
pip3 install -r requirements.txt
```

## TensorRT Installation
- https://developer.nvidia.com/tensorrt \
Tested on tensorrt 8.6.2

## Download models
```bash
cd human_detector_pkgs/mono_depth
sh download_models.sh
```

### Model conversion
※Do this after building the package with colcon build.
```bash
cd human_detector_pkgs/mono_depth/mono_depth
chmod 777 export.bash
source export.bash
```
