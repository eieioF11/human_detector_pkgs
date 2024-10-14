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
git clone https://github.com/eieioF11/human_detector_pkgs.git
cd human_detector_pkgs
git submodule update --init --recursive
```
## Python dependency installation
```bash
cd human_detector_pkgs
pip3 install -r requirements.txt
```
## Download models
```bash
cd human_detector_pkgs/mono_depth
sh download_models.sh
```
