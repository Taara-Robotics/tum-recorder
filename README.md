## Linux

Tested on _Ubuntu 22.04_.

### Prerequisites

```sh
sudo apt update -y
sudo apt upgrade -y --no-install-recommends
# basic dependencies
sudo apt install -y build-essential pkg-config cmake git wget curl unzip
# OpenCV dependencies
sudo apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavutil-dev libswscale-dev libtbb-dev
```

### Build and install dependencies

Choose a directory to download and build the dependencies, eg `~/lib`. Run the provided commands in the chosen directory.

```sh
mkdir -p ~/lib
cd ~/lib
```

In addition, update your `~/.bashrc` file to include `/usr/local/lib` in the library path. Add the following line to the end of the file:

```sh
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"
```

#### Build and install OpenCV

```sh
cd ~/lib
# Download OpenCV
wget -q https://github.com/opencv/opencv/archive/4.5.5.zip
unzip -q 4.5.5.zip && rm -rf 4.5.5.zip
# Download aruco module
wget -q https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip -O opencv_contrib-4.5.5.zip
unzip -q opencv_contrib-4.5.5.zip && rm -rf opencv_contrib-4.5.5.zip
# Build and install OpenCV
cd opencv-4.5.5
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PROTOBUF=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_ml=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DENABLE_CXX11=ON \
    -DENABLE_FAST_MATH=ON \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_TBB=ON \
    -DWITH_OPENMP=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.5/modules \
    ..
make -j4
sudo make install
```

#### Build and install Orbbec SDK

```sh
cd ~/lib
git clone https://github.com/orbbec/OrbbecSDK.git
cd OrbbecSDK
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

Install the udev rules file:

```sh
cd ~/lib/OrbbecSDK/misc/scripts
sudo chmod +x ./install_udev_rules.sh
sudo ./install_udev_rules.sh
sudo udevadm control --reload && sudo udevadm trigger
```

You can run `~/lib/OrbbecSDK/build/bin/OBMultiStream` to verify the installation.

### Build project

```sh
cd ~/lib
git clone --recurse-submodules https://github.com/Taara-Robotics/tum-recorder.git
cd tum-recorder
mkdir build && cd build
# provide the path to the Orbbec SDK directory if not installed system-wide
cmake .. -DOrbbecSDK_DIR=~/lib/OrbbecSDK
make
```

### Run recorder

```sh
# in the build directory
./record_orbbec -o ../recording1
```
