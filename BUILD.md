*针对ubuntu20.04/22.04系统做了验证，gcc/g++版本默认分别为9和11，窗口环境为x11*

**基础环境配置**：

1、更新系统及安装必要工具

```bash
sudo apt update && sudo apt upgrade
sudo apt install cmake make gcc g++ git python2
```

2、安装编译依赖

```bash
sudo apt install -y libboost-all-dev libglew-dev libgtk2.0-dev \
libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev \
libtiff-dev libopenexr-dev libxi-dev libxrandr-dev libx11-dev \
libglu1-mesa-dev libgl1-mesa-dev libusb-1.0-0-dev libudev-dev doxygen \
doxygen-doc libxkbcommon-dev
```

3、安装eigen3

```bash
sudo apt install libeigen3-dev
```

4、安装opencv（我安装时默认版本是4.x）

```bash
sudo apt install libopencv-dev libopencv-highgui-dev libopencv-contrib-dev
```

5、安装Pangolin-0.6

复制到浏览器下载并解压缩：https://codeload.github.com/stevenlovegrove/Pangolin/zip/refs/tags/v0.6

```bash
cd Pangolin-0.6
mkdir build
cd build
cmake ..
make -j
sudo make install
```

测试是否安装成功

```bash
cd Pangolin
cd examples/HelloPangolin
mkdir build && cd build
cmake ..
make
./HelloPangolin
```

如果出现包含方块的窗口则安装成功

**ORBSLAM2基础运行：**

github下载：git clone https://github.com/raulmur/ORB_SLAM2.git

一些修改

```bash
a、ORB_SLAM2/CmakeLists.txt 和 ORB_SLAM2/Thirdparty/DBoW2/CmakeLists.txt 2处
# 修改opencv版本
-find_package(OpenCV 3.0 QUIET)
+find_package(OpenCV 4.2 QUIET)
b、include/LoopClosing.h
// 解决类别定义问题
-Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose; 
+Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose; 
c、include/ORBextractor.h
// 解决opencv头文件引用问题
-#include <opencv/cv.h>
+#include <opencv2/imgproc/imgproc_c.h>
+#include <opencv2/highgui/highgui_c.h>
d、include/System.h
+#include<opencv2/imgcodecs/legacy/constants_c.h> // 解决找不到CV_LOAD_IMAGE_UNCHANGED问题
+#include<unistd.h> // 解决usleep函数不存在问题
```

修改完进行编译

```bash
 cd ORB_SLAM2
./build.sh
```

**ORBSLAM2在ROS下运行:**

设置路径

```bash
sudo vim ~/.bashrc
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/ubuntu/ORB_SLAM2/Examples/ROS
source ~/.bashrc
```

重启终端，开始编译

```bash
./build_ros.sh
```

确认安装

```bash
roscd ORB_SLAM2 
```

