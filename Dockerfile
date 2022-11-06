FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# Install build tools
RUN apt update && apt install -y --no-install-recommends \
  python3-dev python3-pip python3-numpy \
  g++ cmake ninja-build pkg-config wget unzip git

# Install TensorRT
RUN apt install -y --no-install-recommends \
  libnvinfer-bin libnvinfer-dev libnvinfer-plugin-dev libnvinfer-plugin8 \
  libnvinfer8 libnvonnxparsers-dev libnvonnxparsers8 \
  libnvparsers-dev libnvparsers8 python3-libnvinfer python3-libnvinfer-dev

# Build OpenCV
RUN apt install -y --no-install-recommends \
  libpng-dev libjpeg-dev libtiff-dev libwebp-dev libopenjp2-7-dev libopenexr-dev \
  libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libswresample-dev \
  libtbb-dev \
  libgtk2.0-dev libgtkglext1-dev libglx-dev libcanberra-gtk-module

ARG OPENCV_VERSION="4.6.0"
ARG CUDA_ARCH_BIN

RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.zip && \
  wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.zip && \
  unzip opencv.zip && unzip opencv_contrib.zip

RUN cd opencv-${OPENCV_VERSION} && mkdir build && cd build && \
  cmake -GNinja .. \
  -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${OPENCV_VERSION}/modules/cudaimgproc\;../../opencv_contrib-${OPENCV_VERSION}/modules/cudev \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_opencv_apps=OFF \
  -D ENABLE_FAST_MATH=ON \
  -D CUDA_FAST_MATH=ON \
  -D WITH_CUDA=ON \
  -D WITH_CUBLAS=ON \
  -D WITH_CUDNN=ON \
  -D OPENCV_DNN_CUDA=ON \
  -D CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
  -D WITH_V4L=ON \
  -D WITH_FFMPEG=ON \
  -D WITH_MFX=ON \
  -D WITH_TBB=ON \
  -D WITH_OPENMP=ON \
  -D WITH_GTK=ON \
  -D WITH_GTK_2_X=ON \
  -D WITH_QT=OFF \
  -D WITH_OPENGL=ON \
  -D OpenGL_GL_PREFERENCE=GLVND && \
  ninja && ninja install

# Build Hyperpose
RUN apt -y --no-install-recommends install libgflags-dev

RUN wget -O hyperpose.zip https://github.com/tensorlayer/hyperpose/archive/refs/heads/master.zip && \
  unzip hyperpose.zip && mv hyperpose-master hyperpose

RUN cd hyperpose && \
  sed -i "/TARGET_LINK_LIBRARIES(\${target} stdtracer)/d" cmake/3rdparty.cmake && \
  mkdir build && cd build && \
  cmake -GNinja .. \
  -D CMAKE_BUILD_TYPE=Release \
  -D WITH_TRACE=OFF \
  -D BUILD_CLI=ON \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_USER_CODES=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_LIB=ON \
  -D BUILD_PACKAGE=ON &&\
  ninja && ninja install

# Install libtorch
RUN wget -O libtorch.zip https://download.pytorch.org/libtorch/cu117/libtorch-cxx11-abi-shared-with-deps-1.13.0%2Bcu117.zip && \
  unzip libtorch.zip
