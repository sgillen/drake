FROM ubuntu:16.04
ARG DEBIAN_FRONTEND=noninteractive
RUN set -eux \
  && apt-get update \
  && apt-get install --no-install-recommends --yes \
    ca-certificates \
    build-essential \
    cmake \
    git \
    libexpat1-dev \
    libfreetype6-dev \
    libgl1-mesa-dev \
    libhdf5-dev \
    libjpeg8-dev \
    libjsoncpp-dev \
    liblz4-dev \
    libnetcdf-cxx-legacy-dev \
    libnetcdf-dev \
    libogg-dev \
    libpng12-dev \
    libqt5x11extras5-dev \
    libtbb-dev \
    libtheora-dev \
    libtiff5-dev \
    libxml2-dev \
    libxt-dev \
    python-dev \
    qt5-default \
    qttools5-dev \
    wget \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*
RUN set -eux \
  && wget --quiet https://downloads.sourceforge.net/project/ispcmirror/v1.9.2/ispc-v1.9.2-linux.tar.gz \
  && tar -xzf ispc-v1.9.2-linux.tar.gz \
  && cp -f ispc-v1.9.2-linux/ispc /usr/local/bin \
  && rm -rf ispc-v1.9.2-linux ispc-v1.9.2-linux.tar.gz
RUN set -eux \
  && mkdir -p /opt/drake
RUN set -eux \
  && git clone --branch v3.2.0 --depth 1 https://github.com/embree/embree.git \
  && mkdir -p embree-build \
  && cd embree-build \
  && cmake \
    -DCMAKE_BUILD_TYPE:STRING=Debug \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/drake \
    -DEMBREE_MAX_ISA:STRING=SSE4.2 \
    -DEMBREE_TUTORIALS:BOOL=OFF \
    ../embree \
  && make -j16 install \
  && cd .. \
  && rm -rf embree embree-build
RUN set -eux \
  && git clone --branch v1.6.1 --depth 1 https://github.com/ospray/ospray.git \
  && mkdir -p ospray-build \
  && cd ospray-build \
  && cmake \
    -DCMAKE_BUILD_TYPE:STRING=Debug \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/drake \
    -Dembree_DIR:PATH=/opt/drake/lib/cmake/embree-3.2.0 \
    -DOSPRAY_ENABLE_APPS:BOOL=OFF \
    ../ospray \
  && make -j16 install \
  && cd .. \
  && rm -rf ospray ospray-build
RUN set -eux \
  && git clone --branch v8.1.1 --depth 1 https://gitlab.kitware.com/vtk/vtk.git
COPY vtkRenderingOSPRay.patch /vtk/vtkRenderingOSPRay.patch
RUN set -eux \
  && cd vtk \
  && git apply vtkRenderingOSPRay.patch \
  && rm -f vtkRenderingOSPRay.patch \
  && cd .. \
  && mkdir -p vtk-build \
  && cd vtk-build \
  && cmake \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_TYPE:STRING=Debug \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/drake \
    -DModule_vtkRenderingOSPRay:BOOL=ON \
    -DOSPRAY_INSTALL_DIR:PATH=/opt/drake \
    -DVTK_ENABLE_VTKPYTHON:BOOL=OFF \
    -DVTK_Group_Qt:BOOL=ON \
    -DVTK_LEGACY_REMOVE:BOOL=ON \
    -DVTK_QT_VERSION:STRING=5 \
    -DVTK_USE_SYSTEM_EXPAT:BOOL=ON \
    -DVTK_USE_SYSTEM_FREETYPE:BOOL=ON \
    -DVTK_USE_SYSTEM_HDF5:BOOL=ON \
    -DVTK_USE_SYSTEM_JPEG:BOOL=ON \
    -DVTK_USE_SYSTEM_JSONCPP:BOOL=ON \
    -DVTK_USE_SYSTEM_LIBXML2:BOOL=ON \
    -DVTK_USE_SYSTEM_LZ4:BOOL=ON \
    -DVTK_USE_SYSTEM_NETCDF:BOOL=ON \
    -DVTK_USE_SYSTEM_NETCDFCPP:BOOL=ON \
    -DVTK_USE_SYSTEM_OGGTHEORA:BOOL=ON \
    -DVTK_USE_SYSTEM_PNG:BOOL=ON \
    -DVTK_USE_SYSTEM_TIFF:BOOL=ON \
    -DVTK_USE_SYSTEM_ZLIB:BOOL=ON \
    -DVTK_WRAP_PYTHON:BOOL=ON \
    ../vtk \
  && make -j16 install \
  && cd .. \
  && rm -rf vtk vtk-build
RUN cd /opt/drake \
  && tar -czf vtk-v8.1.1-qt-5.5.1-xenial-x86_64-1.tar.gz *
