# This Dockerfile and the accompanying shell script are used by the project
# maintainers to create the precompiled vtk binaries that are downloaded during
# the build. They are neither called during the build nor expected to be called
# by most developers or users of the project.

ARG CODENAME=bionic
FROM ubuntu:$CODENAME
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    ca-certificates \
    cmake \
    g++ \
    gcc \
    git \
    libexpat1-dev \
    libfreetype6-dev \
    libgl1-mesa-dev \
    libglib2.0-dev \
    libhdf5-dev \
    libjpeg8-dev \
    libjsoncpp-dev \
    liblz4-dev \
    libnetcdf-dev \
    libogg-dev \
    libpng-dev \
    libqt5x11extras5-dev \
    libtbb-dev \
    libtheora-dev \
    libtiff5-dev \
    libxml2-dev \
    libxt-dev \
    lsb-release \
    ninja-build \
    python-dev \
    python3-dev \
    qt5-default \
    qttools5-dev \
    wget \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*
RUN wget --quiet https://downloads.sourceforge.net/project/ispcmirror/v1.10.0/ispc-v1.10.0-linux.tar.gz \
  && tar -xzf ispc-*-linux.tar.gz \
  && cp -f ispc-*-Linux/bin/ispc /usr/local/bin \
  && rm -rf ispc-*-Linux ispc-*-linux.tar.gz
RUN git clone --branch v3.5.1 --config advice.detachedHead=false --depth 1 --quiet https://github.com/embree/embree.git \
  && mkdir -p embree-build /opt/vtk \
  && cd embree-build \
  && cmake \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DEMBREE_IGNORE_CMAKE_CXX_FLAGS:BOOL=OFF \
    -DEMBREE_MAX_ISA:STRING=SSE4.2 \
    -DEMBREE_STACK_PROTECTOR:BOOL=ON \
    -DEMBREE_TUTORIALS:BOOL=OFF \
    -GNinja \
    -Wno-dev \
    ../embree \
  && ninja install/strip \
  && cd .. \
  && rm -rf embree embree-build \
COPY ospray_no_fast_math.patch /ospray_no_fast_math.patch
RUN git clone --branch v1.8.2 --config advice.detachedHead=false --depth 1 --quiet https://github.com/ospray/ospray.git \
  && cd ospray \
  && git apply ../ospray_no_fast_math.patch \
  && cd .. \
  && rm -f ospray_no_fast_math.patch \
  && mkdir -p ospray-build \
  && cd ospray-build \
  && cmake \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_PREFIX_PATH:PATH=/opt/vtk \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DOSPRAY_ENABLE_APPS:BOOL=OFF \
    -DOSPRAY_ENABLE_TESTING:BOOL=OFF \
    -DOSPRAY_ENABLE_TUTORIALS:BOOL=OFF \
    -GNinja \
    -Wno-dev \
    ../ospray \
  && ninja install/strip \
  && cd .. \
  && rm -rf ospray ospray-build
COPY vtk_rendering_ospray.patch /vtk_rendering_ospray.patch
RUN git clone --branch v8.2.0 --config advice.detachedHead=false --depth 1 --quiet https://gitlab.kitware.com/vtk/vtk.git \
  && cd vtk \
  && git apply ../vtk_rendering_ospray.patch \
  && cd .. \
  && rm -f vtk_rendering_ospray.patch \
  && mkdir -p vtk-python3-build \
  && cd vtk-python3-build \
  && cmake \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DModule_vtkRenderingOSPRay:BOOL=ON \
    -DOSPRAY_INSTALL_DIR:PATH=/opt/vtk \
    -DPYTHON_EXECUTABLE:PATH=/usr/bin/python3 \
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
    -DVTK_USE_SYSTEM_OGG:BOOL=ON \
    -DVTK_USE_SYSTEM_PNG:BOOL=ON \
    -DVTK_USE_SYSTEM_THEORA:BOOL=ON \
    -DVTK_USE_SYSTEM_TIFF:BOOL=ON \
    -DVTK_USE_SYSTEM_ZLIB:BOOL=ON \
    -DVTK_WRAP_PYTHON:BOOL=ON \
    -GNinja \
    -Wno-dev \
    ../vtk \
  && ninja install/strip \
  && cd ..\
  && rm -rf vtk-python3-build \
  && mkdir -p vtk-python2-build \
  && vtk-python2-build \
  && cmake \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DModule_vtkRenderingOSPRay:BOOL=ON \
    -DOSPRAY_INSTALL_DIR:PATH=/opt/vtk \
    -DPYTHON_EXECUTABLE:PATH=/usr/bin/python2 \
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
    -DVTK_USE_SYSTEM_OGG:BOOL=ON \
    -DVTK_USE_SYSTEM_PNG:BOOL=ON \
    -DVTK_USE_SYSTEM_THEORA:BOOL=ON \
    -DVTK_USE_SYSTEM_TIFF:BOOL=ON \
    -DVTK_USE_SYSTEM_ZLIB:BOOL=ON \
    -DVTK_WRAP_PYTHON:BOOL=ON \
    -GNinja \
    -Wno-dev \
    ../vtk \
  && ninja install/strip \
  && cd .. \
  && rm -rf vtk vtk-python2-build \
  && sed -i 's|VTK_RUNTIME_DIRS "/opt/vtk/bin"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/bin"|g' /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed -i 's|VTK_RUNTIME_DIRS "/opt/vtk/lib"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/lib"|g' /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed -i 's|/opt/vtk|${_IMPORT_PREFIX}|g' /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed -i 's|${_IMPORT_PREFIX}/include/ospray;${_IMPORT_PREFIX}/include/ospray/SDK|${_IMPORT_PREFIX}/include/ospray|g' /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed -i 's|VTK_PYTHONPATH "/opt/vtk/lib/python2.7/site-packages"|VTK_PYTHONPATH "${VTK_INSTALL_PREFIX}/lib/python2.7/site-packages"|g' /opt/vtk/lib/cmake/vtk-*/Modules/vtkPython.cmake
RUN cd /opt/vtk \
  && tar -czf vtk-8.2.0-embree-3.5.1-ospray-1.8.2-python-$(python2 --version 2>&1 | sed -n -e 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-python-$(python3 --version | sed -n -e 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed -n -e 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-$(lsb_release --codename --short)-x86_64-4.tar.gz *
