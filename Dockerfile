FROM larshaalck/ceres:latest

FROM alpine:edge

RUN apk add --update --no-cache \
    build-base \
    linux-headers \
    eigen-dev \
    cmake \
    glog-dev \
    suitesparse-dev \
    openblas-dev \
    libgomp \
    graphviz \
    libpng-dev \
    libjpeg-turbo-dev \
    tiff-dev \
    libxi-dev \
    libxrandr-dev \
    libxxf86vm-dev \
    --repository http://dl-cdn.alpinelinux.org/alpine/edge/testing/

COPY --from=0 /usr/local/include/ceres /usr/local/include/ceres
COPY --from=0 /usr/local/lib64 /usr/local/lib64

ADD . /opt/openMVG

# Build
RUN mkdir /opt/openMVG/build && cd /opt/openMVG/build && \
    cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX="/usr/local" \
    -D EIGEN_INCLUDE_DIR_HINTS="/usr/include/eigen3" \
    -D OpenMVG_BUILD_TESTS=OFF \
    -D OpenMVG_BUILD_DOC=OFF \
    -D OpenMVG_BUILD_EXAMPLES=OFF \
    -D OpenMVG_BUILD_GUI_SOFTWARES=OFF \
    ../src/ && \
    make -j12 && \
    make install && \
    rm -rf /opt/openMVG

RUN apk del \
    build-base \
    linux-headers \
    eigen-dev \
    cmake \
    glog-dev \
    suitesparse-dev \
    openblas-dev \
    libpng-dev \
    libjpeg-turbo-dev \
    tiff-dev \
    libxi-dev \
    libxrandr-dev \
    libxxf86vm-dev

RUN apk add --no-cache \
    glog \
    suitesparse \
    openblas \
    libpng \
    libjpeg-turbo \
    tiff \
    libxi \
    libxrandr \
    libxxf86vm \
    --repository http://dl-cdn.alpinelinux.org/alpine/edge/testing/
