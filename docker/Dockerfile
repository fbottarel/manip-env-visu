ARG from=ubuntu:focal

FROM ${from}

# Dependencies for glvnd and X11.
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    glmark2 \
    && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Set up cpp dev environment
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    software-properties-common \
    git \
    nano \
    curl \
    cmake \
    cmake-curses-gui \
    wget \
    unzip \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Set up dependencies for VTK
RUN apt-get update \
    && apt-get install -y \
    libeigen3-dev \
    libgl1-mesa-dev \
    libxt-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up a workspace
WORKDIR workspace
ENV WS_DIR=/workspace
ENV WS_INSTALL=/workspace/install
ENV LD_LIBRARY_PATH=${WS_INSTALL}/lib:${LD_LIBRARY_PATH}
ENV PATH=${WS_INSTALL}/bin:${PATH}
RUN mkdir -p ${WS_DIR} ${WS_INSTALL}

# Build and install VTK
RUN git clone https://github.com/Kitware/VTK --branch v8.2.0 \
    && mkdir -p VTK/build \
    && cd VTK/build \
    && cmake -DBUILD_EXAMPLES=OFF \
             -DBUILD_SHARED_LIBS=ON \
             -DCMAKE_INSTALL_PREFIX=${WS_INSTALL} \
             .. \
    && make -j8 install \
    && cd ${WS_DIR} && rm -rf VTK

# Install additional dependencies
RUN apt-get update \
    && apt-get install -y \
    liburdfdom-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install the repo
RUN cd
RUN git clone https://github.com/fbottarel/manip-env-visu \
    && mkdir -p manip-env-visu/build \
    && cd manip-env-visu/build \
    && cmake -DVTK_DIR=${WS_INSTALL}/lib/cmake/vtk-8.2 \
             -DCMAKE_INSTALL_PREFIX=${WS_INSTALL} \
             .. \
    && make -j8 install

WORKDIR /
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
