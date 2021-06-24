FROM osrf/ros:melodic-desktop-full

##################################################################
# ------------------------------ CUDA -------------------------- #
##################################################################

#############
# CUDA Base #
#############
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 11.2.2

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-11-2=11.2.152-1 \
    cuda-compat-11-2 \
    && ln -s cuda-11.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.2 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 driver>=450,driver<451"

################
# CUDA Runtime #
################
ENV NCCL_VERSION 2.8.4

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-11-2=11.2.2-1 \
    libnpp-11-2=11.3.2.152-1 \
    cuda-nvtx-11-2=11.2.152-1 \
    libcublas-11-2=11.4.1.1043-1 \
    libcusparse-11-2=11.4.1.1152-1 \
    libnccl2=$NCCL_VERSION-1+cuda11.2 \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold libcublas-11-2 libnccl2


##########
# CUDDNN #
##########
ENV CUDNN_VERSION 8.1.1.33

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=$CUDNN_VERSION-1+cuda11.2 \
    && apt-mark hold libcudnn8 && \
    rm -rf /var/lib/apt/lists/*

##################################################################
# ------------------------------ END -------------------------- #
##################################################################

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Python
RUN apt-get update && \
    apt-get install -y \
       python-pip \
       python-tk \
       python3-tk \
       python3-pip && \
       rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install \
       torch 

RUN pip3 install --upgrade pip && \
    pip3 install \
        rospkg \
        numpy \
        pandas \
        matplotlib \
        wandb \
        seaborn \
        tqdm \
        yaml-1.3 \
        nnio \
        onnxruntime-gpu 

RUN pip3 install --upgrade pip && \
    pip3 install \
	netifaces

RUN pip2 install --upgrade pip && \
    pip2 install \
        typing \
        pathlib \
        PyYAML

# ROS
RUN apt-get update && \
    apt-get install -y \
        python-catkin-tools \
        ros-melodic-ddynamic-reconfigure \
        ros-melodic-move-base \
        ros-melodic-map-server \
        ros-melodic-gmapping \
        ros-melodic-teleop-twist-keyboard \
        ros-melodic-amcl && \
        rm -rf /var/lib/apt/lists/*


# Utils 
RUN apt-get update && \
    apt-get install -y \
        tmux \
        ranger \
        wget \
        neovim && \
        rm -rf /var/lib/apt/lists/*


# Create User
ENV ROSUSER="user"

RUN adduser --home /home/${ROSUSER} --shell /bin/zsh --gecos '' --disabled-password ${ROSUSER} && \
    usermod -a -G root,sudo,video ${ROSUSER} && \
    echo "${ROSUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${ROSUSER}
user $ROSUSER

# Scripts
RUN mkdir -p /home/$ROSUSER/scripts
WORKDIR /home/$ROSUSER/scripts

COPY scripts/utils-setup.sh .
RUN ./utils-setup.sh

COPY scripts/ros-setup.sh .
RUN ./ros-setup.sh

WORKDIR /home/$ROSUSER
RUN rm -rf scripts
