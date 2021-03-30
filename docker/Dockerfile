FROM ros:melodic-perception

ENV ROSUSER="user"

RUN adduser --home /home/$ROSUSER --shell /bin/bash --gecos '' --disabled-password $ROSUSER \
    && usermod -a -G root,sudo,tty,dialout $ROSUSER \
    && echo "$ROSUSER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$ROSUSER

RUN apt-get update && \
    apt-get install -y \
    software-properties-common && \
    apt-add-repository universe  

RUN apt-get install -y \
    bash-completion \
    && sed -i 's/--no-generate //' /usr/share/bash-completion/completions/apt-get \
    && sed -i 's/--no-generate //' /usr/share/bash-completion/completions/apt-cache

RUN apt-get install -y \
    python-catkin-tools \
    ros-melodic-ddynamic-reconfigure 

RUN apt-get install -y \
    tmux \
    ranger \
    neovim \
    python-pip \
    python3-pip 

RUN pip3 install --upgrade pip \
    && pip3 install \
        onnxruntime \
        nnio 

RUN pip2 install --upgrade pip \
  && pip2 install  \
      typing \
      pathlib

user $ROSUSER

RUN cd \
    && git clone https://github.com/gpakosz/.tmux.git \
    && ln -s -f .tmux/.tmux.conf \
    && cp .tmux/.tmux.conf.local .   

RUN mkdir /home/$ROSUSER/catkin_ws/  && \ 
    mkdir /home/$ROSUSER/catkin_ws/src && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /home/${ROSUSER}/catkin_ws/devel/setup.bash" >> ~/.bashrc 

WORKDIR /home/$ROSUSER
