FROM ros:melodic-desktop-full

ENV ROSUSER="user"

RUN adduser --home /home/$ROSUSER --shell /bin/bash --gecos '' --disabled-password $ROSUSER \
    && usermod -a -G root,sudo $ROSUSER \
    && echo "$ROSUSER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$ROSUSER

# >>>> Install basic utils
RUN apt-get update \
    && apt-get install -y \
    python-catkin-tools \
    python-pip \
    tmux

RUN apt-get install -y \
    bash-completion \
    && sed -i 's/--no-generate //' /usr/share/bash-completion/completions/apt-get \
    && sed -i 's/--no-generate //' /usr/share/bash-completion/completions/apt-cache

user $ROSUSER

RUN cd \
    && git clone https://github.com/gpakosz/.tmux.git \
    && ln -s -f .tmux/.tmux.conf \
    && cp .tmux/.tmux.conf.local .   

RUN pip install pathlib
RUN pip install typing


RUN mkdir /home/$ROSUSER/catkin_ws/ \
    && mkdir /home/$ROSUSER/catkin_ws/src \
    && echo "source /home/${ROSUSER}/catkin_ws/devel/setup.bash" >> ~/.bashrc 

WORKDIR /home/$ROSUSER