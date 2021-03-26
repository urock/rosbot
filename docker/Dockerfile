FROM ros:melodic-perception

ENV ROSUSER="user"

RUN adduser --home /home/$ROSUSER --shell /bin/bash --gecos '' --disabled-password $ROSUSER \
    && usermod -a -G root,sudo $ROSUSER \
    && echo "$ROSUSER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$ROSUSER

# >>>> Install basic utils
RUN apt-get update \
    && apt-get install -y \
    python-catkin-tools \
    ros-melodic-ddynamic-reconfigure \
    ros-melodic-diagnostic-updater \
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

RUN mkdir /home/$ROSUSER/catkin_ws/ \
    && mkdir /home/$ROSUSER/catkin_ws/src \
    && cd /home/$ROSUSER/catkin_ws/src \
    && catkin_init_workspace

WORKDIR /home/$ROSUSER