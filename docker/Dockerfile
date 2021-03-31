FROM osrf/ros:melodic-desktop-full

ENV ROSUSER="user"

RUN mkdir -p /scripts
WORKDIR /scripts

# Add user
COPY scripts/add-user.sh .
  RUN chmod +x add-user.sh 
  RUN ./add-user.sh ${ROSUSER}

# Add repositories
COPY scripts/add-repositories.sh .
  RUN chmod +x add-repositories.sh 
  RUN ./add-repositories.sh

# Bash completion
COPY scripts/bash-completion.sh .
  RUN chmod +x bash-completion.sh 
  RUN ./bash-completion.sh

# Utils Installation
COPY scripts/utils-install.sh .
  RUN chmod +x utils-install.sh 
  RUN ./utils-install.sh

# Python 
COPY scripts/python-pip.sh .
  RUN chmod +x python-pip.sh 
  RUN ./python-pip.sh

COPY scripts/python2-pkgs.sh .
  RUN chmod +x python2-pkgs.sh 
  RUN ./python2-pkgs.sh

COPY scripts/python3-pkgs.sh .
  RUN chmod +x python3-pkgs.sh
  RUN ./python3-pkgs.sh

COPY scripts/ros-pkgs.sh .
  RUN chmod +x ros-pkgs.sh 
  RUN ./ros-pkgs.sh

user $ROSUSER

# ROS
COPY scripts/ros-setup.sh .
  RUN sudo chmod +x ros-setup.sh 
  RUN ./ros-setup.sh

# Utils setup
COPY scripts/utils-setup.sh .
  RUN sudo chmod +x utils-setup.sh 
  RUN ./utils-setup.sh

WORKDIR /home/$ROSUSER
