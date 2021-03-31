adduser --home /home/$1 --shell /bin/bash --gecos '' --disabled-password $1 && \
  usermod -a -G root,sudo,video $1 && \
  echo "$1 ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$1
