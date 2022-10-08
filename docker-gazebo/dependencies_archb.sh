curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
sudo pacman-key --add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo pacman -Suuy

sudo pamac install nvidia-docker
sudo pamac install nvidia-container-toolkit

sudo systemctl restart docker
