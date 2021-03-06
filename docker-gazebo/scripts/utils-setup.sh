# TMUX
cd && \
git clone https://github.com/gpakosz/.tmux.git  && \
ln -s -f .tmux/.tmux.conf && \
cp .tmux/.tmux.conf.local .

# ZSH
sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.1/zsh-in-docker.sh)" -- \
	-p git \
	-p https://github.com/zsh-users/zsh-autosuggestions \
	-p https://github.com/zsh-users/zsh-completions \
	-p https://github.com/zsh-users/zsh-syntax-highlighting 
