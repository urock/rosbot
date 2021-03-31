apt-get install -y \
  bash-completion \
  && sed -i 's/--no-generate //' /usr/share/bash-completion/completions/apt-get \
  && sed -i 's/--no-generate //' /usr/share/bash-completion/completions/apt-cache
