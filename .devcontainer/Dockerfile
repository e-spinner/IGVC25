FROM ros:jazzy
ARG USERNAME=sir
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip python3.12-venv clang-format ros-jazzy-rqt-graph curl zsh

# == phidgets == #
RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash -
RUN apt install -y libphidget22

# == PYTHON == #`
RUN python3 -m venv /opt/venv
RUN /opt/venv/bin/pip install numpy pyserial pynmea2 lark catkin_pkg colcon-common-extensions Phidget22 flask flask-socketio
ENV PATH="/opt/venv/bin:$PATH"

USER $USERNAME
ENV HOME /home/$USERNAME
WORKDIR /home/$USERNAME

# == ZSH == #
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git $HOME/.oh-my-zsh/custom/themes/powerlevel10k
COPY zshrc /home/$USERNAME/.zshrc
COPY p10k.zsh /home/$USERNAME/.p10k.zsh
ENV SHELL /bin/zsh

CMD ["/bin/zsh"]