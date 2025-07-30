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
RUN apt-get install -y python3-pip python3.12-venv clang-format
ENV SHELL /bin/bash

# == phidgets == #
RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash -
RUN apt install -y libphidget22

# == PYTHON == #
RUN python3 -m venv /opt/venv
RUN /opt/venv/bin/pip install numpy pyserial pynmea2 lark catkin_pkg colcon-common-extensions Phidget22
ENV PATH="/opt/venv/bin:$PATH"

# == source == #
# Append sourcing logic to .bashrc
RUN echo '\
    # Source ROS 2\n\
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then\n\
    source /opt/ros/jazzy/setup.bash\n\
    fi\n\
    \n\
    # Source your virtual environment\n\
    if [ -f "/home/sir/venv/bin/activate" ]; then\n\
    source /home/sir/venv/bin/activate\n\
    fi\n\
    \n\
    # Source your built ROS workspace (if exists)\n\
    if [ -f "/home/ws/install/setup.bash" ]; then\n\
    source /home/ws/install/setup.bash\n\
    fi\n\
    ' >> /home/sir/.bashrc

USER $USERNAME
CMD ["/bin/bash"]