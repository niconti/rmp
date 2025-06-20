FROM ros:noetic-robot

RUN apt-get update \
 && apt-get install -y x11-apps mesa-utils \
 && apt-get install -y python3-catkin-tools git


ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 #
 # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
 && apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# Setup workspace
ENV WORKSPACE_ROOT=/home/${USERNAME}/workspace/
RUN mkdir -p $WORKSPACE_ROOT
WORKDIR ${WORKSPACE_ROOT}

# Setup environment
ENV DISABLE_ROS1_EOL_WARNINGS=1