FROM pov:dev

ARG VERSION=0.0.1


RUN echo "image dev version: ${VERSION}">/etc/docker_image_dev_version

ENV DEBIAN_FRONTEND=noninteractive

# dev
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      python-is-python3 \
      rviz \
      ros-humble-rqt-image-view \
      ros-humble-rqt \
      python3-colcon-clean \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# runtime
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      tmux \
      ros-humble-gscam \
      ros-humble-diagnostic-updater \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*


ENV DEBIAN_FRONTEND=dialog

USER user