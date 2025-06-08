FROM humble:dev

ARG VERSION=0.0.1


RUN echo "image dev version: ${VERSION}">/etc/docker_image_dev_version

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-key del F42ED6FBAB17C654
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


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
      ros-humble-cv-bridge \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

#
# install gstreamer
#
RUN apt-get update \
&& apt-get -y install --no-install-recommends \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    python3-gi \
    gir1.2-gstreamer-1.0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-libav \
    gstreamer-1.0 \
&& apt-get clean -y \
&& rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

USER user

RUN pip install klvdata