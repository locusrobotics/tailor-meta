FROM ubuntu:noble

LABEL tailor="environment"

SHELL ["/bin/bash", "-c"]

ARG AWS_ACCESS_KEY_ID
ENV AWS_ACCESS_KEY_ID ${AWS_ACCESS_KEY_ID}

ARG AWS_SECRET_ACCESS_KEY
ENV AWS_SECRET_ACCESS_KEY ${AWS_SECRET_ACCESS_KEY}

ENV DEBIAN_FRONTEND noninteractive
ENV PYTHONUNBUFFERED 1
ARG APT_REFRESH_KEY

RUN sed -i 's/archive.ubuntu.com/us-east-1.ec2.&/g' /etc/apt/sources.list
RUN echo "APT refresh key=${APT_REFRESH_KEY}"
RUN apt-get update && apt-get install --no-install-recommends -y locales curl gnupg1 gpgv1 sudo
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install --no-install-recommends -y \
  python3-dev \
  python3-pip \
  python3-setuptools \
  python3-wheel \
  build-essential \
  git \
  python3-apt \
  fakeroot \
  dpkg-dev \
  debhelper \
  devscripts \
  python3-colcon-common-extensions


# Inject the rosdep definitions from source
COPY rosdistro/rosdep/rosdep.yaml /etc/ros/rosdep/rosdep.yaml

COPY tailor-meta tailor-meta
COPY rosdistro rosdistro
RUN pip3 install -e tailor-meta --break-system-packages

# Rename default ubuntu user to tailor
RUN usermod -md /home/tailor -s /bin/bash -l tailor ubuntu
RUN groupmod -n tailor ubuntu
RUN echo "tailor ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers
USER tailor
RUN mkdir -p /home/tailor && \
    usermod -d /home/tailor tailor
