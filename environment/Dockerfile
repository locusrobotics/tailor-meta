FROM ubuntu:jammy

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
  git

COPY tailor-meta tailor-meta
RUN pip3 install -e tailor-meta

# Create non-root user
RUN groupadd -r tailor && useradd -ms /bin/bash -g tailor -G sudo tailor
RUN echo "tailor ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers
USER tailor
RUN mkdir -p /home/tailor && \
    usermod -d /home/tailor tailor
