# syntax=docker/dockerfile:1
ARG OS_VERSION
FROM ubuntu:${OS_VERSION} AS base
ARG OS_VERSION
ARG IMPLEMENTATION_SHA
ARG SOURCE_REPOSITORY=https://github.com/locusrobotics/tailor-meta
LABEL org.opencontainers.image.source="${SOURCE_REPOSITORY}" \
      org.opencontainers.image.revision="${IMPLEMENTATION_SHA}" io.tailor.os="${OS_VERSION}"
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 PYTHONUNBUFFERED=1 HOME=/home/tailor
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates locales curl gnupg sudo git unzip xz-utils bzip2 \
    python3-dev python3-venv python3-apt apt-transport-s3 && \
    locale-gen en_US.UTF-8 && mkdir -p /home/tailor && \
    printf 'Acquire::Retries "3";\nAcquire::http::Timeout "30";\n' > /etc/apt/apt.conf.d/99-tailor && \
    curl -fsSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x142D5F1683E1528B' \
      | gpg --dearmor -o /etc/apt/trusted.gpg.d/tailor.gpg && \
    rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/tailor_venv
ENV PATH=/opt/tailor_venv/bin:${PATH}
COPY . /tailor-meta
RUN pip install -e /tailor-meta && rosdep init
WORKDIR /work

FROM base AS builder
LABEL io.tailor.role="builder"
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ninja-build cmake ccache lld debhelper fakeroot \
    software-properties-common expect && rm -rf /var/lib/apt/lists/*
RUN pip install -e /tailor-meta colcon-common-extensions colcon-clean \
    'git+https://github.com/ruffsl/colcon-cache.git@842696e3e437717046fd31a43a50be4070f23640'
ARG RUSTUP_VERSION=1.29.0
ARG RUSTUP_SHA256=4acc9acc76d5079515b46346a485974457b5a79893cfb01112423c89aeb5aa10
ENV RUSTUP_HOME=/opt/rust/rustup CARGO_HOME=/opt/rust/cargo PATH=/opt/rust/cargo/bin:${PATH}
RUN curl --proto '=https' --tlsv1.2 -fsSL \
      "https://static.rust-lang.org/rustup/archive/${RUSTUP_VERSION}/x86_64-unknown-linux-gnu/rustup-init" \
      -o /tmp/rustup-init && echo "${RUSTUP_SHA256}  /tmp/rustup-init" | sha256sum -c - && \
    chmod +x /tmp/rustup-init && /tmp/rustup-init -y --no-modify-path --profile minimal && \
    rm /tmp/rustup-init && chmod -R a+rwX /opt/rust
ARG RESTIC_VERSION=0.18.1
RUN curl -fsSL "https://github.com/restic/restic/releases/download/v${RESTIC_VERSION}/restic_${RESTIC_VERSION}_linux_amd64.bz2" \
      | bunzip2 > /usr/local/bin/restic && chmod +x /usr/local/bin/restic

FROM base AS tools
LABEL io.tailor.role="tools"
RUN apt-get update && apt-get install -y --no-install-recommends ruby ruby-bundler && \
    rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://github.com/aptly-dev/aptly/releases/download/v1.5.0/aptly_1.5.0_linux_amd64.tar.gz \
      | tar xz --strip-components 1 -C /usr/local/bin
RUN git clone -b per-package https://github.com/locusrobotics/deb-s3.git /opt/deb-s3 && \
    cd /opt/deb-s3 && bundle install && ln -s /opt/deb-s3/bin/deb-s3 /usr/local/bin/deb-s3

FROM base AS imager
LABEL io.tailor.role="imager"
RUN apt-get update && apt-get install -y --no-install-recommends \
    ansible debootstrap gcc iptables libguestfs-tools lxc openssh-client wget && \
    rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://get.docker.com/ | VERSION=28.0.0 sh
RUN curl -fsSL https://releases.hashicorp.com/packer/1.9.1/packer_1.9.1_linux_amd64.zip -o /tmp/packer.zip && \
    unzip /tmp/packer.zip -d /usr/local/bin && rm /tmp/packer.zip && \
    curl -fsSL https://github.com/locusrobotics/packer-builder-qemu-chroot/releases/download/v1.1.0-u1/packer-builder-qemu-chroot_linux_amd64.tar.gz \
      | tar xz -C /usr/local/bin packer-plugin-qemu-chroot
RUN pip install catkin_tools
