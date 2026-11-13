# syntax=docker/dockerfile:1
ARG OS_VERSION=noble
FROM ubuntu:${OS_VERSION}
ARG OS_VERSION
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-apt python3-venv python3-dev build-essential cmake ninja-build ccache \
    git sudo debhelper fakeroot ca-certificates && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/tailor_venv
ENV PATH=/opt/tailor_venv/bin:${PATH}
COPY . /tailor-meta
RUN pip install /tailor-meta colcon-common-extensions colcon-clean \
    'git+https://github.com/ruffsl/colcon-cache.git'
ENV OS_VERSION=${OS_VERSION}
WORKDIR /work
CMD ["python3", "/tailor-meta/ci/smoke/build.py"]
