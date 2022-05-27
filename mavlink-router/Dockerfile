FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        git \
        python3-pip \
        ninja-build \
        pkg-config \
        gcc \
        g++ \
        systemd && \
     rm -rf /var/lib/apt/lists/*

RUN pip3 install meson

#RUN git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git --branch v2
RUN git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git
WORKDIR /mavlink-router
RUN  meson setup --buildtype=release build . && \
    ninja -C build && \
    ninja -C build install

WORKDIR /
COPY main.conf /etc/mavlink-router/main.conf

CMD ["mavlink-routerd"]

