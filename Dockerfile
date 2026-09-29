# Pinned so the toolchain doesn't change underneath the build. Newer images ship
# CMake 4, which rejects this project's cmake_minimum_required(VERSION 2.8.12).
FROM ubuntu:24.04

ENV TZ=America/Chicago
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && \
    apt-get install -y ssh \
    build-essential \
    gcc \
    g++ \
    gdb \
    clang \
    cmake \
    gcc-arm-none-eabi \
    wget \
    unzip && \
    apt-get clean

WORKDIR /home/dev/Wombat-Firmware

CMD ["bash", "build.sh"]