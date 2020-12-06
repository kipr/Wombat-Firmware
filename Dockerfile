FROM ubuntu

ENV TZ=Europe/Vienna
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