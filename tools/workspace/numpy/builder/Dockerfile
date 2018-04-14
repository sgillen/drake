FROM ubuntu:16.04

COPY ./install_prereqs.sh /
COPY ./build.sh /

RUN apt update -y && \
    yes | /install_prereqs.sh && \
    rm -rf /var/lib/apt/lists/* && apt-get clean all
