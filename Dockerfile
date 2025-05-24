FROM alpine:latest

# Install required packages
RUN apk add alpine-sdk bc bison dtc flex gnutls-dev linux-headers ncurses-dev \
  openssl-dev py3-elftools py3-setuptools python3-dev swig util-linux-dev arm-trusted-firmware

# Set workdir
WORKDIR /u-boot

# Default entry
CMD ["/bin/sh"]
