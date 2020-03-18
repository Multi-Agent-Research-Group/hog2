#-------------------------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License. See https://go.microsoft.com/fwlink/?linkid=2090316 for license information.
#-------------------------------------------------------------------------------------------------------------

FROM ubuntu:latest

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
ENV http_proxy=http://proxy-lmi.global.lmco.com:80
ENV https_proxy=${http_proxy}
ENV HTTP_PROXY=${http_proxy}
ENV HTTPS_PROXY=${http_proxy}
ENV CERT_PATH=/usr/local/share/lm_combined_certs.pem
RUN mkdir -p /etc/ssl/certs/ /usr/local/share/ca-certificates/
WORKDIR /usr/local/share/ca-certificates/
COPY lm_combined_certs.pem ./lm_combined_certs.cer
RUN cp /usr/local/share/ca-certificates/lm_combined_certs.cer /etc/ssl/certs/ca-certificates.crt
RUN cp /usr/local/share/ca-certificates/lm_combined_certs.cer ${CERT_PATH}
RUN apt-get update
RUN apt-get -y install --no-install-recommends ca-certificates
RUN update-ca-certificates
WORKDIR /


# This Dockerfile adds a non-root user with sudo access. Use the "remoteUser"
# property in devcontainer.json to use it. On Linux, the container user's GID/UIDs
# will be updated to match your local UID/GID (when using the dockerFile property).
# See https://aka.ms/vscode-remote/containers/non-root-user for details.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Configure apt and install packages
RUN apt-get update \
    && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
    # 
    # Verify git, process tools, lsb-release (useful for CLI installs) installed
    && apt-get -y install git iproute2 procps lsb-release \
    #
    # Install C++ tools
    && apt-get -y install build-essential cmake cppcheck valgrind make libgtest-dev\
    && cd /usr/src/gtest \
    && cmake CMakeLists.txt \
    && make \
     
    # copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
    && cp *.a /usr/lib \
    #
    # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
    #&& groupadd --gid $USER_GID $USERNAME \
    #&& useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    ## [Optional] Add sudo support for the non-root user
    #&& apt-get install -y sudo \
    #&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    #&& chmod 0440 /etc/sudoers.d/$USERNAME \
    #
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog
