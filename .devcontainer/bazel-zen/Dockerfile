################################################################################
# Custom Extension
################################################################################

################################################################################
# Configurable Arguments
################################################################################

ARG BASE_IMAGE=focal
ARG BAZEL_VERSION=6.1.1
ARG NAME=bazel-zen

################################################################################
# Base Image
################################################################################

FROM mcr.microsoft.com/devcontainers/base:${BASE_IMAGE}

################################################################################
# Unset gcr configured ubuntu user
################################################################################

USER root

################################################################################
# Tools & Utilities
################################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    # development
    curl \
    bash \
    g++ \
    gcc \
    git \
    graphviz \
    make \
    pkg-config \
    ssh \
    wget \
    vim \
    less \
    zip \
    # python
    python3-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

################################################################################
# Bazelisk
################################################################################

ARG BAZELISK_VERSION=v1.10.1
ARG BAZELISK_DOWNLOAD_SHA=dev-mode
RUN curl -fSsL -o /usr/local/bin/bazelisk https://github.com/bazelbuild/bazelisk/releases/download/${BAZELISK_VERSION}/bazelisk-linux-amd64  && \
    ([ "${BAZELISK_DOWNLOAD_SHA}" = "dev-mode" ] || echo "${BAZELISK_DOWNLOAD_SHA} */usr/local/bin/bazelisk" | sha256sum --check - ) && \
    chmod 0755 /usr/local/bin/bazelisk && \
    # The only way to get root (CI), vscode (codespaces) & zen
    # (local devcontainer) on the same page without a
    # three-way .bazelrc config
    sudo ln -s /usr/local/bin/bazelisk /usr/local/bin/bazel

ENV USE_BAZEL_VERSION "${BAZEL_VERSION}"

################################################################################
# Login Shell
################################################################################

ENV TERM xterm-256color
ENTRYPOINT ["/bin/bash", "--login", "-i"]

################################################################################
# User 'zen'
################################################################################

ARG USERNAME=zen

# Automatically creates a UID, group ${USERNAME}, adds ${USERNAME} to the group
#
# NB: If using this image as a base for a devcontainer, it is critical to have
# just one user, since dynamic UID mapping will not work otherwise, refer to:
#   https://github.com/microsoft/vscode-remote-release/issues/1155
#
RUN useradd -o --uid 1000 -s "/bin/bash" -m ${USERNAME} && \
    apt-get install -y sudo && \
    echo "${USERNAME} ALL=NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "export PS1='\[\033[01;36m\](docker)\[\033[00m\] \[\033[01;32m\]\u@${NAME}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc && \
    echo "alias ll='ls --color=auto -alFNh'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ls='ls --color=auto -Nh'" >> /home/${USERNAME}/.bashrc

ENV BLESSING="May you be blessed by a tickle from his noodly appendages..."
