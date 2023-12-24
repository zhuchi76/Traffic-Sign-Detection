#!/bin/bash

# install.sh

# Script to install Python 3, pip3, and required Python packages

# Install build dependencies
sudo apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    g++

# Install packages from requirements.txt
echo "Installing Python packages from requirements.txt..."
pip3 install -r requirements.txt

echo "Installation complete."
