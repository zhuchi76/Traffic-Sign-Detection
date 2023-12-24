#!/bin/bash

sudo apt-get update && sudo apt-get install -y \
    build-essential \
    cmake \
    python \
    python-dev \
    python-pip \
    g++

# Install packages from requirements.txt
echo "Installing Python packages from requirements.txt..."
sudo pip install -r requirements.txt

echo "Installation complete."
