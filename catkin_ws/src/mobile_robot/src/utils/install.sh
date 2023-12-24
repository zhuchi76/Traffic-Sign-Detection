#!/bin/bash

# install.sh

# Script to install Python 3, pip3, and required Python packages

# Update the package list
echo "Updating package list..."
sudo apt-get update

# Install Python 3
echo "Installing Python 3..."
sudo apt-get install -y python3

# Check if pip3 is installed, install if it's not
if ! command -v pip3 &> /dev/null
then
    echo "pip3 not found, installing pip3..."
    sudo apt-get install -y python3-pip
else
    echo "pip3 is already installed."
fi

# Install packages from requirements.txt
echo "Installing Python packages from requirements.txt..."
pip3 install -r requirements.txt

echo "Installation complete."
