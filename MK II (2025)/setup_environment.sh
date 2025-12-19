#!/bin/bash

ENV_NAME="gisp_cam"

echo "Where do you want to create the Python virtual environment?"
echo "1) Current Directory: $(pwd)/$ENV_NAME"
echo "2) Home Directory: ~/$(basename ~)/$ENV_NAME"
read -p "Choose 1 or 2: " choice

if [[ "$choice" == "1" ]]; then
    ENV_PATH="$(pwd)/$ENV_NAME"
elif [[ "$choice" == "2" ]]; then
    ENV_PATH="$HOME/$ENV_NAME"
else
    echo "Invalid choice. Exiting."
    exit 1
fi

echo "Creating virtual environment at: $ENV_PATH"

# Create the virtual environment
python3 -m venv "$ENV_PATH"

# Activate it
source "$ENV_PATH/bin/activate"

# Upgrade pip
pip install --upgrade pip

# Install OpenCV
pip install opencv-python

# Confirm install
python -c "import cv2; print('cv2 version:', cv2.__version__)"

echo "✅ Environment '$ENV_NAME' created at $ENV_PATH and cv2 installed."

echo "\n\nTo activate it later:"
echo "source $ENV_PATH/bin/activate"
