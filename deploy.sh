#!/bin/bash
#
#  Modified by: Tobias Madlberger*, 2025
#  * Corresponding author's email: tobias.madlberger@gmail.com
#

set -e  # Exit immediately if a command exits with a non-zero status

USER=kipr # Replace with your username
HOST=192.168.104.179 # Replace with your host IP address
DIR=/home/${USER}/wombat-os/flashFiles # Replace with your target directory on the remote host

# Start Docker Compose in detached mode
docker compose up || { echo "Failed to start Docker Compose"; exit 1; }

# Copy firmware file to the remote host
scp build/Firmware/wombat.bin $USER@$HOST:$DIR/wombat.bin || exit 1

# Execute the flash script on the remote host
ssh $USER@$HOST "cd $DIR && bash ./wallaby_flash"

