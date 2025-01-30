#!/bin/bash

# Update system packages and install required tools
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl git build-essential

# Install Node Version Manager (nvm)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.4/install.sh | bash

# Load nvm (needed to use nvm in the script)
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
[ -s "$NVM_DIR/bash_completion" ] && . "$NVM_DIR/bash_completion"

# Install the latest LTS version of Node.js
nvm install --lts
nvm use --lts

# Reload the shell so that npm and node are recognized
hash -r

# Check if npm is available
if ! command -v npm &> /dev/null; then
    echo "npm could not be found. Please ensure Node.js and npm are installed correctly."
    exit 1
fi

# Install Angular CLI globally
npm install -g @angular/cli

# Navigate to project directory (update this path as needed)
PROJECT_DIR="$HOME/INF3995-102/client"
cd "$PROJECT_DIR" || { echo "Project directory not found!"; exit 1; }

# Clean existing dependencies (if any)
rm -rf node_modules package-lock.json

# Install project dependencies
npm install

# Build the Angular project
ng build

# Confirm success
echo "Environment setup complete. You can now run ng serve."

