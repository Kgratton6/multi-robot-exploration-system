#!/bin/bash

# Ensure script stops on error
set -e

# Load NVM
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"

# Install NVM if not installed
if ! command -v nvm &> /dev/null
then
    echo "NVM not found. Installing..."
    curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.4/install.sh | bash
    source ~/.bashrc
fi

# Install latest LTS version of Node.js
nvm install --lts
nvm use --lts

# Show Node.js and npm versions
node -v
npm -v

# Clean existing dependencies (if any)
rm -rf node_modules package-lock.json

# Install project dependencies
npm install

# Install NestJS CLI globally
npm install -g @nestjs/cli

# Fix missing TypeScript definitions
npm install --save-dev @types/node

# Ensure NestJS core packages are installed
npm install @nestjs/core @nestjs/common @nestjs/platform-express

# Ensure rclnodejs is installed correctly
npm install rclnodejs

# Run the NestJS server
npm run start

# Confirm success
echo "✅ Environment setup complete. You can now run 'npm run start'."
