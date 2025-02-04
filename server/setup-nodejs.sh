#!/bin/bash

curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.4/install.sh | bash

source ~/.bashrc
# Install the latest LTS version of Node.js
nvm install --lts
nvm use --lts

# Clean existing dependencies (if any)
rm -rf node_modules package-lock.json

# Install project dependencies
npm install

npm install -g @nestjs/cli
npx @nestjs/cli new . --skip-git

npm run start

# Confirm success
echo "Environment setup complete. You can now run ng serve."