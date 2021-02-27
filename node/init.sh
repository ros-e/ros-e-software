#!/bin/bash

echo "###########################################################################################"
echo "Install node apps"
echo "###########################################################################################"

# Install node packages
cd /home/rose/software/node/rose-api/ && npm install
cd /home/rose/software/node/rose-frontend/ && npm install
cd /home/rose/software/node/node-tool-api/ && npm install
cd /home/rose/software/node/node-tool-frontend/ && npm install

echo "###########################################################################################"
echo "Start node apps"
echo "###########################################################################################"

# Start services
cd /home/rose/software/node/rose-api/ && pm2 start api.js --watch -n rose-api
cd /home/rose/software/node/rose-frontend/ && pm2 start app.js --watch -n rose-frontend
cd /home/rose/software/node/node-tool-api/ && pm2 start api.js -n node-tool-api
cd /home/rose/software/node/node-tool-frontend/ && pm2 start app.js -n node-tool-frontend

pm2 save

pm2 startup
