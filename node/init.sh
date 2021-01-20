#!/bin/bash

echo "###########################################################################################"
echo "Install node apps"
echo "###########################################################################################"

# Install node packages
npm install /home/rose/software/node/rose-api/ 
npm install /home/rose/software/node/rose-frontend/ 

echo "###########################################################################################"
echo "Start node apps"
echo "###########################################################################################"

# Start services
cd /home/rose/software/node/rose-api/ && pm2 start api.js --watch -n rose-api
cd /home/rose/software/node/rose-frontend/ && pm2 start app.js --watch -n rose-frontend
pm2 save

