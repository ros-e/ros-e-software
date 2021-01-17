#!/bin/bash

echo "###########################################################################################"
echo "Install node apps"
echo "###########################################################################################"

# Install node process manager
npm install pm2 -g

# Install node packages
npm install ~/software/node/rose-api/ 
npm install ~/software/node/rose-frontend/ 

echo "###########################################################################################"
echo "Start node apps"
echo "###########################################################################################"


# Start services
pm2 start ~/software/node/rose-api/api.js --watch -n rose-api
pm2 start ~/software/node/rose-frontend/app.js --watch -n rose-frontend
pm2 save