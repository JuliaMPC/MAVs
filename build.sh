#!/bin/sh

# Pull Docker base image from docker hub
docker pull avpg/cain:middle

# Build Docker Image
#docker build --no-cache -t mavs -f Dockerfile .
docker build -t mavs -f Dockerfile .
