#!/bin/sh

# Pull Docker base image from docker hub
docker pull avpg/cain:middle

# Build Docker Image
docker build -t mavs -f Dockerfile .
