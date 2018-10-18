#!/bin/sh

# Pull Docker base image from docker hub
# docker pull c1yxuan/kinetic-desktop-opengl

# Build Docker Image
docker build -t avpg_base -f Dockerfile .