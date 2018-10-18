#!/bin/sh

# Pull Docker base image from docker hub
docker pull c1yxuan/avpg_base

# Build Docker Image
docker build -t mavs -f Dockerfile .