#!/bin/sh

# Pull Docker base image from docker hub
docker pull avpg/cain:base_cudagl

# Build Docker Image
docker build -t mavs -f Dockerfile .
