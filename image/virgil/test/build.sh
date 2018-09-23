#!/bin/sh

# Build Docker Image

nvidia-docker build -t mavs -f Dockerfile .
