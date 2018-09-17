#!/bin/sh

# Build Docker Image

nvidia-docker build -t base -f Dockerfile .
