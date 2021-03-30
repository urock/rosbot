#!/bin/bash

image_name=rosbot-control

docker build . -t $image_name

echo "Image Built"
