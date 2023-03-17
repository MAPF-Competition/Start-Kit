#!/usr/bin/env bash
echo "Only run this script in the root of your code base."
echo "Create Dockerfile using apt.txt"

# Specify which docker to use.
content="FROM ubuntu:jammy\nRUN apt-get update\n"

# Read packages to install.
pkgs='RUN ["apt-get", "install", "--yes", "--no-install-recommends"'
while read -r line;
do
   pkgs="${pkgs},\"$line\"" ;
done < apt.txt
pkgs="${pkgs}]\n"
content="${content}${pkgs}"

# Copy codes to target dir and set codes dir to be the working directory.
# Then run compile.sh to compile codes.
content="${content}COPY ./. /GPPC2021/codes/ \n"
content="${content}WORKDIR /GPPC2021/codes/ \n"
content="${content}RUN chmod u+x compile.sh \n"
content="${content}RUN ./compile.sh \n"
echo -e $content > Dockerfile

echo "Remove container and images if exist... ..."
out=$(docker container stop gppc_test 2>&1 ; docker container rm gppc_test 2>&1 ; docker rmi gppc_image 2>&1)

echo "Build image and run the container... ..."
docker build -t gppc_image ./
docker container run -it --name gppc_test gppc_image
