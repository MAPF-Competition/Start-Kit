FROM ubuntu:latest
ARG DEBIAN_FRONTEND=noninteractive

# Dependicies Installation Example
RUN apt-get update
RUN apt-get --yes --no-install-recommends install make cmake build-essential

# Copy codes to target dir and set codes dir to be the working directory.
# Then run compile.sh to compile codes.
COPY ./. /codes/
WORKDIR /codes/
