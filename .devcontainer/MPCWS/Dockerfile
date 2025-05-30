# Use Python 3.12 slim image as base
FROM python:3.12-slim-bookworm

COPY requirements.txt /tmp/requirements.txt

# Set time zone
RUN ln -fs /usr/share/zoneinfo/Europe/Berlin /etc/localtime

# Update package list and install minimal required packages
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    build-essential \
    libopenblas-dev coinor-libipopt-dev gfortran \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# Set the working directory
WORKDIR /app

