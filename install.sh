#!/bin/bash
VERSION=0.1
dkms add .
dkms build Guncon3 -v ${VERSION}
dkms install --force Guncon3 -v ${VERSION}
