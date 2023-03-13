#!/bin/bash
VERSION=0.1 
dkms remove -m Guncon3 -v ${VERSION} --all
rmmod guncon3
