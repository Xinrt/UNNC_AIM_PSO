#!/bin/bash

cppname=$1
outname=${main%.*}
outname=$main".out"
g++ $main -o $main
./$main