#!/bin/bash

for file in $(ls *.txt); do
    echo $file
    sed -i 's/dBmfrom/\n/g' $file
    sed -i '1d' $file
    done