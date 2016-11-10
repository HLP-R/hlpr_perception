#!/bin/bash
files=$(find ./ -name 'CMakeLists.txt')
for file in $files 
do
    echo $file
    sed -i 's/find_package(PCL 1.8 REQUIRED COMPONENTS)/find_package(PCL 1.7 REQUIRED COMPONENTS)/g' $file
done
