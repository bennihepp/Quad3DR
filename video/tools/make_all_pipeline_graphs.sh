#!/bin/bash

for dot_file in `ls *.dot`; do
    png_file=${dot_file%.dot}.png
    dot -Tpng ${dot_file} > ${png_file}
done

