#!/bin/bash

png_file=${dot_file%.dot}.png
dot -Tpng $dot_file > $png_file

