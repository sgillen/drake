#!/bin/bash
set -e -u

cd $(dirname $0)

# @ref http://www.andrewhazelden.com/blog/2012/04/automate-your-meshlab-workflow-with-mlx-filter-scripts/

orig_dir=..
files=$(cd $orig_dir && ls *.obj)
set -x
for file in $files; do
    orig_file=$orig_dir/$file
    new_file=./$file
    meshlabserver -i $orig_file -o $new_file -s ./decimate.mlx
done
