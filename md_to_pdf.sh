#!/bin/sh

for f in md/*.md
do
    new_file=$(basename $f)
    pandoc "$f" -o "pdf/${new_file%.md}.pdf"
done 