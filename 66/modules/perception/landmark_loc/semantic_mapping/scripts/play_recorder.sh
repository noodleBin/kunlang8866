#!/bin/bash

source cyber/setup.bash
for file in $1/*.record; do
    cyber_recorder play -f $file
done