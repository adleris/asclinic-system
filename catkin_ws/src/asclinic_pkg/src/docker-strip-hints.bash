#!/usr/bin/bash
# remove type hints from python files
#
# needs to be run from the same directory that those files are in.
# Is not very sophisticated, but does the job.
#
# Requires strip-hints module saved in /root/strip-hints, which is done
# as part of docker image creation.

for file in $(ls | grep "\.py$"); do
    python3 /root/strip-hints/bin/strip_hints.py $file > /tmp/stripped \
    && cat /tmp/stripped > $file
done
