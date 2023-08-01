#! /bin/bash

#echo -n -e \\xd8\\x05\\x07\\x08\\x09\\x0a\\x0c\\xd8 > /dev/pts/6

socat -d -d pty,raw,echo=0 pty,raw,echo=0
