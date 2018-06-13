#!/bin/bash

make clean all
gdb -ex=r --args ./rmh	400/c1/c101 1 5
##time ./rmh 400/c1/c101 1 5


