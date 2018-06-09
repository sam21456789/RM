#!/bin/bash

make clean all
gdb -ex=r --args ./rmh	200/c1/c101 1 5



