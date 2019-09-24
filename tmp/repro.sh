#!/bin/bash

make
python repro_issue12073.py > dl_second.txt
python repro_issue12073.py --torch_first > dl_first.txt

