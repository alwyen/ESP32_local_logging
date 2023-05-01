#!/bin/bash
idf.py flash
idf.py -p /dev/tty.usbserial-1440 monitor
