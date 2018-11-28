#!/usr/bin/env bash

UART=${1:-/dev/cu.SLAB_USBtoUART}

echo
echo "============================"
echo "  Projector status monitor"
echo "    press ctrl+C to exit"
echo "============================"
echo

stty -f $UART 115200 & cat $UART
