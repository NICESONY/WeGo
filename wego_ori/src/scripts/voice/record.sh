#!/bin/bash
arecord -D plughw:2,0 -f S16_LE -r 48000 -c 2 $1.wav
