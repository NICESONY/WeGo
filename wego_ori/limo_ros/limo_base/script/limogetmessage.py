#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import time
limo=limo.LIMO()
#limo.EnableCommand()          # 没有控制LIMO 可以不使能控制
while True:
    time.sleep(0.1)
    print(limo.GetLinearVelocity())
