#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/myplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    MyNewModule = RaveCreateModule(env,'MyNewModule')
    print MyNewModule.SendCommand('help')
finally:
    RaveDestroy()
