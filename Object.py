#!/usr/bin/python

## 
## Holds Obstacle objects
##
## @author Aswathi Radhakrishnan (UNCC ID: 800936796)
##

class Object:
    x = 0
    y = 0
    height = 0
    width = 0
    speed = 0
    pygameObj = None
    def __init__(self, pygameObj, x, y, height, width):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.pygameObj = pygameObj
    
