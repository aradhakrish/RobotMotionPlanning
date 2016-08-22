#!/usr/bin/python

## 
## Holds Node objects
##
## @author Aswathi Radhakrishnan (UNCC ID: 800936796)
##

class Node:
    x = 0
    y = 0
    adj = []
    distance = 0
    predecessor = None
    def __init__(self, x, y):
        self.x = x
        self.y = y
