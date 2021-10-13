#!/usr/bin/env python
#
# State.py
# Description: Program for a class named State, for TuringMachine.py
# Programmer: Adrian Beehner
# Date: 11/22/16

# Imports
from __future__ import print_function



#  Class
class State(object):
	
    class Directions:
        RIGHT = u'RIGHT'
        LEFT = u'LEFT'
        NULL = u'NULL'

    name = None
    nextState = None
    toWrite = str()
    direction = None

    def __init__(self):
        self.name = ""
        self.nextState = ""
        self.toWrite = '0'
        self.direction = self.Directions.NULL

    def setName(self, name):
        self.name = name

    def setToWrite(self, toWrite):
        self.toWrite = toWrite

    def setNextState(self, nextState):
        self.nextState = nextState

    def setDirection(self, direction):
        self.direction = direction

    def getName(self):
        return self.name

    def getToWrite(self):
        return self.toWrite

    def getNextState(self):
        return self.nextState

    def getDirection(self):
        return self.direction
