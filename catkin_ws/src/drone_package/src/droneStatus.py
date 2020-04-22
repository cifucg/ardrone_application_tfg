#!/usr/bin/env python

# This file contains the implementation that we use to known the state that the drone stay in every moment
class DroneStatus(object):
    Emergency = 0
    Inited = 1
    Landed = 2
    Flying = 3
    Hovering = 4
    Test = 5
    TakingOff = 6
    GoToHover = 7
    Landing = 8
    Looping = 9
