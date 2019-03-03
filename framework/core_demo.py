#!/usr/bin/env python3

#Author: Zhiwei Luo

from map import Map
from mouse import Micromouse
from strategy import StrategyTestMultiDFS, Rendezvous
from controller import COREController
from socket import *

mazeMap = Map(16, 16)
mazeMap.readFromFile('/media/psf/VM_Dev/Micromouse/mazes/2012japan-ef.txt')
micromouse = Micromouse(mazeMap)
index = gethostname()[1:]
initPoint = {'1':(0,0), '2':(15,0), '3':(0,15), '4':(15,15)}
micromouse.setMotorController(COREController(index, initPoint[index], '10.0.0.254'))
micromouse.setInitPoint(initPoint[index][0], initPoint[index][1])
micromouse.addTask(Rendezvous(micromouse))
micromouse.run()
