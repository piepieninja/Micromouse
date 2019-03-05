#!/usr/bin/env python3

#Author: Zhiwei Luo
#Modified by: Caleb Adams

from task import Strategy, NetworkInterface
from time import sleep

import logging
import math
import glob
import os

CORE_CELL_WIDTH  = 72
CORE_CELL_HEIGHT = 48

cell_x = CORE_CELL_WIDTH
cell_y = CORE_CELL_HEIGHT

def setLog():
	logging.basicConfig(filename="/tmp/whytho/testing.log", level=logging.DEBUG)

#
# Because the logging system of this is
# so damn confusing and unintuitive
#
def logthis(str):
	logging.debug(str)


# There was no documentation so
# I was basically forced to do
# this just to get this done
def get_all_xy():
	data = []
	# logthis("attempting xy get")
	for path in glob.glob("/tmp/pycore*"):
		# this just gets the real path
		# open each file
		file1 = open(path + "/n1.xy","r+")
		file2 = open(path + "/n2.xy","r+")
		file3 = open(path + "/n3.xy","r+")
		file4 = open(path + "/n4.xy","r+")

		# parse the coords out
		# still as string values
		line1 = file1.readline().strip("\n").split(" ")
		line2 = file2.readline().strip("\n").split(" ")
		line3 = file3.readline().strip("\n").split(" ")
		line4 = file4.readline().strip("\n").split(" ")

		#  make easy struct and convert to float
		n1 = [int(float(line1[0])/cell_x),int((float(line1[1]))/cell_y)]
		n2 = [int(float(line2[0])/cell_x),int((float(line2[1]))/cell_y)]
		n3 = [int(float(line3[0])/cell_x),int((float(line3[1]))/cell_y)]
		n4 = [int(float(line4[0])/cell_x),int((float(line4[1]))/cell_y)]

		# add the lines to the data
		data.append(n1)
		data.append(n2)
		data.append(n3)
		data.append(n4)

		file1.close()
		file2.close()
		file3.close()
		file4.close()
	return data

#
# Get the centroid from the set of points
#
def get_centroid(set):
	x_total = 0
	y_total = 0
	for coord in set:
		x_total = x_total + coord[0]
		y_total = y_total + coord[1]
	x_center = x_total/4
	y_center = y_total/4
	return [int(x_center),int(y_center)]

#
# returns true if graph contains expored location
#
def graph_contain(location,graph):
	for boi in graph:
		logthis(boi[0])
		if (boi[0][0] == location[0] and boi[0][1] == location[1]):
			return True
	return False

#
# returns true if cells are neighbors
#
def is_neighbor(cell0, cell1):
	delta_x = abs(cell0[0]-cell1[0])
	delta_y = abs(cell0[1]-cell1[1])
	if (delta_x == 1 and delta_y == 0):
		return True
	elif (delta_x == 0 and delta_y == 1):
		return True
	else:
		return False

#
# get the euclid dist from two coords
#
def heuristic_dist(loc,centroid):
	x_mag = abs(loc[0] - centroid[0])
	y_mag = abs(loc[1] - centroid[1])
	return (x_mag**2 + y_mag**2)

#
# gets the location of the new goal
#
def get_new_goal(location, center, graph):
	goal = []
	h_dist = 999999
	for node in graph:
		for connection in node[1]:
			logthis("loc: " + str(location) + " | conn: " + str(connection))
			if (heuristic_dist(location,center) < h_dist and not graph_contain(connection,graph)):
				h_dist = heuristic_dist(location,center)
				goal = connection
	return goal

#
# This is really the A* part of the alg
#
def get_path_to_goal(location, goal, graph):
	path = []
	# TODO 	write this if the greedy doesn't work
	return path

#
# sort of like greedy A*
# assumes we're always next to an unexpored region
# Always picks the best unexpored region to move towards
# this should get stuck in dead ends
#
def greedy_get_step(connections,center,graph):
	next_step = []
	h_dist = 999999
	for node in connections:
		if (heuristic_dist(node,center) < h_dist and not graph_contain(node,graph)):
			h_dist = heuristic_dist(node,center)
			next_step = node
	return next_step

#
# sort of like greedy A*
# assumes we're always next to an unexpored region
# Always picks the best unexpored region to move towards
# this should get stuck in dead ends
#
def greedy_get_path_go_goal(location,center,connections,graph):
	path = []
	h_score = 999999
	for node in connections:
		if (heuristic_dist(node,center) < h_dist and not graph_contain(node,graph)):
			h_dist = heuristic_dist(node,center)
			path = node
	return path

#
# Returns true if the location and the query are connected by a single vertex
#
def is_connected(location,query,graph):
	for node in graph:
		# the first element is the normal locations
		if (node[0][0] == location[0] and node[0][1] == location[1]):
			for connection in node[1]:
				logthis("conn: > " + str(connection) + ", q: > " + str(query))
				if (connection[0] == query[0] and connection[1] == query[1]):
					return True
	return False

#
# The node class
#
class Node(object):
	def __init__(self):
		self.up = None
		self.down = None
		self.left = None
		self.right = None
		self.explored = False
		self.x = None
		self.y = None
		self.path_to_me = [] # from root

	def get_children(self):
		ret_boi = []
		if self.up is not None:
			ret_boi.append(self.up)
		if self.down is not None:
			ret_boi.append(self.down)
		if self.left is not None:
			ret_boi.append(self.left)
		if self.right is not None:
			ret_boi.append(self.right)
		return ret_boi

	def get_node(self,loc,ret_node):
		# self should be root
		stack = []
		stack.append(self)
		while (len(stack) > 0):
			curr = stack.pop()
			if (curr.x == loc[0] and curr.y == loc[1]):
				return curr
			else:
				for c in curr.get_children():
					stack.append(c)
		return None
		# logthis("x: " + str(loc[0]) + ", y: " + str(loc[1]))
		# logthis("{" + str(self.x) + ", " + str(self.y) + "}")
		# if (self.x == loc[0] and self.y == loc[1]):
		# 	return self
		# else:
		# 	for c in self.get_children():
		# 		# logthis("node: " + str(n.x) + ", " + str(n.y))
		# 		# logthis("trying to get: " + str(loc[0]) + ", " + str(loc[1]))
		# 		c.get_node(loc,None)
		# return ret_node

	# insert right from the location loc
	def insert_right(self, old_guy):
		# self should be root
		logthis("insert_right")
		new_guy = Node()
		# logthis("old_guy: " + str(old_guy.x) + ", " + str(old_guy.y))
		oof = self.get_node([old_guy.x,old_guy.y],None)
		# logthis("oof: " + str(old_guy.x) + ", " + str(old_guy.y))
		new_guy.x = oof.x + 1
		new_guy.y = oof.y
		# new_guy.path_to_me = oof.path_to_me
		# new_guy.path_to_me.append(new_guy.get_loc())
		oof.right = new_guy
		# new_guy.left = oof
		# insert a node to the right

	def insert_left(self, old_guy):
		# self should be root
		logthis("insert_left")
		new_guy = Node()
		# logthis("old_guy: " + str(old_guy.x) + ", " + str(old_guy.y))
		oof = self.get_node([old_guy.x,old_guy.y],None)
		# logthis("oof: " + str(old_guy.x) + ", " + str(old_guy.y))
		new_guy.x = oof.x - 1
		new_guy.y = oof.y
		# new_guy.path_to_me = oof.path_to_me
		# new_guy.path_to_me.append(new_guy.get_loc())
		oof.left = new_guy
		# new_guy.right = oof

	def insert_down(self, old_guy):
		# self should be root
		logthis("insert_down")
		new_guy = Node()
		# logthis("old_guy: " + str(old_guy.x) + ", " + str(old_guy.y))
		oof = self.get_node([old_guy.x,old_guy.y],None)
		# logthis("oof: " + str(old_guy.x) + ", " + str(old_guy.y))
		new_guy.x = oof.x
		new_guy.y = oof.y + 1
		# new_guy.path_to_me = oof.path_to_me
		# new_guy.path_to_me.append(new_guy.get_loc())
		oof.down = new_guy

	def insert_up(self, old_guy):
		# self should be root
		logthis("insert_up")
		new_guy = Node()
		# logthis("old_guy: " + str(old_guy.x) + ", " + str(old_guy.y))
		oof = self.get_node([old_guy.x,old_guy.y],None)
		# logthis("oof: " + str(old_guy.x) + ", " + str(old_guy.y))
		new_guy.x = oof.x
		new_guy.y = oof.y - 1
		# new_guy.path_to_me = oof.path_to_me
		# new_guy.path_to_me.append(new_guy.get_loc())
		oof.up = new_guy
		# new_guy.down = oof

	def print_all(self):
		logthis("[" + str(self.x) + ", " + str(self.y) + "]")
		for c in self.get_children():
			c.print_all()
			# logthis("[" + str(c.x) + ", " + str(c.y) + "]")

	def get_loc(self):
		return [self.x,self.y]

	def get_leaves(self, stack):
		if not self.explored:
			stack.append(self)
		else:
			for c in self.get_children():
				c.get_leaves(stack)
		return stack

	def get_path(self, root, goal):
		if not root:
			return []
		if (root.x == goal[0] and root.y == goal[1]):
			return [root.get_loc()]
		# clockwise, becasue why not
		returned = self.get_path(root.up,goal)
		if returned:
			return [root.get_loc()] + returned
		returned = self.get_path(root.right,goal)
		if returned:
			return [root.get_loc()] + returned
		returned = self.get_path(root.down,goal)
		if returned:
			return [root.get_loc()] + returned
		returned = self.get_path(root.left,goal)
		if returned:
			return [root.get_loc()] + returned
		return []

#
# Finds the node n, or returns None
#
def findNode(n, loc):
	if (n.x == loc[0] and n.y == loc[1]):
		return n
	else:
		if len(n.get_children()) > 0:
			for c in n.get_children():
				logthis("node: " + str(n.x) + ", " + str(n.y))
				findNode(c,loc)
		else:
			return
	return None

#
# Returns True if contains node
#
def containsNode(n, loc):
	if (n.x == loc[0] and n.y == loc[1]):
		return True
	else:
		if (len(n.get_children()) > 0):
			for c in n.get_children():
				logthis("c: " + str(c.x) + ", " + str(c.y))
				containsNode(c,loc)
		else:
			return
	return False

#
# Returns leaves in the tree
#
def getLeaves(n, stack):
	# logthis(n)
	if (not n.explored):
		stack.append(n)
		return
	else:
		if len(n.get_children()) > 0:
			for c in n.get_children():
				getLeaves(c,stack)
	return stack

#
# helper function for getPath
#
def not_in(boi,path):
	for yeet in path:
		if (yeet.x == boi.x and yeet.y == boi.y):
			return False
	return True

#
# get path to goal
#
def getPathToGoal(n, goal, path):
	# this one sort of worked:
	path.append(n)
	if (n.x == goal.x and n.y == goal.y):
		return path
	for boi in n.get_children():
		if not_in(boi,path):
			temp_path = getPathToGoal(boi,goal,path)
			if temp_path:
				return temp_path
	return None


class StrategyTestProgress(Strategy):
	progress = 10

	def checkFinished(self):
		return self.progress <= 0

	def go(self):
		self.progress = self.progress - 1
		print(self.progress)

class StrategyTestCount(Strategy):
	progress = 0

	def checkFinished(self):
		return self.progress > 10

	def go(self):
		self.progress = self.progress + 1
		print(self.progress)
		sleep(1)

class StrategyTestGoDown(Strategy):
	mouse = None
	mapPainter = None
	progress = 0

	def __init__(self, mouse, mapPainter):
		self.mouse = mouse
		self.mapPainter = mapPainter

	def checkFinished(self):
		return self.progress >= 1

	def go(self):
		self.progress = self.progress + 1
		print(self.progress)
		sleep(1)
		self.mouse.goDown()
		self.mouse.goDown()
		self.mouse.goDown()
		self.mouse.goDown()
		self.mouse.goRight()
		self.mouse.goUp()
		cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
		self.mapPainter.putRobotInCell(cell)
		sleep(1)

class StrategyTestDFS(Strategy):
	mouse = None
	mapPainter = None
	isVisited = []
	path = []
	isBack = False

	def __init__(self, mouse, mapPainter):
		self.mouse = mouse
		self.mapPainter = mapPainter
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1

	def checkFinished(self):
		return self.isBack

	def go(self):
		cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
		self.mapPainter.drawCell(cell, 'grey')

		if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x-1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x-1][self.mouse.y] = 1
			self.mouse.goLeft()
		elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y-1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y-1] = 1
			self.mouse.goUp()
		elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x+1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x+1][self.mouse.y] = 1
			self.mouse.goRight()
		elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y+1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y+1] = 1
			self.mouse.goDown()
		else:
			if len(self.path) != 0:
				x, y = self.path.pop()
				if x < self.mouse.x:
					self.mouse.goLeft()
				elif x > self.mouse.x:
					self.mouse.goRight()
				elif y < self.mouse.y:
					self.mouse.goUp()
				elif y > self.mouse.y:
					self.mouse.goDown()
			else:
				self.isBack = True

		cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
		self.mapPainter.putRobotInCell(cell)
		sleep(0.05)

class StrategyTestMultiDFS(Strategy):
	mouse = None
	isVisited = []
	path = []
	isBack = False
	network = None

	def __init__(self, mouse):
		self.mouse = mouse
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()

	def checkFinished(self):
		return self.isBack

	def go(self):
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		sendData = {'x': self.mouse.x, 'y':self.mouse.y, 'up': not self.mouse.canGoUp(), 'down': not self.mouse.canGoDown(), 'left': not self.mouse.canGoLeft(), 'right': not self.mouse.canGoRight()}
		self.network.sendStringData(sendData)
		recvData = self.network.retrieveData()
		while recvData:
			otherMap = recvData
			cell = self.mouse.mazeMap.getCell(otherMap['x'], otherMap['y'])
			self.isVisited[otherMap['x']][otherMap['y']] = 1
			if otherMap['up']: self.mouse.mazeMap.setCellUpAsWall(cell)
			if otherMap['down']: self.mouse.mazeMap.setCellDownAsWall(cell)
			if otherMap['left']: self.mouse.mazeMap.setCellLeftAsWall(cell)
			if otherMap['right']: self.mouse.mazeMap.setCellRightAsWall(cell)
			recvData = self.network.retrieveData()

		if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x-1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x-1][self.mouse.y] = 1
			self.mouse.goLeft()
		elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y-1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y-1] = 1
			self.mouse.goUp()
		elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x+1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x+1][self.mouse.y] = 1
			self.mouse.goRight()
		elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y+1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y+1] = 1
			self.mouse.goDown()
		else:
			if len(self.path) != 0:
				x, y = self.path.pop()
				if x < self.mouse.x:
					self.mouse.goLeft()
				elif x > self.mouse.x:
					self.mouse.goRight()
				elif y < self.mouse.y:
					self.mouse.goUp()
				elif y > self.mouse.y:
					self.mouse.goDown()
			else:
				self.isBack = True

		sleep(0.5)

class Rendezvous(Strategy):
	mouse = None
	isVisited = []
	isBack = False
	network = None
	second = False

	at_goal = False
	has_goal = False
	backtracking = False
	curr_goal = None
	my_position = None
	# graph not struct, per node is:
	# [node, [connection, connection]]
	# [(x,y),[(x,y),(x,y)]
	graph = [] # a big boi
	oof_path = [] # a stack
	temp_oofer_path = [] # used when backgracking, added back when goal found
	curr_leaves = []
	center = [0,0]
	root = Node()
	seen = []


	def __init__(self, mouse):
		self.mouse = mouse
		# self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		# self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()
		setLog()
		self.root.explored = False
		self.root.x 	  = self.mouse.x
		self.root.y 	  = self.mouse.y
		self.root.path_to_me.append(self.root.get_loc())
		# self.root = self.my_position
		# logthis("init successful!")

	def checkFinished(self):
		return self.isBack

	def go(self):

		logthis("<======>")

		# self.root.print_all()

		# sense the walls
		self.mouse.senseWalls()

		# the location in an cleaner format
		loc = [self.mouse.x,self.mouse.y]
		# logthis("expected loc: " + str(loc[0]) + ", " + str(loc[1]))
		# logthis("root: " + str(self.root.x) + ", " + str(self.root.y))

		# get the node we're at
		self.my_position = self.root.get_node(loc,None)
		logthis("pos: " + str(self.my_position.x) + "," + str(self.my_position.y))

		# logthis("=> attempt 2:")
		# self.root.print_all()

		# see if we need to explore
		if (not self.my_position.explored):
			# NOTE: We want a spanning tree
			# we must check if there is already an entry in the tree
			# so that we avoid cycles
			# left case
			if (self.mouse.canGoLeft() and (self.root.get_node([loc[0]-1,loc[1]],None)) is None ) :
				self.root.insert_left(self.my_position)
			if (self.mouse.canGoRight() and (self.root.get_node([loc[0]+1,loc[1]],None)) is None ) :
				self.root.insert_right(self.my_position)
			if (self.mouse.canGoDown() and (self.root.get_node([loc[0],loc[1]+1],None)) is None ) :
				self.root.insert_down(self.my_position)
			if (self.mouse.canGoUp() and (self.root.get_node([loc[0],loc[1]-1],None)) is None ) :
				self.root.insert_up(self.my_position)
			self.my_position.explored = True

		logthis("=> full graph contains:")
		self.root.print_all()
		# if we don't have a goal, get one!
		if not self.has_goal:
			locations = get_all_xy()
			self.center = get_centroid(locations)
			leaves = self.root.get_leaves([])

			# for leaf in leaves:
			# 	logthis("leaf: " + str(leaf.x) + ", " + str(leaf.y))


			h_val = 999999
			for leaf in leaves:
				# logthis("leaf: " + str(leaf.x) + ", " + str(leaf.y))
				t_loc = [leaf.x,leaf.y]
				# logthis(t_loc)
				if (heuristic_dist(t_loc,self.center) < h_val):
					# logthis("=> attempt 4:")
					# self.root.print_all()
					self.curr_goal = self.root.get_node(t_loc,None)
					# logthis(">> " + str(self.curr_goal))
					# logthis(">> " + str(self.root.get_node(t_loc,None)))
					# logthis("wait: " + str(self.root.get_node(t_loc,None).x) + ", " +str(self.root.get_node(t_loc,None).y))
					# logthis("Goal: " + str(self.curr_goal.x) + ", " +str(self.curr_goal.y))
					h_val = heuristic_dist(t_loc,self.center)
			self.has_goal = True
			logthis("Goal: " + str(self.curr_goal.x) + ", " +str(self.curr_goal.y))

			root_partial = self.root.get_path(self.root,self.curr_goal.get_loc())
			curr_partial = reversed(self.root.get_path(self.root,self.my_position.get_loc()))

			self.oof_path = []
			for step in curr_partial:
				self.oof_path.append(step)
			for step in root_partial:
				if not (self.oof_path[len(self.oof_path)-1][0] == step[0] and self.oof_path[len(self.oof_path)-1][1] == step[1]):
					self.oof_path.append(step)


			p_str = "Path: "
			logthis("Path Len: " + str(len(self.oof_path)))
			for node in self.oof_path:
				p_str += str(node[0]) + ", " + str(node[1]) + " -> "
			logthis(p_str)

		# find the path to the goal
		if self.has_goal:
			next_node = self.oof_path.pop()
			if (next_node[0] == self.curr_goal.x and next_node[1] == self.curr_goal.y):
				self.has_goal = False
			# find the way to move
			if (next_node[0] == self.my_position.x + 1):
				self.my_position = self.root.get_node(loc,None)
				self.my_position = self.my_position.right
				self.mouse.goRight()
				logthis("go Right")
			elif (next_node[0] == self.my_position.x - 1):
				self.my_position = self.root.get_node(loc,None)
				self.my_position = self.my_position.left
				self.mouse.goLeft()
				logthis("go Left")
			elif (next_node[1] == self.my_position.y + 1):
				# redo the grab
				self.my_position = self.root.get_node(loc,None)
				self.my_position = self.my_position.down
				self.mouse.goDown()
				logthis("go Down")
			elif (next_node[1] == self.my_position.y - 1):
				self.my_position = self.root.get_node(loc,None)
				self.my_position = self.my_position.up
				self.mouse.goUp()
				logthis("go Up")

		sleep(0.5)

class Rendezvous5(Strategy):
	mouse = None
	isVisited = []
	isBack = False
	network = None
	second = False

	at_goal = False
	has_goal = False
	backtracking = False
	curr_goal = None
	my_position = None
	# graph not struct, per node is:
	# [node, [connection, connection]]
	# [(x,y),[(x,y),(x,y)]
	graph = [] # a big boi
	oof_path = [] # a stack
	temp_oofer_path = [] # used when backgracking, added back when goal found
	curr_leaves = []
	center = [0,0]
	root = Node()


	def __init__(self, mouse):
		self.mouse = mouse
		# self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		# self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()
		setLog()
		self.root.explored = False
		self.root.x 	  = self.mouse.x
		self.root.y 	  = self.mouse.y
		self.root = self.my_position
		# logthis("init successful!")

	def checkFinished(self):
		return self.isBack

	def go(self):

		logthis("<======>")

		# sense the walls
		self.mouse.senseWalls()

		# the location in an cleaner format
		loc = [self.mouse.x,self.mouse.y]

		# get the node we're at
		self.my_position = findNode(self.root, loc)
		logthis("pos: " + str(self.my_position.x) + "," + str(self.my_position.y))

		# see if we need to explore
		if (not self.my_position.explored):
			# NOTE: We want a spanning tree
			# we must check if there is already an entry in the tree
			# so that we avoid cycles
			# left case
			if (self.mouse.canGoLeft() and not containsNode(self.root,[loc[0]-1,loc[1]])):
				newguy = Node()
				newguy.x = self.my_position.x - 1
				newguy.y = self.my_position.y
				newguy.right = self.my_position
				self.my_position.left = newguy
			else:
				self.my_position.left = None
			# right case
			if (self.mouse.canGoRight() and not containsNode(self.root,[loc[0]+1,loc[1]])):
				self.my_position.right = True
				newguy = Node()
				newguy.x = self.my_position.x + 1
				newguy.y = self.my_position.y
				newguy.left = self.my_position
				self.my_position.right = newguy
			else:
				self.my_position.right = None
			# down case
			if (self.mouse.canGoDown() and not containsNode(self.root,[loc[0],loc[1]+1])):
				self.my_position.down = True
				newguy = Node()
				newguy.x = self.my_position.x
				newguy.y = self.my_position.y + 1
				newguy.up = self.my_position
				self.my_position.down = newguy
			else:
				self.my_position.down = None
			# up case
			if (self.mouse.canGoUp() and not containsNode(self.root,[loc[0],loc[1]-1])):
				self.my_position.Up = True
				newguy = Node()
				newguy.x = self.my_position.x
				newguy.y = self.my_position.y - 1
				newguy.down = self.my_position
				self.my_position.up = newguy
			else:
				self.my_position.Up = None
			# we did it boiz
			self.my_position.explored = True

		# this should only happen the first time, and should connect everything:
		if (not self.root.explored):
			self.root.right = self.my_position.right
			self.root.left = self.my_position.left
			self.root.up = self.my_position.up
			self.root.down = self.my_position.down

		# if we don't have a goal, get one!
		if not self.has_goal:
			locations = get_all_xy()
			self.center = get_centroid(locations)
			leaves = getLeaves(self.root,[])
			h_val = 999999
			for leaf in leaves:
				logthis("leaf: " + str(leaf.x) + ", " + str(leaf.y))
				t_loc = [leaf.x,leaf.y]
				if (heuristic_dist(t_loc,self.center) < h_val):
					self.curr_goal = leaf
					h_val = heuristic_dist(t_loc,self.center)
			self.has_goal = True
			logthis("Goal: " + str(self.curr_goal.x) + ", " +str(self.curr_goal.y))
			# now get the path to that goal!
			self.oof_path = getPathToGoal(self.my_position,self.curr_goal,[])
			p_str = "Path: "
			for node in self.oof_path:
				p_str += str(node.x) + ", " + str(node.y) + " -> "
			logthis(p_str)

		# find the path to the goal
		if self.has_goal:
			next_node = self.oof_path.pop()
			if (next_node.x == self.curr_goal.x and next_node.y == self.curr_goal.y):
				self.has_goal = False
			# find the way to move
			if (next_node.x == self.my_position.x + 1):
				self.mouse.goRight()
			elif (next_node.x == self.my_position.x - 1):
				self.mouse.goLeft()
			elif (next_node.y == self.my_position.y + 1):
				self.mouse.goDown()
			elif (next_node.y == self.my_position.y - 1):
				self.mouse.goUp()

			self.my_position = next_node

		# self.center = get_centroid(locations)


		sleep(0.5)

class Rendezvous4(Strategy):
	mouse = None
	isVisited = []
	isBack = False
	network = None
	second = False

	at_goal = False
	backtracking = False
	curr_goal = [-1,-1]
	# graph not struct, per node is:
	# [node, [connection, connection]]
	# [(x,y),[(x,y),(x,y)]
	graph = [] # a big boi
	oof_path = [] # a stack
	temp_oofer_path = [] # used when backgracking, added back when goal found
	curr_leaves = []
	center = [0,0]


	def __init__(self, mouse):
		self.mouse = mouse
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()
		setLog()
		# logthis("init successful!")

	def checkFinished(self):
		return self.isBack

	def go(self):

		logthis("<======>")

		# sense the walls
		self.mouse.senseWalls()

		# the location in an cleaner format
		loc = [self.mouse.x,self.mouse.y]

		# calculate the centroid, but only if not backgracking
		locations = get_all_xy()
		if (not self.backtracking):
			locations = get_all_xy()
			# the centeroid calc should only change here
			self.center = get_centroid(locations)

		# see if we've been here before
		# if not, add it to the graph
		connections = [] # these are current leaves
		if(graph_contain(loc,self.graph)):
			# need to backgrack here
			logthis("been here")
			self.backtracking = True
			# find goal state
			self.curr_goal = get_new_goal(loc, self.center, self.graph)
		else:
			# graph not struct, per node is:
			# [node, [connection, connection]]
			# [(x,y),[(x,y),(x,y)]
			if self.mouse.canGoLeft():
				connections.append([loc[0]-1,loc[1]])
			if self.mouse.canGoRight():
				connections.append([loc[0]+1,loc[1]])
			if self.mouse.canGoDown():
				connections.append([loc[0],loc[1]+1])
			if self.mouse.canGoUp():
				connections.append([loc[0],loc[1]-1])
			# check if we're at a leaf
			if (len(connections) == 0):
				# need to backtrack
				logthis("need to backtrack")
				self.backtracking = True # do i need this?
			else:
				self.graph.append([loc,connections])
				self.oof_path.append(loc)

		# if we're not backtracking then we choose a leaf to move into
		if (not self.backtracking):
			#
			logthis("move into a leaf")
			h_val = 99999
			for leaf in connections:
				if (heuristic_dist(leaf,self.center) < h_val):
					# finds best h val to move into
					h_val = heuristic_dist(leaf,self.center)
					self.curr_goal = leaf
			# find the way to move
			if (loc[0]+1 == self.curr_goal[0]):
				self.mouse.goRight()
			elif (loc[0]-1 == self.curr_goal[0]):
				self.mouse.goLeft()
			elif (loc[1]+1 == self.curr_goal[1]):
				self.mouse.goDown()
			elif (loc[1]-1 == self.curr_goal[1]):
				self.mouse.goUp()
			self.oof_path.append(loc)
		else:
			# we have to check if we are at our goal
			logthis("this is where we move backwards")
			#
			# NOTE: you CANNOT use is_neighbor because when backgracking that doesnt
			# garuntee that there isnt a wall!
			next_node = []
			if(is_connected(loc,self.curr_goal,self.graph)):
				# move into the goal
				logthis("move into goal")
				if (loc[0]+1 == self.curr_goal[0]):
					self.mouse.goRight()
				elif (loc[0]-1 == self.curr_goal[0]):
					self.mouse.goLeft()
				elif (loc[1]+1 == self.curr_goal[1]):
					self.mouse.goDown()
				elif (loc[1]-1 == self.curr_goal[1]):
					self.mouse.goUp()
				# update the oof path
				new_oof = []
				for e in self.oof_path:
					new_oof.append(e)
				for e in reversed(self.temp_oofer_path):
					new_oof.append(e)
				for e in self.temp_oofer_path:
					new_oof.append(e)
				# add the new thing
				self.oof_path = new_oof
				self.oof_path.append(loc)
				self.backgracking = False
			else:
				# move towards the goal with a pop
				next_node = self.oof_path.pop()
				logthis("move into prev")
				if (loc[0]+1 == next_node[0]):
					self.mouse.goRight()
				elif (loc[0]-1 == next_node[0]):
					self.mouse.goLeft()
				elif (loc[1]+1 == next_node[1]):
					self.mouse.goDown()
				elif (loc[1]-1 == next_node[1]):
					self.mouse.goUp()
				self.backgracking = True
				self.temp_oofer_path.append(loc)

		sleep(0.5)

class GreedyRendezvous3(Strategy):
	mouse = None
	isVisited = []
	isBack = False
	network = None
	second = False

	at_goal = False
	curr_goal = [-1,-1]
	# graph not struct, per node is:
	# [node, [connection, connection]]
	# [(x,y),[(x,y),(x,y)]
	graph = [] # a big boi
	oof_path = [] # a stack


	def __init__(self, mouse):
		self.mouse = mouse
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()
		setLog()
		# logthis("init successful!")

	def checkFinished(self):
		return self.isBack

	def go(self):

		logthis("===========>")
		self.mouse.senseWalls()

		# get the xy locations of everyone
		locations = get_all_xy()
		# logthis(locations)
		loc = [self.mouse.x,self.mouse.y]
		# logthis(temp_yee)

		#  get the centroid for movement
		center = get_centroid(locations)
		# logthis(center)

		# determines when to swap giong to goals and things
		# full goal check is done later
		if (self.curr_goal[0] == -1 and self.curr_goal[1] == -1):
			self.curr_goal[0] = loc[0]
			self.curr_goal[1] = loc[1]
			self.oof_path.append(loc)

		if (self.curr_goal[0] == loc[0] and self.curr_goal[1] == loc[1]):
			self.at_goal = True
			# we set the new goal later

		# add to the local graph
		# first see if we've been here before
		# connections is declared here for the greedy implementation
		connections = []
		if (graph_contain(loc,self.graph)):
			# need to continue towards goal
			logthis("been here before")
			# NOTE: This ONLY works for the greedy get path:
			# next_step = self.path_to_goal
			if (is_neighbor(loc,self.curr_goal)):
				next_step = self.curr_goal
				logthis(next_step)
				# NOTE: this part should work regardless
				if (loc[0] < next_step[0]):
					self.mouse.goRight()
				elif (loc[0] > next_step[0]):
					self.mouse.goLeft()
				elif(loc[1] < next_step[1]):
					self.mouse.goDown()
				elif(loc[1] > next_step[1]):
					self.mouse.goUp()
				# logthis(self.curr_goal)
				self.oof_path.append(loc)
			else: # this should only be triggered if a dead end is found
				next_step = self.oof_path.pop()
				self.curr_goal = get_new_goal(loc,center,self.graph)
				self.at_goal = False
				if (loc[0] < next_step[0]):
					self.mouse.goRight()
				elif (loc[0] > next_step[0]):
					self.mouse.goLeft()
				elif(loc[1] < next_step[1]):
					self.mouse.goDown()
				elif(loc[1] > next_step[1]):
					self.mouse.goUp()
		else:
			# graph not struct, per node is:
			# [node, [connection, connection]]
			# [(x,y),[(x,y),(x,y)]
			if self.mouse.canGoLeft():
				connections.append([loc[0]-1,loc[1]])
			if self.mouse.canGoRight():
				connections.append([loc[0]+1,loc[1]])
			if self.mouse.canGoDown():
				connections.append([loc[0],loc[1]+1])
			if self.mouse.canGoUp():
				connections.append([loc[0],loc[1]-1])
			# the actual struct
			if (not graph_contain(loc,self.graph)):
				self.graph.append([loc,connections])

		# set the real goal here
		if (self.at_goal):
			# logthis("setting new goal...")
			self.curr_goal = get_new_goal(loc,center,self.graph)
			# self.curr_goal = greedy_get_step(connections,center,self.graph)
			self.at_goal = False
			logthis("curr_goal: " + str(self.curr_goal))
			# self.path_to_goal = get_path_to_goal(loc,center,connections,self.graph)

		sleep(0.5)

class GreedyRendezvous2(Strategy):
	mouse = None
	isVisited = []
	isBack = False
	network = None
	second = False

	at_goal = False
	curr_goal = [-1,-1]
	# graph not struct, per node is:
	# [node, [connection, connection]]
	# [(x,y),[(x,y),(x,y)]
	graph = [] # a big boi
	path_to_goal = [] # a stack

	def __init__(self, mouse):
		self.mouse = mouse
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()
		setLog()
		# logthis("init successful!")

	def checkFinished(self):
		return self.isBack

	def go(self):

		logthis("===========>")

		# get the xy locations of everyone
		locations = get_all_xy()
		# logthis(locations)
		loc = [self.mouse.x,self.mouse.y]
		# logthis(temp_yee)

		#  get the centroid for movement
		center = get_centroid(locations)
		# logthis(center)

		# determines when to swap giong to goals and things
		# full goal check is done later
		if (self.curr_goal[0] == -1 and self.curr_goal[1] == -1):
			self.curr_goal[0] = loc[0]
			self.curr_goal[1] = loc[1]

		if (self.curr_goal[0] == loc[0] and self.curr_goal[1] == loc[1]):
			self.at_goal = True
			# we set the new goal later

		# add to the local graph
		# first see if we've been here before
		# connections is declared here for the greedy implementation
		connections = []
		if (graph_contain(loc,self.graph)):
			# need to continue towards goal
			logthis("been here before")
			# NOTE: This ONLY works for the greedy get path:
			# next_step = self.path_to_goal
			next_step = self.curr_goal
			logthis(next_step)
			# NOTE: this part should work regardless
			if (loc[0] < next_step[0]):
				self.mouse.goRight()
			elif (loc[0] > next_step[0]):
				self.mouse.goLeft()
			elif(loc[1] < next_step[1]):
				self.mouse.goDown()
			elif(loc[1] > next_step[1]):
				self.mouse.goUp()
			# logthis(self.curr_goal)
		else:
			# graph not struct, per node is:
			# [node, [connection, connection]]
			# [(x,y),[(x,y),(x,y)]
			if self.mouse.canGoLeft():
				connections.append([loc[0]-1,loc[1]])
			if self.mouse.canGoRight():
				connections.append([loc[0]+1,loc[1]])
			if self.mouse.canGoDown():
				connections.append([loc[0],loc[1]+1])
			if self.mouse.canGoUp():
				connections.append([loc[0],loc[1]-1])
			# the actual struct
			self.graph.append([loc,connections])

		# set the real goal here
		if (self.at_goal):
			# logthis("setting new goal...")
			# self.curr_goal = get_new_goal(loc,center,self.graph)
			self.curr_goal = greedy_get_step(connections,center,self.graph)
			self.at_goal = False
			logthis("curr_goal: " + str(self.curr_goal))
			# self.path_to_goal = get_path_to_goal(loc,center,connections,self.graph)

		sleep(0.5)

class GreedyRendezvous1(Strategy):
	mouse = None
	isVisited = []
	path = []
	isBack = False
	network = None
	second = False

	def __init__(self, mouse):
		self.mouse = mouse
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()
		self.network.startReceiveThread()
		setLog()
		logthis("init successful!")

	def checkFinished(self):
		return self.isBack

	def go(self):

		# get the xy locations of everyone
		locations = get_all_xy()
		logthis(locations)
		temp_yee = [self.mouse.x,self.mouse.y]
		logthis(temp_yee)

		#  get the centroid for movement
		center = get_centroid(locations)
		logthis(center)

		#
		# +1 | should move positively in that direction
		#  0 | don't move at all in that direction
		# -1 | should move negativley in that direction
		# priority | if x or y should be the way to go first if a tie
		# 1 = x, 2 = y, 0 = doesn't matter
		# initialize:
		should_move_x = 0
		should_move_y = 0
		priority = 0

		#  determine ideal x movment
		if (self.mouse.x > center[0]):
			should_move_x = -1 # move left
		elif (self.mouse.x < center[0]):
			should_move_x = 1 # move right
		else:
			should_move_x = 0

		#  determine ideal y movment
		if (self.mouse.y > center[1]):
			should_move_y = 1 # move down
		elif (self.mouse.y < center[1]):
			should_move_y = -1 # move up
		else:
			should_move_y = 0

		# find the way to move if we can't move the way we want
		mag_x = abs(self.mouse.x - center[0])
		mag_y = abs(self.mouse.y - center[1])
		if (mag_x > mag_y):
			priority = 1
		elif (mag_y > mag_x):
			priority = 2
		else:
			priority = 0

		self.mouse.senseWalls()

		# note we always have the option to do nothing
		moved = False
		if ((should_move_x == 1) and self.mouse.canGoRight() and not moved):
			self.mouse.goRight()
			self.isVisited[self.mouse.x+1][self.mouse.y]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True
		if ((should_move_x == -1) and self.mouse.canGoLeft() and not moved):
			self.mouse.goLeft()
			self.isVisited[self.mouse.x-1][self.mouse.y]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True
		if ((should_move_y == 1) and self.mouse.canGoUp() and not moved):
			self.mouse.goUp()
			self.isVisited[self.mouse.x][self.mouse.y-1]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True
		if ((should_move_y == -1) and self.mouse.canGoDown() and not moved):
			self.mouse.goDown()
			self.isVisited[self.mouse.x][self.mouse.y+1]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True

		# at this point we cannot do anything clear
		# move to the closest unvisited cell if possible
		if (self.mouse.canGoRight() and self.isVisited[self.mouse.x+1][self.mouse.y] and not moved):
			self.mouse.goRight()
			self.isVisited[self.mouse.x+1][self.mouse.y]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True
		elif (self.mouse.canGoLeft() and self.isVisited[self.mouse.x-1][self.mouse.y] and not moved):
			self.mouse.goLeft()
			self.isVisited[self.mouse.x-1][self.mouse.y]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True
		elif (self.mouse.canGoDown() and self.isVisited[self.mouse.x][self.mouse.y+1] and not moved):
			self.mouse.goDown()
			self.isVisited[self.mouse.x][self.mouse.y+1]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True
		elif (self.mouse.canGoUp() and self.isVisited[self.mouse.x][self.mouse.y-1] and not moved):
			self.mouse.goUp()
			self.isVisited[self.mouse.x][self.mouse.y-1]
			# self.path.append([self.mouse.x,self.mouse.y])
			moved = True

		# logthis(self.path)
		sleep(0.5)

class StrategyTestDFSEV3(Strategy):
	mouse = None
	#mapPainter = None
	isVisited = []
	path = []
	isBack = False

	def __init__(self, mouse):
		self.mouse = mouse
		#self.mapPainter = mapPainter
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1

	def checkFinished(self):
		return self.isBack

	def go(self):
		#cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
		#self.mapPainter.drawCell(cell, 'grey')
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())

		if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x-1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x-1][self.mouse.y] = 1
			self.mouse.goLeft()
		elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y-1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y-1] = 1
			self.mouse.goUp()
		elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x+1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x+1][self.mouse.y] = 1
			self.mouse.goRight()
		elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y+1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y+1] = 1
			self.mouse.goDown()
		else:
			if len(self.path) != 0:
				x, y = self.path.pop()
				if x < self.mouse.x:
					self.mouse.goLeft()
				elif x > self.mouse.x:
					self.mouse.goRight()
				elif y < self.mouse.y:
					self.mouse.goUp()
				elif y > self.mouse.y:
					self.mouse.goDown()
			else:
				self.isBack = True

		#cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
		#self.mapPainter.putRobotInCell(cell)

class StrategyTestGoStepEV3(Strategy):
	mouse = None
	progress = 0

	def __init__(self, mouse):
		self.mouse = mouse

	def checkFinished(self):
		return self.progress >= 1

	def go(self):
		self.progress = self.progress + 1
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		self.mouse.goLeft()
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		self.mouse.goRight()
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		self.mouse.goUp()
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		self.mouse.goDown()
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		sleep(1)

class StrategyTestInitEV3(Strategy):
	mouse = None
	flag = False

	def __init__(self, mouse):
		self.mouse = mouse

	def checkFinished(self):
		return self.flag

	def go(self):
		self.mouse.commandTranslator.motorController.gyreset()
		self.flag = True
		sleep(1)

class StrategyTestDFSDisplayEV3(Strategy):
	mouse = None
	isVisited = []
	path = []
	isBack = False

	def __init__(self, mouse):
		self.mouse = mouse
		self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
		self.isVisited[self.mouse.x][self.mouse.y] = 1
		self.network = NetworkInterface()
		self.network.initSocket()

	def checkFinished(self):
		return self.isBack

	def go(self):
		self.mouse.senseWalls()
		print(self.mouse.getCurrentCell().getWhichIsWall())
		sendData = {'x': self.mouse.x, 'y':self.mouse.y, 'up':self.mouse.canGoUp(), 'down':self.mouse.canGoDown(), 'left':self.mouse.canGoLeft(), 'right':self.mouse.canGoRight()}
		self.network.sendStringData(sendData)

		if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x-1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x-1][self.mouse.y] = 1
			self.mouse.goLeft()
		elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y-1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y-1] = 1
			self.mouse.goUp()
		elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x+1][self.mouse.y]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x+1][self.mouse.y] = 1
			self.mouse.goRight()
		elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y+1]:
			self.path.append([self.mouse.x, self.mouse.y])
			self.isVisited[self.mouse.x][self.mouse.y+1] = 1
			self.mouse.goDown()
		else:
			if len(self.path) != 0:
				x, y = self.path.pop()
				if x < self.mouse.x:
					self.mouse.goLeft()
				elif x > self.mouse.x:
					self.mouse.goRight()
				elif y < self.mouse.y:
					self.mouse.goUp()
				elif y > self.mouse.y:
					self.mouse.goDown()
			else:
				self.isBack = True
