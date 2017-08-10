import random
from utils import *
import copy
from sets import Set

class Navigator(object):
	def __init__(self):
		''' Initialization 
		'''
		self.heading_map = {0: -90, 1:0, 2:90}
		self.move_map = {"l":(-1,0),"u":(0,1),"r":(1,0),"d":(0,-1), "left":(-1,0),"up":(0,1),"right":(1,0),"down":(0,-1),
		0:(-1,0),1:(0,1),2:(1,0),3:(0,-1)}
		self.dir_sensors = {"up":["l","u","r"], "right":["u","r","d"],"down":["r","d","l"], 
							"left":["d","l","r"], "u":["l","u","r"], "r":["u","r","d"], "d":["r","d","l"], "l":["d","l","u"]}
		self.idx_heading = {0:"l", 1:"u", 2:"r",3:"d"}
		self.dir_map = {"left":0, "up":1, "right":2, "down":3,
		"l":0, "u":1, "r":2, "d":3}
		# store planned movements
		self.steps = []


	def random_search(self, sensors):
		''' Random search
		'''
		headings = []
		for i in range(len(sensors)):
			if (sensors[i] > 0):
				headings.append(i)
		if (len(headings) == 0):
			return self.moveEnd()
		heading = random.choice(headings)
		return (self.heading_map[heading], 1)

	def counter_search(self, location, heading, sensors, mapper):
		''' Counter search, combine reandom search with deanend and counter maps
		'''
		if (not mapper.goal_found):
			# check if current location goal or not
			mapper.goal_found = mapper.check_goal(location)
		if (mapper.goal_found and mapper.area_visited() > 0.50):
			# if goal found and visited portion larger than threshold, then using A* search for optimal path and transfer it into moves
			path_close, cost_close = self.find_optimal_path(mapper, False, [0,0])
			paths = self.find_all_paths(cost_close, mapper)
			best_move = None
			min_move = 10000
			best_path = None
			for path in paths:
				del path[0]
				optimal_moves = self.path_to_move([0,0], 'u', path)
				if (len(optimal_moves) < min_move):
					min_move = len(optimal_moves)
					best_move = optimal_moves
					best_path = path
			self.steps = best_move
			# some information display.
			print("optimal {}").format(best_move)
			print(min_move)
			mapper.draw_optimal_map(mapper.maze_dim, best_path)
			return ('Reset','Reset')
		new_location = location
		new_heading = heading
		if (mapper.deadend[new_location[0]][new_location[1]]==1):
			# get rid of deadend
			back_heading = mapper.parent_heading[new_location[0]][new_location[1]]
			cur_heading = mapper.dir_map[heading]
			count = 0
			if (abs(cur_heading-back_heading)==2):
				while(count < 3 and mapper.deadend[new_location[0]][new_location[1]]==1 and abs(cur_heading-back_heading)==2):
					count+=1
					new_x = new_location[0]+mapper.move_map[back_heading][0]
					new_y = new_location[1]+mapper.move_map[back_heading][1]
					new_location = [new_x, new_y]
					back_heading = mapper.parent_heading[new_location[0]][new_location[1]]
				return (0, -count)
			else:
				# calculate movement
				rotation = self.calculate_rotation(cur_heading, back_heading)
				cur_heading = back_heading
				while (count < 3 and mapper.deadend[new_location[0]][new_location[1]]==1 and abs(cur_heading-back_heading)==0):
					count+=1
					new_x = new_location[0]+mapper.move_map[back_heading][0]
					new_y = new_location[1]+mapper.move_map[back_heading][1]
					new_location = [new_x, new_y]
				return (rotation, count)

		headings = []
		counts = []
		min_count = 10000
		# update maze information
		for i in range(len(sensors)):
			if (sensors[i] > 0):
				abs_heading = self.dir_sensors[heading][i]
				new_x = location[0] + self.move_map[abs_heading][0]
				new_y = location[1] + self.move_map[abs_heading][1]
				if (mapper.deadend[new_x][new_y]!=1):
					cur_count = mapper.counter[new_x][new_y]
					if (cur_count < min_count):
						min_count = cur_count
						headings = [i]
					elif (cur_count == min_count):
						headings.append(i)
		if (len(headings)==0):
			return self.moveEnd()

		heading = random.choice(headings)
		return (self.heading_map[heading], 1)

	def target_search(self, location, heading, mapper):
		if (not mapper.goal_found):
			# check goal visited
			mapper.goal_found = mapper.check_goal(location)
		if mapper.goal_found:
			# compare path open with path close to determine whether stop the first run or not
			path_open, cost_open = self.find_optimal_path(mapper, True, [0,0])
			path_close, cost_close = self.find_optimal_path(mapper, False, [0,0])
			# print("============")
			if (len(path_open) == len(path_close)):
				# we can stop the first run, calculate actual movements
				paths = self.find_all_paths(cost_open, mapper)
				best_move = None
				min_move = 10000
				best_path = None
				for path in paths:
					del path[0]
					optimal_moves = self.path_to_move([0,0], 'u', path)
					if (len(optimal_moves) < min_move):
						min_move = len(optimal_moves)
						best_move = optimal_moves
						best_path = path
				self.steps = best_move
				print("optimal {}").format(best_move)
				print("move length {}").format(min_move)
				mapper.draw_optimal_map(mapper.maze_dim, best_path)
				return('Reset','Reset')

		# calculate next movement
		if (len(self.steps) > 0):
			# if still have steps not carried out, use those steps
			step = self.steps[0]
			valid_movement = self.valid_step(location, heading, mapper, step)
			# check the calidation of the movement
			if (valid_movement == 0):
				move = self.find_new_move(location, heading, mapper)
				return move
			elif (valid_movement < step[1]):
				# print("step: {}").format((self.steps[0][0], valid_movement))
				rotaion = self.steps[0][0]
				self.steps = []
				return (rotaion, valid_movement)
			else:
				# print("step: {}").format(self.steps[0])
				del self.steps[0]
				return step
		else:
			# calculate new target and paths
			move = self.find_new_move(location, heading, mapper)
			return move

	def find_all_paths(self, cost, mapper):
		''' Given cost map, calculate all possible valid paths that has the same path length
		'''
		res_path = []
		ini_path = []
		pos = tuple(mapper.goal_loc)
		ini_path.insert(0,pos)
		res_path.append(ini_path)
		# result paths list
		total_cost = cost[pos]
		while(total_cost > 0):
			total_cost -= 1
			new_res_path = []
			for path in res_path:
				# iterate to get all possible paths from goal to start location using the cost map
				loc = path[0]
				for i in range(4):
					move = self.move_map[i]
					new_x = loc[0]+move[0]
					new_y = loc[1]+move[1]
					if (new_x >= 0 and new_x < mapper.maze_dim and new_y >= 0 and new_y < mapper.maze_dim 
						and (new_x, new_y) in cost and cost[(new_x, new_y)] == total_cost and self.valid_move(loc, (new_x, new_y),mapper)):
						path.insert(0, (new_x, new_y))
						n_path = copy.deepcopy(path)
						new_res_path.append(n_path)
						del path[0]
			res_path = new_res_path
		return res_path

	def find_optimal_path(self, mapper, is_open, start):
		''' A* search implementation using priority queue, is_open can be used to calculate open path and close path seperately
		'''
		frontier = PriorityQueue()
		frontier.put(tuple(start), 0)
		parent = {}
		cost = {}
		parent[tuple(start)] = None
		cost[tuple(start)] = 0
		while not frontier.empty():
			cur = frontier.get()
			if (cur == mapper.goal_loc):
				break
			for i in range(4):
				if (self.is_valid(cur,i, is_open, mapper)):
					move = self.move_map[i]
					new_x = cur[0]+move[0]
					new_y = cur[1]+move[1]
					new_loc = (new_x, new_y)
					new_cost = cost[cur]+1
					if (new_loc not in cost or new_cost < cost[new_loc]):
						cost[new_loc] = new_cost
						priority = new_cost + self.heuristic(new_loc, mapper.goal_loc)
						frontier.put(new_loc, priority)
						parent[new_loc] = cur
		res_path = []
		pos = tuple(mapper.goal_loc)
		# if (is_open):
		# 	print("goal {}".format(pos))
		while(pos!=tuple(start)):
			res_path.insert(0, [pos[0],pos[1]])
			pos = parent[pos]
		return res_path, cost

	def is_valid(self, location, heading_idx, is_open, mapper):
		''' check the movement validity
		'''
		move = self.move_map[heading_idx]
		new_x = location[0]+move[0]
		new_y = location[1]+move[1]
		if (new_x < 0 or new_x >= mapper.maze_dim or new_y < 0 or new_y >= mapper.maze_dim):
			return False
		if is_open:
			if (mapper.walls[location[0]][location[1]][heading_idx])==1:
				return False
			return True
		else:
			if (mapper.walls[location[0]][location[1]][heading_idx])==1 or (mapper.walls[location[0]][location[1]][heading_idx]==-1):
				return False
			return True

	def heuristic(self, loc, target):
		''' Mahanttan distance heuristic
		'''
		return abs(loc[0]-target[0])+abs(loc[1]-target[1])

	def find_new_move(self, location, heading, mapper):
		''' Find new target use uncertainty map and return correspoding path assume open edge
		'''
		moves = []
		# print("location {}").format(location)
		target = mapper.find_target(location)
		# print("target {}").format(target)
		paths = mapper.find_path(location, target)
		# print("paths {}".format(paths))
		if (paths == None):
			raise Exception('Something went wrong!')
		moves = self.path_to_move(location, heading, paths)
		# print("moves {}").format(moves)
		move = moves[0]
		if (len(moves) > 1):
			del moves[0]
			self.steps = moves
		return move

	def valid_step(self, location, heading, mapper, step): 
		''' check if step made is valid or not
		'''
		max_movement = step[1]
		heading_idx = self.dir_map[heading]
		if (step[0] == 90):
			heading_idx = (heading_idx+1)%4
		elif (step[0] == -90):
			heading_idx = (heading_idx-1)%4
		elif (max_movement < 0):
			heading_idx = (heading_idx+2)%4
		move = self.move_map[self.idx_heading[heading_idx]]
		i = 0
		# print(heading_idx)
		while (i < abs(max_movement)):
			new_x = location[0]+i*move[0]
			new_y = location[1]+i*move[1]
			if (new_x < 0 or new_x >= mapper.maze_dim or new_y < 0 or new_y >= mapper.maze_dim):
				return i
			if (mapper.walls[new_x][new_y][heading_idx])==1:
				return i
			i+=1
		return i

	def moveEnd(self):
		''' turn left or right when meet end
		'''
		return (self.heading_map[random.choice([0,2])], 0)

	def calculate_rotation(self, heading, new_heading):
		''' Calculate actual rotation
		'''
		if (heading+1)%4 == new_heading:
			return 90
		elif (heading-1)%4 == new_heading:
			return -90
		elif (heading==new_heading):
			return 0
		else:
			raise Exception('Something went wrong!')

	def path_to_move(self, location, heading, path):
		''' transfer path to actual movements
		'''
		one_move = []
		path.insert(0, location)
		idx_heading = self.dir_map[heading]
		goback = False
		rotaion = 0 
		for i in range(len(path)-1):
			goback = False
			com_heading = self.cal_rotation(path[i], path[i+1])
			idx_com_heading = self.dir_map[com_heading]
			movement = 1
			if (idx_heading == idx_com_heading):
				rotaion = 0
			elif ((idx_heading-idx_com_heading)%4 == 1):
				rotaion = -90
			elif ((idx_heading-idx_com_heading)%4 == 3):
				rotaion = 90
			else:
				goback = True
				movement = -1
			one_move.append((rotaion, movement))
			if goback:
				idx_heading = idx_heading
			else:
				idx_heading = idx_com_heading
		res = []
		i = 0
		while i < len(one_move)-1:
			movement = one_move[i][1]
			rotaion = one_move[i][0]
			while i < len(one_move)-1 and (one_move[i+1][0]==0) and abs(movement)< 3:
				movement += one_move[i][1]
				i += 1
			res.append((rotaion, movement))
			i+= 1
		if (i == len(one_move)-1):
			res.append(one_move[len(one_move)-1])
		# print(one_move)
		return res

	def valid_move(self, loc1, loc2, mapper):
		direction = self.cal_rotation(loc1, loc2)
		return mapper.walls[loc1[0]][loc1[1]][self.dir_map[direction]] == 0



	def cal_rotation(self, loc1, loc2):
		''' Given two locations, calculate actual heading change
		'''
		del_x = loc2[0]-loc1[0]
		del_y = loc2[1]-loc1[1]
		if (del_x == 0 and del_y == 1):
			return "u"
		elif (del_x == 0 and del_y == -1):
			return "d"
		elif (del_x == 1 and del_y == 0):
			return "r"
		else:
			return "l"

