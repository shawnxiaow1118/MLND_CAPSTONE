#-*- coding: UTF-8 -*-
import Queue
class Mapper(object):
    ''' used for maintaining the information of the mao known so far
    '''
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.walls = self.initWalls(self.maze_dim)
        self.dir_sensors = {"up":["l","u","r"], "right":["u","r","d"], 
                    "down":["r","d","l"], "left":["d","l","r"], 
                    "u":["l","u","r"], "r":["u","r","d"], 
                    "d":["r","d","l"], "l":["d","l","u"]}
        self.dir_map = {"left":0, "up":1, "right":2, "down":3,
        "l":0, "u":1, "r":2, "d":3}
        self.idx_dir_map = {0:"l", 1: "u", 2:"r", 3:"d"}
        # move map calculate next step
        self.move_map = {0:(-1,0),1:(0,1),2:(1,0),3:(0,-1)}
        self.counter = [[0 for _ in range(maze_dim)] for _ in range(maze_dim)]
        self.deadend = [[0 for _ in range(maze_dim)] for _ in range(maze_dim)]
        self.parent_heading = [[-1 for _ in range(maze_dim)] for _ in range(maze_dim)] 
        self.total_visited = 0
        self.uncertainties = self.initUncern(maze_dim)
        self.heading_icon = {"u":"^ ", "l":"< ", "r":" >", "d":"v ",
        "up":"^ ", "left":"< ", "right":" >", "down":"v "}
        self.goal_found = False
        self.goal_loc = []
        self.heuristic = [[1000 for _ in range(maze_dim)] for _ in range(maze_dim)]

    def initUncern(self, maze_dim):
        ''' initialize the uncertainty of the maze given maze dimension
            the uncertainty just count the number of unknown directions in that cell
            the corner cell only has 2 knowns, and the edge cell has 3, others has 4
        '''
    	uncertainties = [[4 for _ in range(maze_dim)] for _ in range(maze_dim)]
        for i in range(maze_dim):
            uncertainties[0][i] = 3
            uncertainties[i][0] = 3
            uncertainties[i][maze_dim-1] = 3
            uncertainties[maze_dim-1][i] = 3
        uncertainties[0][0] = 2
        uncertainties[0][maze_dim-1] = 2
        uncertainties[maze_dim-1][0] = 2
        uncertainties[maze_dim-1][maze_dim-1] = 2
        # uncertainties[maze_dim/2][maze_dim/2] = 2
        # uncertainties[maze_dim/2-1][maze_dim/2-1] = 2
        # uncertainties[maze_dim/2][maze_dim/2-1] = 2
        # uncertainties[maze_dim/2-1][maze_dim/2] = 2
        return uncertainties

    def initWalls(self, maze_dim):
        ''' 0: left, 1: up, 2: right, 3: bottom, initialize the wall map
            all edges between walls are unknow (-1), except for those lie in the out edge of the entire maze
        '''
        walls = [[[-1]*4 for _ in range(maze_dim)] for _ in range(maze_dim)]
        # handle border cases
        for i in range(maze_dim):
            walls[i][0][3] = 1
            walls[0][i][0] = 1
            walls[maze_dim-1][i][2] = 1
            walls[i][maze_dim-1][1] = 1 
        return walls



    def draw_map(self, maze_dim, location, heading):
        ''' Draw the map, "——" for known horizontal wall, ".." for unkonwn horizontal edge
            "|" for known vertical wall, ":" for vertical unknown edge space means connected
        '''
        hmap = {-1:"..", 1:"——", 0:"  "}
        vmap = {-1:":", 1:"|", 0: " "}
        # first line
        for j in range(maze_dim):
            index = maze_dim-1-j
            if(j==0):
                for i in range(maze_dim):
                    if (i==0):
                        print("*"),
                    print(hmap[self.walls[i][index][1]]),
                    print("*"),
                print("")

            for i in range(maze_dim):
                if (i==0):
                    print(vmap[self.walls[i][index][0]]),
                if (i==location[0] and index == location[1]):
                    print(self.heading_icon[heading]),
                else:
                    print("  "),
                print(vmap[self.walls[i][index][2]]),
            print("")
            for i in range(maze_dim):
                if (i==0):
                    print("*"),
                print(hmap[self.walls[i][index][3]]),
                print("*"),
            print("")

    def update_walls(self, location, sensors, heading):
        ''' update wall information
        '''
        if (self.counter[location[0]][location[1]] == 0):
        	self.total_visited += 1
        self.counter[location[0]][location[1]] += 1
        for i in range(len(sensors)):
            abs_heading = self.dir_sensors[heading][i]
            if (sensors[i] == 0):
                self.set_wall_value(location, abs_heading, 1)
            else:
            	neighbor = location
                for i in range(sensors[i]):
                    self.set_wall_value(neighbor, abs_heading, 0)
                    neighbor = self.find_neighbors(neighbor, abs_heading)
                self.set_wall_value(neighbor, abs_heading, 1)
        # update deadend and uncertainty map
        self.update_deadend(location, sensors, heading)
        self.update_uncern(location, sensors, heading)

    def set_wall_value(self, location, heading, value):
        ''' set wall value in wall map
        ''' 
        x,y = location
        self.walls[x][y][self.dir_map[heading]] = value
        neighbor = self.find_neighbors(location, heading)
        if (neighbor != None):
            if (self.dir_map[heading]+2 > 3):
                self.walls[neighbor[0]][neighbor[1]][(self.dir_map[heading]+2)%2] = value
            else:
                self.walls[neighbor[0]][neighbor[1]][self.dir_map[heading]+2] = value

    def find_neighbors(self, location, heading):
        ''' find neighbors
        '''
        x,y = location
        if heading == "up" or heading == "u" and y!=self.maze_dim-1:
            return [x, y+1]
        if heading == "left" or heading == "l" and x!=0:
            return [x-1, y]
        if heading == "down" or heading == "d" and y!=0:
        	return [x, y-1]
        if heading == "right" or heading == "r" and x!=self.maze_dim-1:
        	return [x+1, y]
        else:
        	return None
    
    def update_deadend(self, location, sensors, heading):
        ''' update deadend in the end map given sensors
        '''
        if (max(sensors)==0):
            heading = self.dir_map[heading]
            opens = self.find_real_opens(location)
            while(len(opens)<=1):
                self.deadend[location[0]][location[1]] = 1
                back_heading = -1
                for i in range(len(opens)):
                    back_heading = opens[i]
                if back_heading==-1:
                    return 
                new_x = location[0]+self.move_map[back_heading][0]
                new_y = location[1]+self.move_map[back_heading][1]
                self.parent_heading[location[0]][location[1]] = back_heading
                location = [new_x, new_y]
                opens = self.find_real_opens(location)

    def update_uncern(self, location, sensors, heading):
        ''' update uncertainty map given sensors
        '''
        for i in range(3):
            abs_heading = self.dir_sensors[heading][i]
            move = self.move_map[self.dir_map[abs_heading]]
            for j in range(sensors[i]+2):
                new_x = location[0]+j*move[0]
                new_y = location[1]+j*move[1]
                if (new_x >= 0 and new_y >= 0 and new_x < self.maze_dim and new_y < self.maze_dim):
                    self.uncertainties[new_x][new_y] = self.count_unknown((new_x, new_y))
                    if (self.uncertainties[new_x][new_y]==1):
                        if self.cal_total([new_x,new_y]) == 2:
                            self.update_unknown([new_x,new_y])


    def update_unknown(self, location):
        ''' set unknow edge
        '''
        for i in range(4):
            if self.walls[location[0]][location[1]][i]==-1:
                self.set_wall_value(location, self.idx_dir_map[i], 0)

    def cal_total(self, location):
        ''' calculate a sum statistic for the location
        '''
        res = 0
        for i in range(4):
            res += self.walls[location[0]][location[1]][i]
        return res

    def count_unknown(self, location):
        ''' count unknown edges for the location
        '''
        count = 0
        for i in range(4):
            if (self.walls[location[0]][location[1]][i] == -1):
                count+=1
        return count

    def find_target(self, location):
        ''' find the location with largest uncertainty and is nearest to the current location
        '''
        max_uncern = max([max(row) for row in self.uncertainties])
        possible_locs = []
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if (self.uncertainties[i][j] == max_uncern):
                    possible_locs.append([i,j])

        res = possible_locs[0]
        for loc in possible_locs:
            if (self.cal_man_dist(loc, location) < self.cal_man_dist(res, location)):
                res = loc
        return res


    def cal_man_dist(self, location, target):
        ''' calcualte distance
        '''
        return abs(location[0]-target[0])+abs(location[1]-target[1])

    def find_real_opens(self, location):
        opens = []
        for i in range(4):
            new_x = location[0]+self.move_map[i][0]
            new_y = location[1]+self.move_map[i][1] 
            if (new_x >= 0 and new_x < self.maze_dim and new_y >= 0 and new_y < self.maze_dim 
                and self.walls[location[0]][location[1]][i] == 0 and self.deadend[new_x][new_y]!=1):
                opens.append(i)
        return opens


    def check_goal(self, location):
        ''' check current location goal or not
        '''
        row_max = self.maze_dim/2
        row_min = self.maze_dim/2-1
        # print("'------")
        # print(location)
        res = (location[0] >= row_min and location[0]<=row_max and location[1]>=row_min and location[1] <= row_max)
        if (not self.goal_found and res):
            self.goal_found = True
            self.goal_loc = location
        return res

    def area_visited(self):
    	return self.total_visited*1.0/(self.maze_dim*self.maze_dim)

    def find_path(self, location, target):
        ''' find path between two locations given the maze know so far
        '''
        q = Queue.Queue()
        parent = {}
        new_location = (location[0],location[1])
        new_target = (target[0],target[1])
        q.put(new_location)
        parent[new_location] = new_location
        idx = 0
        # algorithm used is Dijkstra
        while not q.empty():
            idx+=1
            loc = q.get()
            for i in range(4):
                if (self.is_valid(loc, i)):
                    move = self.move_map[i]
                    new_x = loc[0]+move[0]
                    new_y = loc[1]+move[1]
                    if ((new_x,new_y) not in parent.keys()):
                        q.put((new_x, new_y))
                        parent[(new_x, new_y)] = loc
                    if (new_target == (new_x, new_y)):
                        res_path = []
                        pos = new_target
                        while(pos!=new_location):
                            res_path.insert(0, [pos[0],pos[1]])
                            pos = parent[pos]
                        return res_path

    def is_valid(self, location, heading_idx):
        ''' checker method
        '''
        move = self.move_map[heading_idx]
        new_x = location[0]+move[0]
        new_y = location[1]+move[1]
        if (new_x < 0 or new_x >= self.maze_dim or new_y < 0 or new_y >= self.maze_dim):
            return False
        if (self.walls[location[0]][location[1]][heading_idx])==1:
            return False
        return True


    def draw_optimal_map(self, maze_dim, paths):
        ''' draw maze with path 
        '''
        print(paths)
        paths.insert(0,(0,0))
        hmap = {-1:"..", 1:"——", 0:"  "}
        vmap = {-1:":", 1:"|", 0: " "}
        # first line
        for j in range(maze_dim):
            index = maze_dim-1-j
            if(j==0):
                for i in range(maze_dim):
                    if (i==0):
                        print("*"),
                    print(hmap[self.walls[i][index][1]]),
                    print("*"),
                print("")

            for i in range(maze_dim):
                if (i==0):
                    print(vmap[self.walls[i][index][0]]),
                #if (i==location[0] and index == location[1]):
                    # print(self.heading_icon[heading]),
                if((i,index) in paths):
                    print("OO"),
                else:
                    print("  "),
                print(vmap[self.walls[i][index][2]]),
            print("")
            for i in range(maze_dim):
                if (i==0):
                    print("*"),
                print(hmap[self.walls[i][index][3]]),
                print("*"),
            print("")
