import numpy as np
from mapper import *
from Navigator import *
class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.maze = Mapper(maze_dim)
        self.idx_dir_map = {0:"l", 1:"u",2:"r", 3:"d"}
        self.dir_idx_map = {"left":0, "up":1,"right":2, "down":3,
        "l":0, "u":1, "r":2, "d":3}
        self.navigator = Navigator()
        self.second = False

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        if self.second:
            # if in the second run, carry out steps in the store optimal moves
            (rotation, movement) = self.navigator.steps[0]
            del self.navigator.steps[0]
            return rotation, movement
        # default use the target search
        (rotation, movement) = self.navigator.target_search(self.location, self.heading, self.maze)
        # (rotation, movement) = self.navigator.counter_search(self.location, self.heading, sensors, self.maze)
        if (rotation == 0 and movement == 0):
            raise Exception('Something went wrong!')
        if (rotation=='Reset' and movement == 'Reset'):
            self.second = True
            return rotation, movement

        # update robot location and heading information
        idx = self.dir_idx_map[self.heading]
        if (rotation == -90):
            new_idx = (idx-1)%4
            self.heading = self.idx_dir_map[new_idx]
        if (rotation == 0):
            new_idx = (idx-0)%4
            self.heading = self.idx_dir_map[new_idx]
        if (rotation == 90):
            new_idx = (idx+1)%4
            self.heading = self.idx_dir_map[new_idx]


        if (self.heading=="left" or self.heading=="l" ):
            new_x = self.location[0]-movement
            new_y = self.location[1]
        if (self.heading=="up" or self.heading=="u"):
            new_x = self.location[0]
            new_y = self.location[1]+movement
        if (self.heading=="right" or self.heading=="r"):
            new_x = self.location[0]+movement
            new_y = self.location[1]
        if (self.heading=="down" or self.heading=="d"):
            new_x = self.location[0]
            new_y = self.location[1]-movement
        self.location = [new_x, new_y]


        return rotation, movement