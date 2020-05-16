from math import *
import numpy as numpy


class Node:

    def __init__(self, gameState, value, parent, parentAction, depth):
        self.gameState = gameState
        self.value = value
        self.parent = parent
        self.parentAction = parentAction
        self.depth = depth
        self.child = []

class MiniMax:

    def __init__(self, gameState, value, depth):
        initialNode = Node(gameState, value, None, None, depth)
        self.tree = [[initialNode]]
        for _ in range(depth - 1):
            self.tree.append([])
        self.total_depth = depth
    
    def ChooseBestAction(self):
        total_maxmin = (self.total_depth - 1) // 2 
        for _ in range(total_maxmin, 0, -1):
            self.MinPart(2*total_maxmin - 1)
            self.MaxPart(2*total_maxmin - 2)
        for node in self.tree[1]:
            if node.value == self.tree[0][0].value:
                return node.parentAction
        

    def MaxPart(self, depth):
        for node in self.tree[depth]:
            values = []
            for childIndex in node.child:
                values.append(self.tree[depth + 1][childIndex].value)
            node.value = max(values)
    
    def MinPart(self, depth):
        for node in self.tree[depth]:
            values = []
            for childIndex in node.child:
                values.append(self.tree[depth + 1][childIndex].value)
            node.value = min(values)



    
    
