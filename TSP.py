
# Author Name: Jieying Lin

import sys
from collections import defaultdict

# Use Kruskal's algorithm to find
# Minimum Spanning Tree 
# Resources: https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/

# construct a graph
class Graph:

    def __init__(self, vertices):
        self.V = vertices
        self.graph = []

    def addEdge(self, u, v, w):
        self.graph.append([u, v, w])

    # A utility function to find set of an element i
    # (uses path compression technique)
    def find(self, parent, i):
        if parent[i] == i:  # if parent is itself
            return i
        return self.find(parent, parent[i])  # find the root of this subtree

    # A function that does union of two sets of x and y
    # (uses union by rank)
    def union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)

        # Attach smaller rank tree under root of
        # high rank tree (Union by Rank)
        # so that the attach tree can find the root easily
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot

        # If ranks are same, then make one as root
        # and increment its rank by one
        else:
            parent[yroot] = xroot
            rank[xroot] += 1

    # The main function to construct MST using Kruskal's
    # algorithm

    def KruskalMST(self):
        # EdgeSum = 0

        result = []

        i = 0  # Sorted edges index
        EdgeNum = 0  # An index variable, used for result[]#fjdkjlk

        # Sort by edges
        self.graph = sorted(self.graph, key=lambda item: item[2])
        parent = []
        rank = []

        # Create Vertex subsets with single elements
        # So the parent is itself
        for node in range(self.V):
            parent.append(node)
            rank.append(0)

        # we need to take V-1 of edge in total
        while EdgeNum < self.V - 1:

            # Pick the smallest edge
            u, v, w = self.graph[i]
            i = i + 1

            # Find the root of these two vertices belong to
            RootU = self.find(parent, u)
            RootV = self.find(parent, v)

            if RootU != RootV:
                EdgeNum = EdgeNum + 1
                result.append([u, v, w])
                self.union(parent, rank, RootU, RootV)
            # Else discard the edge
        EdgeSum = 0
        for u, v, weight in result:
            EdgeSum += weight

        return EdgeSum

# Read file and create the graph
NumOfNodes = 0 # store total visited nodes
cities = []
cityDictionary = {}
f = open(sys.argv[1], "r")
lines = f.readlines()
TotalVertice = int(lines[0])
for i in range(1, len(lines)):
    city, x, y = (val for val in lines[i].split())
    x = int(x)
    y = int(y)
    cities.append((city, x, y))
    cityDictionary[city] = i - 1
f.close()

# Compute edges between each vertices
edgeMatrix = [[0 for x in range(TotalVertice)] for y in range(TotalVertice)]
for i in range(0, len(cities)):
    for j in range(0, len(cities)):
        if i == j:
            continue
        else:
            x1 = cities[i][1]
            y1 = cities[i][2]
            x2 = cities[j][1]
            y2 = cities[j][2]
            distance = ((x1 - x2)**2 + (y1 - y2)**2) ** (0.5)
            edgeMatrix[i][j] = distance
            edgeMatrix[j][i] = distance

currState = (['A'], 0, 0) #path, already visited, heuristic
frontier = []
while len(currState[0]) != TotalVertice:
    UnVisited = []  # a list of unvisited nodes and their heuristic
    # create a list of unvisited cities
    for city in cities:
        if city[0] in currState[0]:
            continue
        else:
            UnVisited.append(city[0])
    # construct a minimun spanning tree
    g = Graph(len(UnVisited))
    minReturn = float('inf')
    minLastToTree = float('inf')
    for i in range(0, len(UnVisited)-1):
        for j in range(i + 1, len(UnVisited)):
            g.addEdge(i, j, edgeMatrix[cityDictionary[UnVisited[i]]][cityDictionary[UnVisited[j]]])
    # find shorest path return to start city
    for city in UnVisited:
        pathToStart = edgeMatrix[cityDictionary['A']][cityDictionary[city]]
        if pathToStart < minReturn:
            minReturn = pathToStart
    # Add states to frontier
    for city in UnVisited:
        newPath = currState[0][:]
        newPath.append(city)
        pathLength = currState[1] + edgeMatrix[cityDictionary[currState[0][-1]]][cityDictionary[city]]
        tempUnVisited = UnVisited[:]
        tempUnVisited.remove(city)
        frontier.append((newPath, pathLength, pathLength + g.KruskalMST() + minReturn))
        NumOfNodes += 1
    # find the state with smallest heuristic
    frontier = sorted(frontier, key=lambda city: city[2])
    currState = frontier.pop(0)
# add return to path
returnVal = edgeMatrix[cityDictionary['A']][cityDictionary[currState[0][-1]]]
currState[0].append('A')
print("The shortest path length is " + str(currState[1] + returnVal))
print("The shortest path is " + str(currState[0]))

