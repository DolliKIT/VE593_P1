#search algorithms

import heapq
#import numpy
import math
import queue
import timeit

from testgraphs import SimpleValuedGraph
from testgraphs import SimpleGraph
#from testing import heuristic


class Node:
    def __init__(self, state, suc):
        self.state = state
        self.suc = suc

# BFS
def BFS(Graph, initialState):

    if Graph.isGoal(initialState):
        return (initialState,0)

    toVisit = queue.Queue()
    init = Node(initialState, initialState)
    finished = [init.state]
    for i in Graph.successors(initialState):
        it = Node(i,init)
        toVisit.put(it)

    path = []
    cost = 0
    while not toVisit.empty():
        visitIt = toVisit.get()
        if visitIt.state not in finished:
            finished.append(visitIt.state)
        if Graph.isGoal(visitIt.state):
            while visitIt.state != initialState:
                path = [visitIt.state] + path
                visitIt = visitIt.suc
                cost += 1
            return ([initialState] + path, cost)
        else:
            for i in Graph.successors(visitIt.state):
                if i not in finished:
                    it = Node(i, visitIt)
                    toVisit.put(it)
"""
        it = len(finished)
        goThrough = finished[it - 1]
        while goThrough.state != initialState:
            path = [goThrough.state] + path
            goThrough = goThrough.suc
            cost += 1
        return ([initialState] + path, cost)
"""

# DFS
def DFS(Graph, initialState):

    if Graph.isGoal(initialState):
        return (initialState,0)

    toVisit = queue.LifoQueue()
    init = Node(initialState,initialState)
    finished = [init.state]
    for i in Graph.successors(initialState):
        it = Node(i,init)
        toVisit.put(it)

    path = []
    cost = 0
    while not toVisit.empty():
        visitIt = toVisit.get()
        if Graph.isGoal(visitIt.state):
            while visitIt.state != initialState:
                path = [visitIt.state] + path
                visitIt = visitIt.suc
                cost += 1
            return ([initialState] + path, cost)
        else:
            if visitIt.state not in finished:
                finished.append(visitIt.state)
                for i in Graph.successors(visitIt.state):
                    if i not in finished:
                        it = Node(i, visitIt)
                        toVisit.put(it)


# DLS
def DLS(Graph, initialState, depthLimit):
    finished = []
    path = []
    cost = 0
    found = False
    init = Node(initialState, initialState)
    (finished, found) = DLS_rec(Graph, init, depthLimit, finished, found)
    if found == True:
        # path finding by successors
        it = len(finished)
        goThrough = finished[it-1]
        while goThrough.state != initialState:
            path = [goThrough.state] + path
            goThrough = goThrough.suc
            cost += 1
        return ([initialState] + path, cost)
    else:
        #print('Goal node not found (assuming its existence because of Depth Limit', depthLimit)
        return ([],0)

def DLS_rec(Graph, cur, depthLimit, finished, found):

    if found == True:
        return (finished, True)
    else:
        if Graph.isGoal(cur.state):
            finished.append(cur)
            found = True
        else:
            if cur not in finished:
                finished.append(cur)
                if depthLimit > 0:
                    depthLimit -= 1
                    for suc in Graph.successors(cur.state):
                        if suc not in [c.state for c in finished]:
                            sucNode = Node(suc,cur)
                            (path, found) = DLS_rec(Graph, sucNode, depthLimit, finished, found)

        return (finished, found)


# IDS
NRANGE = 1000
def IDS(Graph, initialState):

    for depth in range(1,NRANGE):
        start = timeit.default_timer()
        (path, cost) = DLS(Graph, initialState, depth)
        #print('depth', depth, '-', path, cost)
        end = timeit.default_timer()
        #print('Time: ', end - start)
        if len(path) != 0:
            return (path, cost)


def gen_sucs(graph, cur):
    sucs = graph.successors(cur[1])
    for i in range(len(sucs)):
        yield sucs[i]

# UCS
def UCS(valuedGraph, sInit):
    path = []
    pathCost = 0
    toVisit = queue.PriorityQueue()
    visited = []
    sInit = (1, sInit)
    countQ = 0 # counter for equal prio
    toVisit.put((0, countQ, sInit))
    countQ += 1

    while toVisit.empty() == 0:
        current = toVisit.get()[2]
        pathCost = pathCost + current[0]
        #genSucs = gen_sucs(valuedGraph, current)
        SucsCur = [it for it in valuedGraph.successors(current[1])]
        for iter in SucsCur:
            if valuedGraph.isGoal(iter[1]):
                path.append(iter[1])
                return path, pathCost
            curCost = pathCost + iter[0]
            if iter not in visited:
                toVisit.put((curCost, countQ, iter))
                countQ += 1
                visited.append(iter)
                path.append(current[1])
                if len(path) > 1:
                    if path[-2] == current[1]:
                        del path[-1]
        visited.append(current)

    return path, pathCost


class PriorityQueue:
    def __init__(self):
        self.elements = []
    def putNode(self, node, priority):
        #print('priority:', priority, 'node:', node)
        heapq.heappush(self.elements, (priority, node))
    def popNode(self):
        return heapq.heappop(self.elements)[1]
    def nodesExist(self):
        return len(self.elements) != 0



def A_star(valuedGraph, sInit, heuristic):
    path = []
    pathCost = 0
    toVisit = PriorityQueue()
    visited = []
    sInit = (0, sInit)
    toVisit.putNode(sInit, 0)

    while toVisit.nodesExist() != 0:
        current = toVisit.popNode()
        pathCost = pathCost + current[0]

        if valuedGraph.isGoal(current[1]):
            path.append(current[1])
            return path, pathCost
        else:
            genSucs = gen_sucs(valuedGraph, current)
            for iter in genSucs:
                curCost = pathCost + iter[0]
                if (iter not in visited or curCost < pathCost):
                    prio = curCost + heuristic(iter[1])
                    toVisit.putNode(iter, prio)
                    visited.append(iter)
                    path.append(current[1])
                    if len(path) > 1:
                        if path[-2] == current[1]:
                            del path[-1]
            #visited.append(current)

    return path, pathCost


NUM_STATES = 5

class NodeM:
    def __init__(self, state, sucs, Q, N, parent, reward):
        self.state = state
        self.sucs = sucs # tuple (cost, successor)
        self.Q = Q
        self.N = N
        self.parents = parent
        self.reward = reward

    def add_suc(self, sucState):
        self.sucState = sucState

    def update(self, reward):
        self.reward = reward



"""
def gen_nodeGraph(sourceGraph):
    nodeGraph = []
    for i in range(NUM_STATES): # all nodes available
        node = Node(i, sourceGraph.successors(i), 0, 0, [], 0)
        nodeGraph.append(node)
    return nodeGraph

def MCTS(ValuedGraph, state, budget):
    graph = gen_nodeGraph(ValuedGraph)
    for iter in range(int(budget)):
        #leaf = treePolicy(state)
        #reward = rolloutPolicy(leaf.state)
        #backUp(lea, reward)
    #return bestChild(state, reward)

def treePolicy(node):
"""

