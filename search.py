#search algorithms

import heapq
#import numpy
import math
import queue
import timeit
import random

from testgraphs import SimpleValuedGraph
from testgraphs import SimpleGraph
#from testing import heuristic


class Node:
    def __init__(self, element, parent):
        self.element = element
        self.parent = parent

# BFS
def BFS(Graph, initialState):

    if Graph.isGoal(initialState):
        return (initialState,0)

    toVisit = queue.Queue()
    init = Node(initialState, initialState)
    finished = [init.element]
    for i in Graph.successors(initialState):
        it = Node(i,init)
        toVisit.put(it)

    path = []
    cost = 0
    while not toVisit.empty():
        visitIt = toVisit.get()
        if visitIt.element not in finished:
            finished.append(visitIt.element)
        if Graph.isGoal(visitIt.element):
            while visitIt.element != initialState:
                path = [visitIt.element] + path
                visitIt = visitIt.parent
                cost += 1
            return ([initialState] + path, cost)
        else:
            for i in Graph.successors(visitIt.element):
                if i not in finished:
                    it = Node(i, visitIt)
                    toVisit.put(it)
"""
        it = len(finished)
        goThrough = finished[it - 1]
        while goThrough.element != initialState:
            path = [goThrough.element] + path
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
    finished = [init.element]
    for i in Graph.successors(initialState):
        it = Node(i,init)
        toVisit.put(it)

    path = []
    cost = 0
    while not toVisit.empty():
        visitIt = toVisit.get()
        if Graph.isGoal(visitIt.element):
            while visitIt.element != initialState:
                path = [visitIt.element] + path
                visitIt = visitIt.parent
                cost += 1
            return ([initialState] + path, cost)
        else:
            if visitIt.element not in finished:
                finished.append(visitIt.element)
                for i in Graph.successors(visitIt.element):
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
        # path finding by parents
        it = len(finished)
        goThrough = finished[it-1]
        while goThrough.element != initialState:
            path = [goThrough.element] + path
            goThrough = goThrough.parent
            cost += 1
        return ([initialState] + path, cost)
    else:
        #print('Goal node not found (assuming its existence because of Depth Limit', depthLimit)
        return ([],0)

def DLS_rec(Graph, cur, depthLimit, finished, found):

    if found == True:
        return (finished, True)
    else:
        if Graph.isGoal(cur.element):
            finished.append(cur)
            found = True
        else:
            if cur not in finished:
                finished.append(cur)
                if depthLimit > 0:
                    depthLimit -= 1
                    for suc in Graph.successors(cur.element):
                        if suc not in [c.element for c in finished]:
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
    tupleInit = (0, sInit)
    nodeInit = Node(tupleInit,Node(0,tupleInit))
    visited = [nodeInit]
    countQ = 0 # counter for equal prio
    toVisit.put((0, countQ, nodeInit))
    countQ += 1

    while toVisit.empty() == 0:
        current = toVisit.get()[2]
        pathCost = pathCost + current.element[0]
        SucsCur = [it for it in valuedGraph.successors(current.element[1])]
        for iter in SucsCur:
            nodeIter = Node(iter, current)
            if valuedGraph.isGoal(nodeIter.element[1]):
                # path finding by parents
                visited.append(nodeIter)
                length = visited.index(nodeIter)
                goThrough = visited[length]
                cost = 0
                while goThrough.element[1] != sInit:
                    path = [goThrough.element[1]] + path
                    cost += goThrough.element[0]
                    goThrough = goThrough.parent
                return ([sInit] + path, cost)

            curCost = pathCost + iter[0]
            if iter not in visited:
                iterNode = Node(iter, current)
                toVisit.put((curCost, countQ, iterNode))
                countQ += 1
                visited.append(iterNode)

    return path, pathCost



def A_star(valuedGraph, sInit, heuristic):
    path = []
    pathCost = 0
    toVisit = queue.PriorityQueue()
    toVisitDocu = []
    tupleInit = (0, sInit)
    nodeInit = Node(tupleInit, Node(0, tupleInit))
    visited = []
    countQ = 0  # counter for equal prio
    toVisit.put((0, countQ, nodeInit))
    toVisitDocu.append(nodeInit)
    countQ += 1

    while toVisit.empty() == 0:
        current = toVisit.get()[2]
        toVisitDocu.remove(current)
        visited.append(current)
        pathCost = 0
        #pathCost = pathCost + current.element[0]
        SucsCur = [it for it in valuedGraph.successors(current.element[1])]
        for iter in SucsCur:
            nodeIter = Node(iter, current)
            if valuedGraph.isGoal(nodeIter.element[1]):
                # path finding by parents
                visited.append(nodeIter)
                length = visited.index(nodeIter)
                goThrough = visited[length]
                while goThrough.element[1] != sInit:
                    path = [goThrough.element[1]] + path
                    pathCost += goThrough.element[0]
                    goThrough = goThrough.parent
                return ([sInit] + path, pathCost)

            curCost = pathCost + iter[0]
            if iter not in visited:
                iterNode = Node(iter, current)
                prio = curCost + heuristic(iter[1])
                print(heuristic(iter[1]))
                insert = (prio, countQ, iterNode)
                if insert[2] not in toVisitDocu:
                    toVisit.put(insert)
                    toVisitDocu.append(insert[2])
                    countQ += 1 ## overflow possible

    return path, pathCost


NUM_STATES = 5

class NodeT:
    def __init__(self, state, sucs, Q, N, parent, reward):
        self.state = state # State(value, blank position)
        self.sucs = sucs # tuple (cost, successor)
        self.Q = Q
        self.N = N
        self.parent = parent
        self.reward = reward

    def add_suc(self, sucState):
        self.sucState = sucState

    def update(self, reward):
        self.reward = reward



def MCTS(ValuedGraph, state, budget):
    initNodeState = NodeT(state, [], 0, 0, [], 0)
    treeList = []
    for iter in range(int(budget)):
        leaf = treePolicy(ValuedGraph, initNodeState)
        reward = rolloutPolicy(ValuedGraph, leaf)
        #backUp(ValuedGraph, leaf, reward)
    return bestChild(state) # reward

def treePolicy(ValuedGraph, nodeT):
    while not ValuedGraph.isGoal(suc[1]) and len(ValuedGraph.successors(suc[1])) != 0:
        if len(nodeT.sucs) == 0:
            return expand(ValuedGraph, nodeT)
        else:
            suc = bestChild(ValuedGraph, nodeT)
    return suc


def bestChild(graph, nodeT):
    c = 1/math.sqrt(2)
    sucsCost = [0]*len(nodeT.sucs)
    if nodeT.N != 0:
        sucsCost = [(nodeT.Q / nodeT.N + c * math.sqrt(2*math.log(i[1].N/nodeT.N))) for i in nodeT.sucs]
    else:
        i = 0
        for itSuc in nodeT.sucs:
            if itSuc[1].N == 0:
                sucsCost[i] = 99999
            else:
                sucsCost[i] = (c * math.sqrt(2 * math.log(itSuc.N / nodeT.N)))
            i += 1
    iSuc = sucsCost.index(max(sucsCost))
    suc = graph.successors(nodeT.state)[iSuc]
    return suc # tuple (cost, state)

def expand(ValuedGraph, nodeT):
    succs = [i for i in ValuedGraph.successors(nodeT.state) if i not in nodeT.sucs]
    toExpand = succs[0] # choose first element
    toExpandNode = NodeT(toExpand[1],[],0,0,nodeT,0)
    nodeT.sucs.append((toExpand[0], toExpandNode))
    return toExpand # (cost, state)

def rolloutPolicy(ValuedGraph, leaf):
    # TODO Node not necessary ...
    nodeG = Node(leaf[1], 0)
    cost = 0
    num = 10
    visited = []
    while (not ValuedGraph.isGoal(nodeG.element)) and num != 0:
        simSuc = random.randint(0,len(ValuedGraph.successors(nodeG.element))-1)
        g = ValuedGraph.successors(nodeG.element)[simSuc]
        if g not in visited:
            nodeG = Node(g[1],nodeG)
            cost += g[0]
        visited.append(g)
        num -= 1
    if cost == 0:
        reward = 1
    else:
        reward = 1 / cost

def backUp(ValuedGraph, nodeT, reward):