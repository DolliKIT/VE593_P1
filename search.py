#search algorithms

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

class NodeV:
    def __init__(self, element, parent, costSoFar):
        self.element = element
        self.parent = parent
        self.costSoFar = costSoFar

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
        (path, cost) = DLS(Graph, initialState, depth)
        end = timeit.default_timer()
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
    curCost = 0
    toAnalyse = queue.PriorityQueue()
    # toAnalyseDocu = []
    tupleInit = (0, sInit)  # (cost, state)
    nodeInit = Node(tupleInit, Node(0, tupleInit))
    visited = [nodeInit.element]  # list of visited Nodes
    countQ = 0  # counter for equal prio
    toAnalyse.put((0, countQ, nodeInit))
    # toAnalyseDocu.append(nodeInit)
    countQ += 1

    while toAnalyse.empty() == 0:
        path = []
        pathCost = 0
        curCost = 0
        toAnalyse = queue.PriorityQueue()
        # toAnalyseDocu = []
        tupleInit = (0, sInit)  # (cost, state)
        nodeInit = NodeV(tupleInit, NodeV(0, tupleInit, 0), 0)
        visited = [nodeInit.element]  # list of visited Nodes
        countQ = 0  # counter for equal prio
        toAnalyse.put((0, countQ, nodeInit))
        # toAnalyseDocu.append(nodeInit)
        countQ += 1

        while toAnalyse.empty() == 0:
            current = toAnalyse.get()[2]
            #print('get Element:',current.element, 'with costSoFar:', current.costSoFar)
            # curCost += current.element[0]
            visited.append(current.element)
            SucsCur = [it for it in valuedGraph.successors(current.element[1])]
            for iter in SucsCur:
                nodeIter = NodeV(iter, current, current.costSoFar + iter[0])

                if valuedGraph.isGoal(nodeIter.element[1]):
                    # path finding by parents
                    goThrough = current
                    while goThrough.element[1] != sInit:
                        path = [goThrough.element[1]] + path
                        pathCost += goThrough.element[0]
                        goThrough = goThrough.parent
                    return ([sInit] + path + [nodeIter.element[1]], pathCost)

                if iter not in visited or nodeIter.costSoFar < curCost:  # or cost is better
                    prio = nodeIter.costSoFar
                    insert = (prio, countQ, nodeIter)
                    toAnalyse.put(insert)
                    visited.append(nodeIter.element)
                    countQ += 1
                curCost = current.costSoFar

        return path, pathCost



def A_star(valuedGraph, sInit, heuristic):
    path = []
    pathCost = 0
    curCost = 0
    toAnalyse = queue.PriorityQueue()
    #toAnalyseDocu = []
    tupleInit = (0, sInit)  # (cost, state)
    nodeInit = NodeV(tupleInit, NodeV(0, tupleInit, 0), 0)
    visited = [nodeInit.element] # list of visited Nodes
    countQ = 0  # counter for equal prio
    toAnalyse.put((0, countQ, nodeInit))
    #toAnalyseDocu.append(nodeInit)
    countQ += 1

    while toAnalyse.empty() == 0:
        current = toAnalyse.get()[2]
        #curCost += current.element[0]
        visited.append(current.element)
        SucsCur = [it for it in valuedGraph.successors(current.element[1])]
        for iter in SucsCur:
            nodeIter = NodeV(iter, current, current.costSoFar + iter[0])

            if valuedGraph.isGoal(nodeIter.element[1]):
                # path finding by parents
                goThrough = current
                while goThrough.element[1] != sInit:
                    path = [goThrough.element[1]] + path
                    pathCost += goThrough.element[0]
                    goThrough = goThrough.parent
                return ([sInit] + path + [nodeIter.element[1]], pathCost)

            if iter not in visited or nodeIter.costSoFar < curCost: # or cost is better
                prio = nodeIter.costSoFar + heuristic(iter[1])
                insert = (prio, countQ, nodeIter)
                toAnalyse.put(insert)
                visited.append(nodeIter.element)
                countQ += 1
            curCost = current.costSoFar

    return path, pathCost






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
    while nodeT.parent != 0:
        nodeT.N += 1
        nodeT.Q += reward
        reward = -reward
        nodeT = nodeT.parent