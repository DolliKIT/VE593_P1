import queue

class NodeV:
    def __init__(self, element, parent, costSoFar):
        self.element = element
        self.parent = parent
        self.costSoFar = costSoFar


def A_star(valuedGraph, sInit, heuristic):
    path = []
    pathCost = 0
    curCost = 0
    toAnalyse = queue.PriorityQueue()
    #toAnalyseDocu = []
    tupleInit = (0, sInit)  # (cost, state)
    nodeInit = NodeV(tupleInit, NodeV(0, tupleInit, 0), 0)
    visitItem = tuple(nodeInit.element)
    visited = [visitItem] # list of visited Nodes, set much more efficient but elements must be mutable...
    countQ = 0  # counter for equal prio
    toAnalyse.put((0, countQ, nodeInit))
    #toAnalyseDocu.append(nodeInit)
    countQ += 1

    while toAnalyse.empty() == 0:
        current = toAnalyse.get()[2]
        valuedGraph.setNM(current.element[1])
        #print('N:',valuedGraph.N,', M:', valuedGraph.M)
        #curCost += current.element[0]
        visited.append(current.element)
        SucsCur = [it for it in valuedGraph.successors(current.element[1])]
        for iter in SucsCur:
            nodeIter = NodeV(iter, current, current.costSoFar + iter[0])

            if valuedGraph.isGoal(nodeIter.element[1]):
                # path finding by parents
                goThrough = nodeIter
                while goThrough.element[1] != sInit:
                    path = [goThrough.element[1]] + path
                    pathCost += goThrough.element[0]
                    goThrough = goThrough.parent
                return ([sInit] + path, pathCost)

            if iter not in visited: # or cost is better or nodeIter.costSoFar > curCost
                prio = nodeIter.costSoFar + heuristic(iter[1]) # max is the goal
                insert = (1 / prio, countQ, nodeIter)
                toAnalyse.put(insert)
                visited.append(nodeIter.element)
                countQ += 1
            curCost = current.costSoFar

    return path, pathCost
