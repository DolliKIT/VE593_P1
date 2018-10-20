# test it

import timeit
import random
from search import BFS
from search import DFS
from search import DLS
from search import UCS
from search import IDS
from search import A_star
from testgraphs import SimpleGraph
from testgraphs import SimpleValuedGraph
from testgraphs import State
from testgraphs import nPuzzleGraph
from testgraphs import nPuzzleGraphValued

def heuristicGraph(state):
    if state == 0: return 5
    if state == 1: return 4
    if state == 2: return 4
    if state == 3: return 2
    if state == 4: return 2
    if state == 5: return 0

def heuristicPuzzle(state):
    heuristic = 0
    goalState = [1,2,3,4,5,6,7,8,0]
    for i in range(0,len(state.value)-1):
        if state.value[i] != goalState[i]:
            heuristic += 1
    return heuristic


graph = SimpleGraph()
graphValued = SimpleValuedGraph()
print('---------Graph----------')
""""
print('BFS:', BFS(graph, 0))
print('DFS:', DFS(graph, 0))
print('IDS:', IDS(graph, 0))
print('DLS:', DLS(graph, 0, 3))

print('UCS:', UCS(graphValued, 0))
print('Astar:', A_star(graphValued, 0, heuristicGraph))
"""
def gen_state(n):
    N = n * n
    init = list(range(1, N))
    for i in range(1,2*5 - 1):
        rand1 = random.randint(0, N - 2)
        rand2 = random.randint(0, N - 2)
        init[rand1], init[rand2] = init[rand2], init[rand1]
        print(init)
    init.append(0)
    ind = init.index(0)
    rand = random.randint(0, N - 2)
    init[rand], init[ind] = init[ind], init[rand]
    print('Init State:', init)
    return init


DEPTHLIMIT = 21
n = 3
puzzle = nPuzzleGraph(n)
puzzleValued = nPuzzleGraphValued(n)
#initState = [7, 2, 4, 5, 0, 6, 8, 3, 1]
#initState = [2,1,6,4,3,5,8,7,0]
initState = State(gen_state(n))
puzzle.setInitialState(initState)

puzzleValued.setInitialState(initState)

print('-------TestPuzzle--------')



#start = timeit.default_timer()
#print('UCS:',UCS(puzzleValued, puzzleValued.initialState))
#print('A_star:',A_star(puzzleValued, puzzleValued.initialState, heuristicPuzzle))
#end = timeit.default_timer()
#print('Time: ', end - start)


"""
start = timeit.default_timer()
print('IDS:',IDS(puzzle, puzzle.initialState))
end = timeit.default_timer()
print('Time: ', end - start)
"""
start = timeit.default_timer()
print('DLS:',DLS(puzzle, puzzle.initialState, DEPTHLIMIT))
end = timeit.default_timer()
print('Time: ', end - start)
"""
start = timeit.default_timer()
print('BFS:', BFS(puzzle, puzzle.initialState))
end = timeit.default_timer()
print('Time: ', end - start)
"""
"""
start = timeit.default_timer()
print('DFS:',DFS(puzzle, puzzle.initialState))
end = timeit.default_timer()
print('Time: ', end - start)
"""

NUMITER = 1


def wrapper(func, *args, **kwargs):
    def wrapped():
        return func(*args, **kwargs)
    return wrapped

print('------MeasurePuzzle-------')
"""
wrapped = wrapper(BFS, puzzle, puzzle.initialState)
print('BFS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)

wrapped = wrapper(DFS, puzzle, puzzle.initialState)
print('DFS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)

wrapped = wrapper(IDS, puzzle, puzzle.initialState)
print('IDS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)
"""

wrapped = wrapper(DLS, puzzle, puzzle.initialState, DEPTHLIMIT)
print('DLS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)


"""
wrapped = wrapper(UCS, graph, puzzle.initialState)
print('UCS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)
"""
