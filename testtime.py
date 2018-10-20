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

def heuristicPuzzle(state):
    heuristic = 0
    goalState = [1,2,3,4,5,6,7,8,0]
    for i in range(0,len(state.value)-1):
        if state.value[i] != goalState[i]:
            heuristic += 1
    return heuristic

def gen_state(n):
    N = n * n
    init = list(range(1, N))
    for i in range(1,2*5):
        rand1 = random.randint(0, N - 2)
        rand2 = random.randint(0, N - 2)
        init[rand1], init[rand2] = init[rand2], init[rand1]
    init.append(0)
    ind = init.index(0)
    rand = random.randint(0, N - 2)
    init[rand], init[ind] = init[ind], init[rand]
    print('Init State:', init)
    return init

n = 3
puzzle = nPuzzleGraph(n)
puzzleValued = nPuzzleGraphValued(n)
initState = State(gen_state(n))
puzzle.setInitialState(initState)

puzzleValued.setInitialState(initState)


NUMITER = 5
DEPTHLIMIT = 25

print('------MeasurePuzzle-------')

wrapped = wrapper(BFS, puzzle, puzzle.initialState)
print('BFS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)

wrapped = wrapper(DFS, puzzle, puzzle.initialState)
print('DFS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)

wrapped = wrapper(IDS, puzzle, puzzle.initialState)
print('IDS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)


wrapped = wrapper(DLS, puzzle, puzzle.initialState, DEPTHLIMIT)
print('DLS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)



wrapped = wrapper(UCS, graph, puzzle.initialState)
print('UCS:', timeit.timeit(wrapped, number=NUMITER) / NUMITER)
