# test it

import timeit
import random
from search import BFS
from search import DFS
from search import DLS
from search import UCS
from search import IDS
from search import A_star
from testgraphs import State
from testgraphs import nPuzzleGraph
from testgraphs import nPuzzleGraphValued
from search import MCTS

def heuristicPuzzle(state):
    heuristic = 0
    goalState = [1,2,3,4,5,6,7,8,0]
    for x in list(range(0,3)):
        for y in list(range(0,3)):
            x_value = state.value[x]
            y_value = state.value[x*y]
            x_goal =  goalState[x]
            y_goal = goalState[x*y]
            heuristic += abs(x_value - x_goal) + abs(y_value - y_goal)
    return heuristic


def gen_state(n):
    N = n * n
    init = list(range(1, N))
    for i in range(1,2*4 + 1):
        rand1 = random.randint(0, N - 2)
        rand2 = random.randint(0, N - 2)
        init[rand1], init[rand2] = init[rand2], init[rand1]
    init.append(0)
    ind = init.index(0)
    rand = random.randint(0, N - 2)
    init[rand], init[ind] = init[ind], init[rand]
    print('Init State:', init)
    return init


DEPTHLIMIT = 100
"""
n = 4
puzzle = nPuzzleGraph(n)
puzzleValued = nPuzzleGraphValued(n)
initState = [7, 2, 4, 5, 0, 6, 8, 3, 1]
#initState = [1,2,3,0,4,5,6,7,8]
#initState = gen_state(n)
initState = State(initState)
puzzle.setInitialState(initState)
puzzleValued.setInitialState(initState)
"""

print('-------MeasurePuzzle--------')

NumIterations = 5

sumIDS = 0
sumBFS = 0
sumDFS = 0
sumDLS = 0
sumAstar = 0
sumUCS = 0



#for i in range(NumIterations):

inits = [[1, 5, 4, 7, 8, 6, 3, 0, 2], [7, 5, 1, 6, 2, 0, 3, 8, 4], [0, 4, 6, 3, 8, 5, 7, 2, 1], [1, 8, 7, 0, 5, 3, 4, 2, 6], [7, 8, 4, 2, 5, 1, 0, 3, 6]]

n = 4
print('Complexity Measurements of Search Algorithms with N =', NumIterations, 'Iterations.')
print('Size of Puzzle: n =', n)

#initState = gen_state(n)
# and i in range(1,NumIter)


#for initState in inits:
for i in range(1,NumIterations):
    print('Iteration', i) # i instead of initState

    puzzle = nPuzzleGraph(n)
    puzzleValued = nPuzzleGraphValued(n)
    initState = gen_state(n)
    initState = State(initState)
    puzzle.setInitialState(initState)
    puzzleValued.setInitialState(initState)
    """
    start = timeit.default_timer()
    print('A_star:',A_star(puzzleValued, puzzleValued.initialState, heuristicPuzzle))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumAstar += end - start

    
    start = timeit.default_timer()
    print('UCS:',UCS(puzzleValued, puzzleValued.initialState))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumUCS += end - start
    """
    start = timeit.default_timer()
    print('BFS:', BFS(puzzle, puzzle.initialState))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumBFS += end - start

    start = timeit.default_timer()
    DFS(puzzle, puzzle.initialState)
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumDFS += end - start


    """
for initState in inits:

    print('Iteration', initState)  # i instead of initState

    puzzle = nPuzzleGraph(n)
    initState = State(initState)
    puzzle.setInitialState(initState)
    """
    """
    start = timeit.default_timer()
    print('IDS:',IDS(puzzle, puzzle.initialState))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumIDS += end - start

    start = timeit.default_timer()
    print('DLS:',DLS(puzzle, puzzle.initialState, DEPTHLIMIT))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumDLS += end - start
    
    start = timeit.default_timer()
    print('BFS:', BFS(puzzle, puzzle.initialState))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumBFS += end - start
    
    start = timeit.default_timer()
    print('DFS:',DFS(puzzle, puzzle.initialState))
    end = timeit.default_timer()
    print('Time: ', end - start)
    sumDFS += end - start
"""

print('average measured time for Astar:', sumAstar / NumIterations)
print('average measured time for UCS:', sumUCS / NumIterations)
print('average measured time for BFS:', sumBFS / NumIterations)
print('average measured time for DFS:', sumDFS / NumIterations)
print('average measured time for DLS:', sumDLS / NumIterations)
print('average measured time for IDS:', sumIDS / NumIterations)
