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


VARIANCE = 4
def gen_state(n):
    N = n * n
    init = list(range(1, N))
    for i in range(1,2*VARIANCE + 1):
        rand1 = random.randint(0, N - 2)
        rand2 = random.randint(0, N - 2)
        init[rand1], init[rand2] = init[rand2], init[rand1]
    init.append(0)
    ind = init.index(0)
    rand = random.randint(0, N - 2)
    init[rand], init[ind] = init[ind], init[rand]
    print('Init State:', init)
    return init


print('-------MeasurePuzzle--------')

DEPTHLIMIT = 900 #recursion depth limit: 990
NumIterations = 5

sumIDS = 0
sumBFS = 0
sumDFS = 0
sumDLS = 0
sumAstar = 0
sumUCS = 0
sumMCTS = 0

nEnd = 11

print('Complexity Measurements of Search Algorithms with N =', NumIterations, 'Iterations.')

for n in range(3,nEnd+1,2):
    print('Size of Puzzle: n =', n)

    puzzle = nPuzzleGraph(n)
    puzzleValued = nPuzzleGraphValued(n)

    for i in range(NumIterations):
        print('Iteration', i)

        initState = gen_state(n)
        initState = State(initState)
        puzzle.setInitialState(initState)
        puzzleValued.setInitialState(initState)

        start = timeit.default_timer()
        BFS(puzzle, puzzle.initialState)
        end = timeit.default_timer()
        sumBFS += end - start

        start = timeit.default_timer()
        UCS(puzzleValued, puzzleValued.initialState)
        end = timeit.default_timer()
        sumUCS += end - start

        start = timeit.default_timer()
        DFS(puzzle, puzzle.initialState)
        end = timeit.default_timer()
        sumDFS += end - start

        start = timeit.default_timer()
        DLS(puzzle, puzzle.initialState, DEPTHLIMIT)
        end = timeit.default_timer()
        sumDLS += end - start

        start = timeit.default_timer()
        IDS(puzzle, puzzle.initialState)
        end = timeit.default_timer()
        sumIDS += end - start

        start = timeit.default_timer()
        A_star(puzzleValued, puzzleValued.initialState, heuristicPuzzle)
        end = timeit.default_timer()
        sumAstar += end - start

        # ! not yet working !
        """
        start = timeit.default_timer()
        MCTS(puzzleValued, puzzleValued.initialState, 100)
        end = timeit.default_timer()
        sumMCTS += end - start
        """

    print('average measured time for BFS:', sumBFS / NumIterations)
    print('average measured time for UCS:', sumUCS / NumIterations)
    print('average measured time for DFS:', sumDFS / NumIterations)
    print('average measured time for DLS:', sumDLS / NumIterations)
    print('average measured time for IDS:', sumIDS / NumIterations)
    print('average measured time for Astar:', sumAstar / NumIterations)
    #print('average measured time for MCTS:', sumMCTS / NumIterations)





