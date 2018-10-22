from clickomania import *
from Astar_click import A_star


def heuristic(toMeasure):
    # heuristic = score + c * single tiles better, first simpler: only score inverse
    state = StateC(0, toMeasure)
    return state.score


click = Clickomania(3,3,4)
initState = click.generateInitState()

def clickomaniaplayer(click, initState):
    print('Init State of Game:')
    for i in range(len(initState.value)):
          print(initState.value[i])

    blocks = click.findBlocks(initState)
    #succs = click.successors(initState)


    (path, scoreN) = A_star(click,initState.value,heuristic)

    print('Cost after Astar:', scoreN)

    scoreOverall = 0
    m = 0

    print('Path:', path)

    scoreM = 0
    if len(path) == 0:
        m = click.N * click.M
    else:
        if len(path[-1]) > 0:
            m = len(path[-1]) * len(path[-1][0])
            scoreM = (m-1)**2
    print('m:',m)

    scoreOverall = scoreN - scoreM

    print('Path:', path)
    print('Score:', scoreOverall)

    return (path, scoreOverall