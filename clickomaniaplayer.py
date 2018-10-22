from clickomania import *
from Astar_click import A_star


def heuristic(toMeasure):
    # heuristic = score + c * single tiles better, first simpler: only score
    state = StateC(0, toMeasure)
    return state.score


def clickomaniaplayer(click, initState):

    (path, scoreN) = A_star(click,initState.value,heuristic)

    m = 0
    scoreM = 0

    if len(path) == 0:
        m = click.N * click.M
    else:
        if len(path[-1]) > 0:
            m = len(path[-1]) * len(path[-1][0])
            scoreM = (m-1)**2

    scoreOverall = scoreN - scoreM

    return (path, scoreOverall)


"""
click = Clickomania(3,3,5)
initState = click.generateInitState()

print('Init State of Game:')
for i in range(len(initState.value)):
    print(initState.value[i])
"""