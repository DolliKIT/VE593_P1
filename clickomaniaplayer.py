from clickomania import *
from search import A_star


def heuristic(state):
    # heuristic = 1 / score + c * single tiles better, first simpler: only score inverse
    return state.score


click = Clickomania(4,4,3)
initState = click.generateInitState()

print('Init State of Game:')
for i in range(len(initState.value)):
      print(initState.value[i])

blocks = click.findBlocks(initState)
succs = click.successors(initState)
