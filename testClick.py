from clickomania import *

click = Clickomania(4,4,2)
initState = click.generateInitState()

print('Init:')
for i in range(len(initState.value)):
      print(initState.value[i])

tile = Tile(1, 2, 0)

blocks = click.findBlocks(initState)

for i in range(len(blocks)):
    print(blocks[i])
    click.deleteBlock(initState, blocks[i])

print('Deleted:')
for i in range(len(initState.value)):
      print(initState.value[i])