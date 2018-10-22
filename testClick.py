from clickomania import *

click = Clickomania(4,4,3)
initState = click.generateInitState()

print('Init:')
for i in range(len(initState.value)):
      print(initState.value[i])


blocks = click.findBlocks(initState)
print('Found Blocks:')
for i in range(len(blocks)):
    print(blocks[i])

print('Successors:')
succs = click.successors(initState)
for i in range(len(succs)):
    print(succs[i].value)

#click.deleteBlock(initState, blocks[i])
#click.fallDown(initState)
