import random

# state of Clickomania game
class StateC:
    def __init__(self, scoreStep, value):
        self.scoreStep = scoreStep
        self.value = value # matrix[N X M]

class Tile:
    def __init__(self, n, m, colour):
        self.n = n
        self.m = m
        self.colour = colour

class Clickomania:

    def __init__(self, N, M, K):
        self.N = N
        self.M = M
        self.K = K

    def isGoal(self, state):
        if len(self.findBlocks(state)) == 0:
            return 1
        else:
            0

    def generateInitState(self):
        values = []
        for n in range(self.N):
            valuesM = []
            for m in range(self.M):
                colour = random.randint(1,self.K)
                valuesM.append(colour)
            values.append(valuesM)
        init = StateC(0,values)
        return init


    def findBlocks(self, state):
        # also possible with filter function but less convenient to read
        tilesInBlock = []
        blocks = []
        for n in range(self.N):
            for m in range(self.M): # tile = state.value[n,m], here m = tile
                if state.value[n][m] != 0 and (n,m) not in tilesInBlock:
                    tile = Tile(n,m,state.value[n][m])
                    block = []

                    # looking for blocks for each tile
                    # look up (start from init n)
                    nB = tile.n
                    while (nB < self.N) and (state.value[nB][tile.m] == tile.colour) and ((nB,tile.m) not in tilesInBlock):
                        block.append((nB, tile.m))
                        tilesInBlock.append((nB, tile.m))
                        # look left (start from init m)
                        mB = tile.m
                        while (mB > 0) and (state.value[nB][mB] == tile.colour) and ((nB,mB) not in tilesInBlock):
                            block.append((tile.n, mB))
                            tilesInBlock.append((nB, mB))
                            mB -= 1
                        # look right
                        mB = tile.m + 1
                        while (mB < self.M) and (state.value[nB][mB] == tile.colour) and ((nB,mB) not in tilesInBlock):
                            block.append((nB, mB))
                            tilesInBlock.append((nB, mB))
                            mB += 1
                        nB += 1
                    # look down
                    nB = tile.n - 1
                    while (nB > 0) and (state.value[nB][tile.m] == tile.colour) and ((nB,tile.m) not in tilesInBlock):
                            block.append((nB, tile.m))
                            tilesInBlock.append((nB, tile.m))
                            # look left (start from init m)
                            mB = tile.m
                            while (mB > 0) and (state.value[nB][mB] == tile.colour) and ((nB,mB) not in tilesInBlock):
                                block.append((tile.n, mB))
                                tilesInBlock.append((nB, mB))
                                mB -= 1
                            # look right
                            mB = tile.m + 1
                            while (mB < self.M) and (state.value[nB][mB] == tile.colour) and ((nB,mB) not in tilesInBlock):
                                block.append((nB, mB))
                                tilesInBlock.append((nB, mB))
                                mB += 1
                            nB -= 1

                    if len(block) > 1:
                        blocks.append(block) # block: list of tile tuples of a block
        return blocks


    def deleteBlock(self, state, delete):
        colours = 0
        for toDel in delete:
            n = toDel[0]
            m = toDel[1]
            state.value[n][m] = 0 # deleted tiles

        # check if a row only consists of zeros
        for n in range(self.N):
            for m in range(self.M):
                colours += state.value[n][m]
            if colours == 0:
                self.N -= 1
                print('N StateValue before Del:',state.value)
                del state.value[n][:]
                print('N StateValue after Del:', state.value)
            else:
                colours = 0

        # check if a column only consists of zeros
        for m in range(self.M):
            for n in range(self.N):
                colours += state.value[n][m]
            if colours == 0:
                self.M -= 1
                print('M StateValue before Del:', state.value)
                del state.value[:][m]
                print('M StateValue after Del:', state.value)
            else:
                colours = 0
"""
    def successors(self, state):
        blocks = self.findBlocks(state)
        succs = []
        for  b in blocks:
   """


