import random
import queue
import copy

# state of Clickomania game
class StateC:
    def __init__(self, score, value):
        self.score = score
        self.value = value # matrix[N X M]

    def clone(self):
        return StateC(self.score, copy.copy(self.value))

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

    def setNM(self, stateValue):
        sizeN = len(stateValue)
        sizeM = len(stateValue[0])
        self.N = sizeN
        self.M = sizeM


    def isGoal(self, toConsider):
        state = StateC(0, toConsider)
        if len(self.findBlocks(state)) == 0:
            return 1
        else:
            return 0

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
        tilesInBlock = []
        blocks = []
        toVisit = queue.Queue()
        for i in range(self.N):
            for j in range(self.M): # tile = state.value[n,m], here m = tile
                n = i
                m = j
                if state.value[n][m] != 0 and (n,m) not in tilesInBlock:
                    tile = Tile(n,m,state.value[n][m])
                    block = []
                    toVisit.put((n,m))
                    block.append((n,m))
                    tilesInBlock.append((n,m))
                    while not toVisit.empty():
                        (n, m) = toVisit.get()
                        if n < self.N and m < self.M:
                            # look up
                            if (n - 1 >= 0) and (state.value[n - 1][m] == tile.colour) and ((n - 1, m) not in tilesInBlock):
                                block.append((n - 1, m))
                                tilesInBlock.append((n - 1, m))
                                toVisit.put((n-1,m))
                            # look down
                            if (n + 1 < self.N) and (state.value[n+1][m] == tile.colour) and ((n+1,m) not in tilesInBlock):
                                block.append((n+1,m))
                                tilesInBlock.append((n+1,m))
                                toVisit.put((n+1,m))
                            # look left
                            if (m - 1 >= 0) and (state.value[n][m-1] == tile.colour) and ((n,m-1) not in tilesInBlock):
                                block.append((n,m-1))
                                tilesInBlock.append((n,m-1))
                                toVisit.put((n,m-1))
                            # look right
                            if (m + 1 < self.M) and (state.value[n][m+1] == tile.colour) and ((n,m+1) not in tilesInBlock):
                                block.append((n,m+1))
                                tilesInBlock.append((n,m+1))
                                toVisit.put((n,m+1))

                    if len(block) > 1:
                        blocks.append(block) # block: list of tile tuples of a block
        return blocks


    def deleteBlock(self, oldValue, delete):
        # setting colour to 0
        newValue = []
        for i in range(self.N):
            newValue.append([])
            for j in range(self.M):
                newValue[i].append(0)
        #newValue = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]] # so, now link between numbers...

        for i in range(self.N):
            for j in range(self.M):
                test = (i,j)
                if test not in delete:
                    newValue[i][j] = oldValue[i][j]

        return newValue

    def fallDown(self, state):
        colours = 0
        # "falling down"
        toFallDown = []
        for i in range(self.N):
            for j in range(self.M):
                if state.value[i][j] == 0:
                    toFallDown.append((i,j))

        toFallDown.sort(reverse=True)
        for fD in toFallDown:
            if fD[0] != self.N - 1: # highest row not necessary
                for n in range(fD[0],self.N - 1):
                    state.value[n][fD[1]] = state.value[n+1][fD[1]]
                state.value[self.N - 1][fD[1]] = 0

        for i in range(max(self.N, self.M)): # testing it iteratively
            # check if a row only consists of zeros
            n = 0
            m = 0
            while n < self.N:
                while m < self.M:
                    colours += state.value[n][m]
                    m += 1
                if colours == 0:
                    self.N -= 1
                    del state.value[n]
                else:
                    colours = 0
                n += 1
                m = 0

            # check if a column only consists of zeros
            m = 0
            n = 0
            while m < self.M:
                while n < self.N:
                    colours += state.value[n][m]
                    n += 1
                if colours == 0:
                    self.M -= 1
                    for i in state.value:
                        del i[m]
                else:
                    colours = 0
                m += 1
                n = 0

        return state


    def successors(self, current):
        parState = StateC(0, current)
        blocks = self.findBlocks(parState)
        succs = []
        toChange = self
        for b in blocks:
            deletedBlocks = len(b)
            newValue = toChange.deleteBlock(parState.value, b)
            newState = StateC(parState.score, newValue)
            newState =  toChange.fallDown(newState)
            score = (deletedBlocks - 1)**2

            succs.append((score, newState.value))

        return succs


