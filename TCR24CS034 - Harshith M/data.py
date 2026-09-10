class State:
    def __init__(self, i, em):
        self.i = i
        self.em = em

class Edge:
    def __init__(self, i, f, t, c, s, r, a):
        self.i = i
        self.f = f
        self.t = t
        self.c = c
        self.s = s
        self.r = r
        self.a = a

class Prob:
    def __init__(self, start, end, bad, states, edges):
        self.start = start
        self.end = end
        self.bad = bad
        self.states = states
        self.edges = edges

class Res:
    def __init__(self, ok, path, cost):
        self.ok = ok
        self.path = path
        self.cost = cost
