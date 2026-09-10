import math
import heapq
from data import Res

class Plan:
    def __init__(self, b, g, d):
        self.b = b
        self.g = g
        self.d = d

    def dist(self, s1, s2):
        tot = 0.0
        for v1, v2 in zip(s1.em, s2.em):
            tot = tot + (v1 - v2) * (v1 - v2)
        return math.sqrt(tot)

    def bad_dist(self, s, p, smap):
        if len(p.bad) == 0:
            return 1000.0
        m_dist = 9999.0
        for b_id in p.bad:
            if b_id in smap:
                d = self.dist(s, smap[b_id])
                if d < m_dist:
                    m_dist = d
        return m_dist

    def run(self, p):
        adj = {}
        for e in p.edges:
            if e.a == True:
                if e.f not in adj:
                    adj[e.f] = []
                adj[e.f].append(e)

        smap = {}
        for s in p.states:
            smap[s.i] = s

        bset = set(p.bad)
        q = []
        heapq.heappush(q, (0.0, p.start, 0.0, [p.start]))
        m_cost = {}
        m_cost[p.start] = 0.0

        while len(q) > 0:
            fc, curr, gc, path = heapq.heappop(q)

            if curr == p.end:
                return Res(True, path, gc)

            if gc > m_cost.get(curr, 9999.0):
                continue

            if curr not in adj:
                continue

            for e in adj[curr]:
                nxt = e.t
                if nxt in bset:
                    continue

                s_dist = self.bad_dist(smap[nxt], p, smap)
                step = (self.b * e.c) - (self.g * s_dist) - (self.d * e.r)
                if step < 0.01:
                    step = 0.01

                ngc = gc + step

                if ngc < m_cost.get(nxt, 9999.0):
                    m_cost[nxt] = ngc
                    hc = self.dist(smap[nxt], smap[p.end])
                    nfc = ngc + hc
                    
                    npath = list(path)
                    npath.append(nxt)
                    heapq.heappush(q, (nfc, nxt, ngc, npath))

        return Res(False, [], 0.0)
