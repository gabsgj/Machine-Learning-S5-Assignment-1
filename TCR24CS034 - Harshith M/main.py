from data import State, Edge, Prob
from plan import Plan

def show(name, res):
    print(name)
    if res.ok:
        print("path:", res.path)
        print("cost:", round(res.cost, 2))
    else:
        print("no path")
    print("")

def main():
    sys = Plan(1.0, 0.5, 0.5)

    s0 = State(0, [0.0, 0.0])
    s1 = State(1, [1.0, 0.0])
    s2 = State(2, [2.0, 0.0])
    s3 = State(3, [3.0, 0.0])
    s4 = State(4, [1.0, 1.0])
    s5 = State(5, [1.0, -2.0])
    s6 = State(6, [2.0, -2.0])
    
    st = [s0, s1, s2, s3, s4, s5, s6]
    p = Prob(0, 3, [], st, [])

    p.edges = [
        Edge(1, 0, 1, 1.0, 1.0, 1.0, True),
        Edge(2, 1, 2, 1.0, 1.0, 1.0, True),
        Edge(3, 2, 3, 1.0, 1.0, 1.0, True)
    ]
    show("test 1", sys.run(p))

    p.bad = [4]
    p.edges.append(Edge(4, 1, 4, 1.0, 1.0, 1.0, True))
    p.edges.append(Edge(5, 4, 3, 1.0, 1.0, 1.0, True))
    p.edges.append(Edge(6, 0, 5, 2.0, 1.0, 1.0, True))
    p.edges.append(Edge(7, 5, 6, 2.0, 1.0, 1.0, True))
    p.edges.append(Edge(8, 6, 3, 2.0, 1.0, 1.0, True))
    show("test 2", sys.run(p))

    s7 = State(7, [2.0, 0.5])
    p.states.append(s7)
    p.bad.append(7)
    show("test 3", sys.run(p))

    p.bad = []
    p.edges[1].a = False
    show("test 4", sys.run(p))

    p.edges[1].a = True
    p.end = 6
    show("test 5", sys.run(p))

    p.end = 3
    p.edges.append(Edge(9, 0, 3, 0.5, 1.0, 1.0, True))
    show("test 6", sys.run(p))

if __name__ == "__main__":
    main()
