
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

struct State {
    uint64_t id{};
    vector<double> embedding;
};

struct Transition {
    uint64_t id{}, from{}, to{};
    double cost{1.0};
    double safety{1.0};       // [0,1], higher is safer
    double reliability{1.0}; // [0,1], higher is better
    bool available{true};
};

struct PlanningProblem {
    uint64_t initialState{}, goalState{};
    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};

struct PlanningResult {
    bool success{false};
    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;
    double totalCost{0.0};
    double safetyScore{0.0};       // minimum Euclidean distance to a bad state
    double cumulativeReliability{0.0};
    size_t exploredStates{0};
    double planningTimeMs{0.0};
    size_t memoryBytes{0};
    double objectiveScore{0.0};
};

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

/*
 D* Lite planner.
 The graph is static between update calls, but D* Lite keeps its g/rhs
 values and repairs only affected portions when edge/goal updates occur.
 For simplicity and clarity, this implementation rebuilds the predecessor
 index only when the graph topology is changed; the shortest-path values
 themselves are incrementally repaired.
*/
class DStarLite : public Planner {
    static constexpr double INF = numeric_limits<double>::infinity();
    static constexpr double EPS = 1e-10;

    struct HeapEntry {
        double k1, k2;
        uint64_t u;
        uint64_t stamp;
    };
    struct Compare {
        bool operator()(const HeapEntry& a, const HeapEntry& b) const {
            if (a.k1 != b.k1) return a.k1 > b.k1;
            if (a.k2 != b.k2) return a.k2 > b.k2;
            return a.u > b.u;
        }
    };

    PlanningProblem p;
    unordered_map<uint64_t, State> stateMap;
    unordered_map<uint64_t, Transition> trMap;
    unordered_map<uint64_t, vector<uint64_t>> outgoing, incoming;
    unordered_set<uint64_t> bad;
    unordered_map<uint64_t, double> g, rhs;
    unordered_map<uint64_t, uint64_t> version;
    priority_queue<HeapEntry, vector<HeapEntry>, Compare> open;

    uint64_t start{}, goal{};
    double km = 0.0;
    uint64_t stamp = 1;

    // Weights for the scalar planning objective.
    double wCost = 1.0;
    double wSafety = 2.0;
    double wReliability = 1.0;

    double nearestBadDistance(uint64_t u) const {
        if (bad.empty()) return INF;
        auto it = stateMap.find(u);
        if (it == stateMap.end()) return INF;
        double best = INF;
        for (uint64_t b : bad) {
            auto ib = stateMap.find(b);
            if (ib == stateMap.end()) continue;
            const auto &a = it->second.embedding;
            const auto &bb = ib->second.embedding;
            double sum = 0.0;
            size_t d = min(a.size(), bb.size());
            for (size_t i = 0; i < d; ++i) {
                double x = a[i] - bb[i];
                sum += x*x;
            }
            // If dimensions differ, treat missing coordinates as 0.
            if (a.size() > d) for (size_t i=d;i<a.size();++i) sum += a[i]*a[i];
            if (bb.size() > d) for (size_t i=d;i<bb.size();++i) sum += bb[i]*bb[i];
            best = min(best, sqrt(sum));
        }
        return best;
    }

    double heuristic(uint64_t a, uint64_t b) const {
        auto ia = stateMap.find(a), ib = stateMap.find(b);
        if (ia == stateMap.end() || ib == stateMap.end()) return 0.0;
        const auto &x = ia->second.embedding;
        const auto &y = ib->second.embedding;
        size_t d = min(x.size(), y.size());
        double sum = 0.0;
        for (size_t i=0;i<d;++i) {
            double z=x[i]-y[i]; sum += z*z;
        }
        return sqrt(sum);
    }

    double edgeWeight(uint64_t tid) const {
        const auto &t = trMap.at(tid);
        if (!t.available || bad.count(t.to)) return INF;

        // Safety term: small distance to bad states => large penalty.
        // It is bounded and nonnegative, so D* Lite remains valid.
        double d = nearestBadDistance(t.to);
        double safetyPenalty = 1.0 / (1.0 + d);

        // Reliability is rewarded by reducing penalty when reliability is high.
        double reliabilityPenalty = 1.0 - max(0.0, min(1.0, t.reliability));

        return wCost * max(0.0, t.cost)
             + wSafety * safetyPenalty
             + wReliability * reliabilityPenalty;
    }

    double getG(uint64_t u) const {
        auto it=g.find(u); return it==g.end()?INF:it->second;
    }
    double getRhs(uint64_t u) const {
        auto it=rhs.find(u); return it==rhs.end()?INF:it->second;
    }
    void setG(uint64_t u,double v){g[u]=v;}
    void setRhs(uint64_t u,double v){rhs[u]=v;}

    pair<double,double> calculateKey(uint64_t u) const {
        double m = min(getG(u), getRhs(u));
        return {m + heuristic(start,u) + km, m};
    }

    bool keyLess(const pair<double,double>& a,const pair<double,double>& b) const {
        if (a.first < b.first-EPS) return true;
        if (a.first > b.first+EPS) return false;
        return a.second < b.second-EPS;
    }

    void pushOpen(uint64_t u) {
        auto [k1,k2]=calculateKey(u);
        uint64_t v=++version[u];
        open.push({k1,k2,u,v});
    }

    bool isStale(const HeapEntry &e) const {
        auto it=version.find(e.u);
        if (it==version.end() || it->second!=e.stamp) return true;
        auto [k1,k2]=calculateKey(e.u);
        return fabs(k1-e.k1)>1e-8 || fabs(k2-e.k2)>1e-8;
    }

    uint64_t bestPredecessor(uint64_t u) const {
        double best=INF; uint64_t bestT=0;
        auto it=incoming.find(u);
        if (it==incoming.end()) return 0;
        for(uint64_t tid:it->second){
            double c=edgeWeight(tid);
            if(c>=INF/2) continue;
            const auto &t=trMap.at(tid);
            double val=getG(t.from)+c;
            if(val<best-EPS){best=val;bestT=tid;}
        }
        return bestT;
    }

    void updateVertex(uint64_t u) {
        if (u!=goal) {
            double best=INF;
            auto it=outgoing.find(u);
            if(it!=outgoing.end()){
                for(uint64_t tid:it->second){
                    double c=edgeWeight(tid);
                    if(c>=INF/2) continue;
                    const auto &t=trMap.at(tid);
                    best=min(best,c+getG(t.to));
                }
            }
            setRhs(u,best);
        }
        if(fabs(getG(u)-getRhs(u))>EPS) pushOpen(u);
        else ++version[u]; // invalidate older queue copies
    }

    void initialize(bool resetAll=true) {
        if(resetAll){
            g.clear(); rhs.clear(); version.clear();
            while(!open.empty()) open.pop();
            km=0.0;
        }
        start=p.initialState;
        goal=p.goalState;
        for(const auto &s:p.states) {
            stateMap[s.id]=s;
            if(resetAll){g[s.id]=INF;rhs[s.id]=INF;version[s.id]=0;}
        }
        for(const auto &t:p.transitions) trMap[t.id]=t;
        rebuildAdjacency();
        bad.clear(); for(auto b:p.badStates) bad.insert(b);

        rhs[goal]=0.0;
        pushOpen(goal);
    }

    void rebuildAdjacency() {
        outgoing.clear(); incoming.clear();
        for(const auto &kv:trMap) {
            const auto &t=kv.second;
            outgoing[t.from].push_back(t.id);
            incoming[t.to].push_back(t.id);
        }
    }

    void computeShortestPath(size_t &expanded) {
        while(!open.empty()) {
            auto top=open.top();
            auto startKey=calculateKey(start);
            if(!isStale(top) &&
               !keyLess({top.k1,top.k2},startKey) &&
               fabs(getRhs(start)-getG(start))<=EPS) break;
            open.pop();
            if(isStale(top)) continue;

            uint64_t u=top.u;
            double gu=getG(u), ru=getRhs(u);
            if(gu>ru) {
                setG(u,ru);
                auto it=incoming.find(u);
                if(it!=incoming.end())
                    for(uint64_t tid:it->second) updateVertex(trMap.at(tid).from);
            } else {
                setG(u,INF);
                updateVertex(u);
                auto it=incoming.find(u);
                if(it!=incoming.end())
                    for(uint64_t tid:it->second) updateVertex(trMap.at(tid).from);
            }
            ++expanded;
        }
    }

    vector<uint64_t> reconstructTransitions() const {
        vector<uint64_t> path;
        if(getG(start)>=INF/2) return path;
        uint64_t u=start;
        unordered_set<uint64_t> seen;
        while(u!=goal) {
            if(seen.count(u)) { path.clear(); return path; }
            seen.insert(u);
            double best=INF; uint64_t bestT=0;
            auto it=outgoing.find(u);
            if(it!=outgoing.end()) {
                for(uint64_t tid:it->second) {
                    double c=edgeWeight(tid);
                    if(c>=INF/2) continue;
                    const auto &t=trMap.at(tid);
                    double val=c+getG(t.to);
                    if(val<best-EPS) {best=val;bestT=tid;}
                }
            }
            if(!bestT || best>=INF/2) {path.clear(); return path;}
            path.push_back(bestT);
            u=trMap.at(bestT).to;
        }
        return path;
    }

public:
    explicit DStarLite(double wc=1.0,double ws=2.0,double wr=1.0)
        : wCost(wc),wSafety(ws),wReliability(wr) {}

    PlanningResult plan(const PlanningProblem& problem) override {
        auto t0 = chrono::steady_clock::now();

        p=problem;
        stateMap.clear(); trMap.clear();
        initialize(true);
        size_t expanded=0;
        computeShortestPath(expanded);

        PlanningResult r;
        r.exploredStates=expanded;
        auto tpath=reconstructTransitions();
        if(start==goal) tpath.clear();
        if(start!=goal && tpath.empty()) {
            r.success=false;
            r.planningTimeMs = chrono::duration<double,milli>(chrono::steady_clock::now()-t0).count();
            r.memoryBytes = estimateMemoryBytes();
            return r;
        }

        r.success=true;
        r.transitionPath=tpath;
        r.statePath.push_back(start);
        double rawCost=0, minDist=INF, relSum=0;
        uint64_t u=start;
        minDist=min(minDist,nearestBadDistance(u));
        for(uint64_t tid:tpath){
            const auto &t=trMap.at(tid);
            rawCost+=t.cost;
            relSum+=t.reliability;
            u=t.to; r.statePath.push_back(u);
            minDist=min(minDist,nearestBadDistance(u));
        }
        r.totalCost=rawCost;
        r.safetyScore=minDist;
        r.cumulativeReliability=relSum;
        r.objectiveScore=1000.0 - rawCost + 10.0*(minDist==INF?0:minDist) + 5.0*relSum;
        r.planningTimeMs = chrono::duration<double,milli>(chrono::steady_clock::now()-t0).count();
        r.memoryBytes = estimateMemoryBytes();
        return r;
    }

    size_t estimateMemoryBytes() const {
        return g.size()*sizeof(double)*2 + trMap.size()*sizeof(Transition)
             + stateMap.size()*sizeof(State) + bad.size()*sizeof(uint64_t);
    }

    // Change goal without discarding the current graph values.
    void updateGoal(uint64_t newGoal, size_t &expanded, double &replanMs) {
        auto t0 = chrono::steady_clock::now();
        // Goal changes alter the value function. Keep the graph/adjacency,
        // but reinitialize only g/rhs/open values (no graph rebuild).
        goal=newGoal;
        g.clear(); rhs.clear(); version.clear();
        while(!open.empty()) open.pop();
        for(const auto &s:p.states) {
            g[s.id]=INF; rhs[s.id]=INF; version[s.id]=0;
        }
        rhs[goal]=0.0;
        pushOpen(goal);
        computeShortestPath(expanded);
        replanMs = chrono::duration<double,milli>(chrono::steady_clock::now()-t0).count();
    }

    // Change current start (e.g., agent moved).
    void moveStart(uint64_t newStart, size_t &expanded, double &replanMs) {
        auto t0 = chrono::steady_clock::now();
        km += heuristic(start,newStart);
        start=newStart;
        computeShortestPath(expanded);
        replanMs = chrono::duration<double,milli>(chrono::steady_clock::now()-t0).count();
    }

    // Change availability/cost/safety/reliability of an existing transition.
    void updateTransition(uint64_t tid, const Transition &newT, size_t &expanded, double &replanMs) {
        auto t0 = chrono::steady_clock::now();
        auto old=trMap[tid];
        trMap[tid]=newT;
        // For an edge u->v, the one-step lookahead (rhs) of u changes.
        if(old.from != newT.from || old.to != newT.to) {
            rebuildAdjacency();
        }
        updateVertex(old.from);
        updateVertex(newT.from);
        computeShortestPath(expanded);
        replanMs = chrono::duration<double,milli>(chrono::steady_clock::now()-t0).count();
    }

    // Add a new transition.
    void addTransition(const Transition &t, size_t &expanded, double &replanMs) {
        auto t0 = chrono::steady_clock::now();
        trMap[t.id]=t;
        outgoing[t.from].push_back(t.id);
        incoming[t.to].push_back(t.id);

        // Adding u -> v changes rhs(u).
        updateVertex(t.from);
        computeShortestPath(expanded);
        replanMs = chrono::duration<double,milli>(chrono::steady_clock::now()-t0).count();
    }

    PlanningResult currentResult() const {
        PlanningResult r;
        auto tpath=reconstructTransitions();
        r.success=(start==goal || !tpath.empty());
        r.transitionPath=tpath;
        r.statePath.push_back(start);
        double raw=0, mind=INF, rel=0; uint64_t u=start;
        mind=min(mind,nearestBadDistance(u));
        for(auto tid:tpath){
            const auto&t=trMap.at(tid);
            raw+=t.cost; rel+=t.reliability;
            u=t.to; r.statePath.push_back(u);
            mind=min(mind,nearestBadDistance(u));
        }
        r.totalCost=raw;
        r.safetyScore=mind;
        r.cumulativeReliability=rel;
        r.memoryBytes=estimateMemoryBytes();
        return r;
    }
};

// Helper functions for the six required test cases.
State S(uint64_t id, double x, double y)
{
    State s;
    s.id = id;
    s.embedding.push_back(x);
    s.embedding.push_back(y);
    return s;
}

Transition E(uint64_t id, uint64_t f, uint64_t t, double c,
             double safety=1, double rel=.95, bool av=true)
{
    Transition e;
    e.id = id;
    e.from = f;
    e.to = t;
    e.cost = c;
    e.safety = safety;
    e.reliability = rel;
    e.available = av;
    return e;
}

void printResult(const string& name,const PlanningResult&r){
    cout << name << "\n";
    cout << "  Success: " << boolalpha << r.success << "\n";
    cout << "  State path: ";
    for(size_t i=0;i<r.statePath.size();++i){
        if(i) cout<<" -> ";
        cout<<r.statePath[i];
    }
    cout << "\n  Transition ID path: ";
    for(size_t i=0;i<r.transitionPath.size();++i){
        if(i) cout<<" -> ";
        cout<<r.transitionPath[i];
    }
    cout << "\n  Total path cost: " << fixed << setprecision(3) << r.totalCost;
    if (r.safetyScore == numeric_limits<double>::infinity())
        cout << "\n  Minimum safety distance: N/A (no bad states)";
    else
        cout << "\n  Minimum safety distance: " << fixed << setprecision(3) << r.safetyScore;
    cout << "\n  Cumulative reliability: " << fixed << setprecision(3)
         << r.cumulativeReliability
         << "\n  Explored states: " << r.exploredStates
         << "\n  Planning/replanning time: " << setprecision(4) << r.planningTimeMs << " ms"
         << "\n  Memory usage: " << r.memoryBytes << " bytes\n\n";
}

PlanningProblem makeBase(){
    PlanningProblem p;
    p.initialState=1; p.goalState=6;
    p.states={S(1,0,0),S(2,1,0),S(3,2,0),S(4,0,1),S(5,1,1),S(6,2,1),S(7,1,0.05)};
    p.badStates={7};
    p.transitions={
        E(1,1,2,1), E(2,2,3,1), E(3,3,6,1),
        E(4,1,4,1.2), E(5,4,5,1.2), E(6,5,6,1.2),
        E(7,2,7,0.2), E(8,7,6,0.2)
    };
    return p;
}

int main(){
    // 1. Basic reachability
    {
        PlanningProblem p;
        p.initialState=1;p.goalState=4;
        p.states={S(1,0,0),S(2,1,0),S(3,2,0),S(4,3,0)};
        p.transitions={E(1,1,2,1),E(2,2,3,1),E(3,3,4,1)};
        DStarLite d;
        printResult("Test Case 1: Basic Reachability",d.plan(p));
    }

    // 2. Bad state avoidance
    {
        PlanningProblem p;
        p.initialState=1;p.goalState=4;
        p.states={S(1,0,0),S(2,1,0),S(3,2,0),S(4,3,0),S(5,1,1)};
        p.badStates={3};
        p.transitions={E(1,1,2,1),E(2,2,3,.1),E(3,3,4,.1),
                       E(4,1,5,1.2),E(5,5,4,1.2)};
        DStarLite d;
        printResult("Test Case 2: Bad State Avoidance",d.plan(p));
    }

    // 3. Safety margin: direct path is cheap but near bad state; upper path is safer.
    {
        PlanningProblem p;
        p.initialState=1;p.goalState=4;
        p.states={S(1,0,0),S(2,1,0.1),S(3,2,0.1),S(4,3,0),
                  S(5,1,2),S(6,2,2),S(7,1.5,0)};
        p.badStates={7};
        p.transitions={E(1,1,2,.5),E(2,2,3,.5),E(3,3,4,.5),
                       E(4,1,5,1.0),E(5,5,6,1.0),E(6,6,4,1.0)};
        DStarLite d(1.0,8.0,1.0);
        printResult("Test Case 3: Safety Margin",d.plan(p));
    }

    // 4. Dynamic transition: A->G is removed after initial planning.
    {
        PlanningProblem p;
        p.initialState=1;p.goalState=4;
        p.states={S(1,0,0),S(2,1,0),S(3,1,1),S(4,2,0)};
        p.transitions={E(1,1,2,1),E(2,2,4,1),
                       E(3,1,3,1.4),E(4,3,4,1.4)};
        DStarLite d;
        auto before=d.plan(p);
        printResult("Test Case 4A: Before transition removal",before);

        DStarLite d2;
        d2.plan(p);
        size_t expanded=0;
        double replanMs=0;
        auto t=p.transitions[1]; // 2 -> 4
        t.available=false;
        d2.updateTransition(t.id,t,expanded,replanMs);
        auto after=d2.currentResult();
        after.exploredStates=expanded;
        after.planningTimeMs=replanMs;
        printResult("Test Case 4B: After transition removal",after);
    }

    // 5. Goal update without rebuilding the whole graph.
    {
        PlanningProblem p=makeBase();
        DStarLite d;
        d.plan(p);
        size_t expanded=0;
        double replanMs=0;
        d.updateGoal(5,expanded,replanMs);
        auto r=d.currentResult(); r.exploredStates=expanded; r.planningTimeMs=replanMs;
        printResult("Test Case 5: Goal Update (new goal = state 5)",r);
    }

    // 6. Add shortcut.
    {
        PlanningProblem p=makeBase();
        DStarLite d;
        d.plan(p);
        size_t expanded=0;
        double replanMs=0;
        // A safe shortcut is inserted from state 4 directly to the goal.
        d.addTransition(E(99,4,6,0.2,1.0,.99,true),expanded,replanMs);
        auto r=d.currentResult(); r.exploredStates=expanded; r.planningTimeMs=replanMs;
        printResult("Test Case 6: Transition Addition (safe shortcut 4->6)",r);
    }
}
