#include <iostream>
#include <sstream>

#include "httplib.h"
#include "LPAStarPlanner.h"

int main()
{
    httplib::Server server;

    server.Get("/plan",
        [](const httplib::Request& req,
           httplib::Response& res)
        {
            // Create states
            State start(1, {0.0, 0.0});
            State safeA(2, {2.0, 1.0});
            State badState(3, {2.0, -1.0});
            State goal(4, {4.0, 0.0});

            // Create transitions
            Transition startToA(
                1, 1, 2,
                3.0, 0.95, 0.98
            );

            Transition AToGoal(
                2, 2, 4,
                4.0, 0.90, 0.95
            );

            Transition startToBad(
                3, 1, 3,
                1.0, 0.20, 0.80
            );

            Transition badToGoal(
                4, 3, 4,
                1.0, 0.10, 0.70
            );

            // Create planning problem
            PlanningProblem problem;

            problem.initialState = 1;
            problem.goalState = 4;

            problem.states = {
                start,
                safeA,
                badState,
                goal
            };

            problem.transitions = {
                startToA,
                AToGoal,
                startToBad,
                badToGoal
            };

            // State 3 is unsafe
            problem.badStates = {
                3
            };

            // Run planner
            LPAStarPlanner planner;

            PlanningResult result =
                planner.plan(problem);

            // Create JSON response
            std::stringstream json;

            json << "{";

            json << "\"success\":"
                 << (result.success ? "true" : "false")
                 << ",";

            json << "\"path\":[";

            for (size_t i = 0;
                 i < result.statePath.size();
                 ++i)
            {
                json << result.statePath[i];

                if (i + 1 <
                    result.statePath.size())
                {
                    json << ",";
                }
            }

            json << "],";

            json << "\"cost\":"
                 << result.totalCost
                 << ",";

            json << "\"safety\":"
                 << result.safetyScore;

            json << "}";

            res.set_content(
                json.str(),
                "application/json"
            );
        });

    std::cout
        << "Safe Semantic Planner Server\n";

    std::cout
        << "Running at http://localhost:8080\n";

    server.listen(
        "0.0.0.0",
        8080
    );

    return 0;
}