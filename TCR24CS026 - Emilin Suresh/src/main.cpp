#include "../include/planner.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>


/*
 * Helper function to print a separator.
 */
void printSeparator()
{
    std::cout
        << "------------------------------------------------------------\n";
}


/*
 * Print a path.
 */
void printPath(
    const PlanningProblem& problem,
    const PlanningResult& result)
{
    if (!result.success)
    {
        std::cout
            << "No valid path found.\n";

        return;
    }

    for (std::size_t i = 0;
         i < result.statePath.size();
         ++i)
    {
        std::uint64_t id =
            result.statePath[i];

        std::string name =
            std::to_string(id);

        for (const State& state :
             problem.states)
        {
            if (state.id == id)
            {
                name = state.name;
                break;
            }
        }

        std::cout << name;

        if (i + 1 <
            result.statePath.size())
        {
            std::cout << " -> ";
        }
    }

    std::cout << '\n';
}


/*
 * Print complete planning result.
 */
void printResult(
    const PlanningProblem& problem,
    const PlanningResult& result)
{
    std::cout
        << "Success                 : "
        << (result.success ? "YES" : "NO")
        << '\n';

    std::cout
        << "State path              : ";

    printPath(problem, result);

    std::cout
        << std::fixed
        << std::setprecision(4);

    std::cout
        << "Total path cost         : "
        << result.totalCost
        << '\n';

    std::cout
        << "Minimum safety distance : "
        << result.minimumSafetyDistance
        << '\n';

    std::cout
        << "Safety score            : "
        << result.safetyScore
        << '\n';

    std::cout
        << "Cumulative reliability  : "
        << result.cumulativeReliability
        << '\n';

    std::cout
        << "Explored states         : "
        << result.exploredStates
        << '\n';

    std::cout
        << "Planning time (ms)      : "
        << result.planningTimeMs
        << '\n';

    std::cout
        << "Replanning time (ms)    : "
        << result.replanningTimeMs
        << '\n';

    std::cout
        << "Estimated memory (bytes): "
        << result.memoryUsageBytes
        << '\n';
}


/*
 * Count bad states in the final path.
 */
int countBadStates(
    const PlanningProblem& problem,
    const PlanningResult& result)
{
    int count = 0;

    for (std::uint64_t id :
         result.statePath)
    {
        for (std::uint64_t bad :
             problem.badStates)
        {
            if (id == bad)
            {
                count++;
            }
        }
    }

    return count;
}


/*
 * Create a state.
 */
State makeState(
    std::uint64_t id,
    double x,
    double y,
    const std::string& name)
{
    return State(
        id,
        {x, y},
        name
    );
}


/*
 * Test Case 1
 *
 * S -> A -> B -> G
 */
PlanningProblem createTestCase1()
{
    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {

        makeState(
            0, 0.0, 0.0, "S"
        ),

        makeState(
            1, 1.0, 0.0, "A"
        ),

        makeState(
            2, 2.0, 0.0, "B"
        ),

        makeState(
            3, 3.0, 0.0, "G"
        )
    };


    problem.transitions = {

        Transition(
            0,
            0, 1,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            1,
            1, 2,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            2,
            2, 3,
            1.0,
            1.0,
            0.95,
            true
        )
    };


    return problem;
}


/*
 * Test Case 2
 *
 * S -> A -> X -> G
 * S -> C -> D -> G
 *
 * X is bad.
 */
PlanningProblem createTestCase2()
{
    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 5;

    problem.states = {

        makeState(
            0, 0.0, 0.0, "S"
        ),

        makeState(
            1, 1.0, 1.0, "A"
        ),

        makeState(
            2, 2.0, 1.0, "X_BAD"
        ),

        makeState(
            3, 1.0, -1.0, "C"
        ),

        makeState(
            4, 2.0, -1.0, "D"
        ),

        makeState(
            5, 3.0, 0.0, "G"
        )
    };


    problem.badStates = {
        2
    };


    problem.transitions = {

        /*
         * Bad path.
         */
        Transition(
            0,
            0, 1,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            1,
            1, 2,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            2,
            2, 5,
            1.0,
            1.0,
            0.95,
            true
        ),


        /*
         * Safe path.
         */
        Transition(
            3,
            0, 3,
            2.0,
            1.0,
            0.98,
            true
        ),

        Transition(
            4,
            3, 4,
            1.0,
            1.0,
            0.98,
            true
        ),

        Transition(
            5,
            4, 5,
            1.0,
            1.0,
            0.98,
            true
        )
    };


    return problem;
}


/*
 * Test Case 3
 *
 * Two valid paths:
 *
 * Path 1:
 * S -> NEAR -> G
 *
 * Path 2:
 * S -> SAFE1 -> SAFE2 -> G
 *
 * Path 1 has lower base cost but passes
 * close to a bad state.
 */
PlanningProblem createTestCase3()
{
    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 5;

    problem.states = {

        makeState(
            0, 0.0, 0.0, "S"
        ),

        makeState(
            1, 1.0, 0.0, "NEAR"
        ),

        makeState(
            2, 2.0, 0.0, "BAD"
        ),

        makeState(
            3, 1.0, 3.0, "SAFE1"
        ),

        makeState(
            4, 2.0, 3.0, "SAFE2"
        ),

        makeState(
            5, 3.0, 0.0, "G"
        )
    };


    problem.badStates = {
        2
    };


    problem.transitions = {

        /*
         * Cheap but dangerous path.
         */
        Transition(
            0,
            0, 1,
            1.0,
            0.2,
            0.90,
            true
        ),

        Transition(
            1,
            1, 5,
            1.0,
            0.2,
            0.90,
            true
        ),


        /*
         * More expensive but safer path.
         */
        Transition(
            2,
            0, 3,
            2.0,
            0.9,
            0.99,
            true
        ),

        Transition(
            3,
            3, 4,
            2.0,
            0.9,
            0.99,
            true
        ),

        Transition(
            4,
            4, 5,
            2.0,
            0.9,
            0.99,
            true
        )
    };


    return problem;
}


/*
 * Test Case 4
 *
 * Initially:
 *
 * S -> A -> G
 *
 * Then A -> G becomes unavailable.
 */
PlanningProblem createTestCase4()
{
    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 4;

    problem.states = {

        makeState(
            0, 0.0, 0.0, "S"
        ),

        makeState(
            1, 1.0, 1.0, "A"
        ),

        makeState(
            2, 1.0, -1.0, "C"
        ),

        makeState(
            3, 2.0, -1.0, "D"
        ),

        makeState(
            4, 3.0, 0.0, "G"
        )
    };


    problem.transitions = {

        /*
         * Initial best path.
         */
        Transition(
            0,
            0, 1,
            1.0,
            1.0,
            0.98,
            true
        ),

        Transition(
            1,
            1, 4,
            1.0,
            1.0,
            0.98,
            true
        ),


        /*
         * Alternative.
         */
        Transition(
            2,
            0, 2,
            2.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            3,
            2, 3,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            4,
            3, 4,
            1.0,
            1.0,
            0.95,
            true
        )
    };


    return problem;
}


/*
 * Test Case 5
 *
 * Goal changes during execution.
 */
PlanningProblem createTestCase5()
{
    PlanningProblem problem;

    problem.initialState = 0;

    /*
     * Initial goal.
     */
    problem.goalState = 5;

    problem.states = {

        makeState(
            0, 0.0, 0.0, "S"
        ),

        makeState(
            1, 1.0, 0.0, "A"
        ),

        makeState(
            2, 2.0, 0.0, "B"
        ),

        makeState(
            3, 1.0, 2.0, "C"
        ),

        makeState(
            4, 2.0, 2.0, "D"
        ),

        makeState(
            5, 3.0, 0.0, "G"
        )
    };


    problem.transitions = {

        Transition(
            0,
            0, 1,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            1,
            1, 2,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            2,
            2, 5,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            3,
            0, 3,
            1.5,
            1.0,
            0.95,
            true
        ),

        Transition(
            4,
            3, 4,
            1.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            5,
            4, 5,
            1.0,
            1.0,
            0.95,
            true
        )
    };


    return problem;
}


/*
 * Test Case 6
 *
 * Initially:
 *
 * S -> C -> D -> G
 *
 * Then a new shortcut:
 *
 * S -> G
 *
 * is added.
 */
PlanningProblem createTestCase6()
{
    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 4;

    problem.states = {

        makeState(
            0, 0.0, 0.0, "S"
        ),

        makeState(
            1, 1.0, -1.0, "C"
        ),

        makeState(
            2, 2.0, -1.0, "D"
        ),

        makeState(
            3, 2.0, 1.0, "A"
        ),

        makeState(
            4, 3.0, 0.0, "G"
        )
    };


    problem.transitions = {

        Transition(
            0,
            0, 1,
            2.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            1,
            1, 2,
            2.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            2,
            2, 4,
            2.0,
            1.0,
            0.95,
            true
        ),


        /*
         * Another non-optimal path.
         */
        Transition(
            3,
            0, 3,
            3.0,
            1.0,
            0.95,
            true
        ),

        Transition(
            4,
            3, 4,
            3.0,
            1.0,
            0.95,
            true
        )
    };


    return problem;
}


/*
 * Run a normal test case.
 */
PlanningResult runTest(
    int testNumber,
    PlanningProblem& problem,
    Planner& planner)
{
    printSeparator();

    std::cout
        << "TEST CASE "
        << testNumber
        << '\n';

    printSeparator();

    PlanningResult result =
        planner.plan(problem);

    printResult(
        problem,
        result
    );

    std::cout
        << "Bad states visited     : "
        << countBadStates(
            problem,
            result
        )
        << '\n';

    return result;
}


/*
 * Test Case 4 dynamic demonstration.
 */
void runDynamicTransitionTest()
{
    printSeparator();

    std::cout
        << "TEST CASE 4: DYNAMIC TRANSITION\n";

    printSeparator();


    PlanningProblem problem =
        createTestCase4();

    Planner planner;


    /*
     * Initial planning.
     */
    std::cout
        << "\nInitial environment:\n";

    PlanningResult initial =
        planner.plan(problem);

    printResult(
        problem,
        initial
    );


    /*
     * Disable transition A -> G.
     *
     * Transition ID = 1.
     */
    std::cout
        << "\nUpdating environment...\n";

    planner.updateTransition(
        problem,
        1,
        false
    );


    /*
     * Replan.
     */
    auto start =
        std::chrono::high_resolution_clock::now();

    PlanningResult replanned =
        planner.plan(problem);

    auto end =
        std::chrono::high_resolution_clock::now();


    double replanningMs =
        std::chrono::duration<double,
                              std::milli>(
            end - start
        ).count();

    replanned.replanningTimeMs =
        replanningMs;


    std::cout
        << "\nAfter transition A -> G becomes unavailable:\n";

    printResult(
        problem,
        replanned
    );
}


/*
 * Test Case 5 dynamic goal demonstration.
 */
void runGoalUpdateTest()
{
    printSeparator();

    std::cout
        << "TEST CASE 5: GOAL UPDATE\n";

    printSeparator();


    PlanningProblem problem =
        createTestCase5();

    Planner planner;


    /*
     * First goal = G.
     */
    std::cout
        << "\nInitial goal = G\n";

    PlanningResult first =
        planner.plan(problem);

    printResult(
        problem,
        first
    );


    /*
     * Change goal to D.
     *
     * D has state ID 4.
     */
    std::cout
        << "\nChanging goal to D...\n";

    planner.updateGoal(
        problem,
        4
    );


    auto start =
        std::chrono::high_resolution_clock::now();

    PlanningResult second =
        planner.plan(problem);

    auto end =
        std::chrono::high_resolution_clock::now();


    second.replanningTimeMs =
        std::chrono::duration<double,
                              std::milli>(
            end - start
        ).count();


    std::cout
        << "\nAfter goal update:\n";

    printResult(
        problem,
        second
    );
}


/*
 * Test Case 6 dynamic transition addition.
 */
void runTransitionAdditionTest()
{
    printSeparator();

    std::cout
        << "TEST CASE 6: TRANSITION ADDITION\n";

    printSeparator();


    PlanningProblem problem =
        createTestCase6();

    Planner planner;


    /*
     * Initial planning.
     */
    std::cout
        << "\nBefore shortcut is added:\n";

    PlanningResult initial =
        planner.plan(problem);

    printResult(
        problem,
        initial
    );


    /*
     * Add shortcut S -> G.
     */
    std::cout
        << "\nAdding new shortcut S -> G...\n";

    Transition shortcut(
        100,
        0,
        4,
        0.5,
        1.0,
        0.99,
        true
    );

    planner.addTransition(
        problem,
        shortcut
    );


    /*
     * Replan.
     */
    auto start =
        std::chrono::high_resolution_clock::now();

    PlanningResult improved =
        planner.plan(problem);

    auto end =
        std::chrono::high_resolution_clock::now();


    improved.replanningTimeMs =
        std::chrono::duration<double,
                              std::milli>(
            end - start
        ).count();


    std::cout
        << "\nAfter shortcut is added:\n";

    printResult(
        problem,
        improved
    );
}


/*
 * Save a summary row to CSV.
 */
void saveCSVRow(
    std::ofstream& file,
    int testNumber,
    const PlanningProblem& problem,
    const PlanningResult& result)
{
    file
        << testNumber
        << ","
        << (result.success ? 1 : 0)
        << ","
        << countBadStates(
            problem,
            result
        )
        << ","
        << result.totalCost
        << ","
        << result.minimumSafetyDistance
        << ","
        << result.exploredStates
        << ","
        << result.planningTimeMs
        << ","
        << result.replanningTimeMs
        << ","
        << result.memoryUsageBytes
        << "\n";
}


/*
 * Main function.
 */
int main()
{
    std::cout
        << "\n"
        << "============================================================\n"
        << "       SAFE SEMANTIC PLANNER USING LPA*\n"
        << "       PCCST503 - Machine Learning\n"
        << "       Assignment 1\n"
        << "============================================================\n\n";


    /*
     * CSV output.
     */
    std::ofstream csv(
        "results/experimental_results.csv"
    );


    if (csv.is_open())
    {
        csv
            << "TestCase,"
            << "GoalSuccess,"
            << "BadStatesVisited,"
            << "TotalCost,"
            << "MinimumSafetyDistance,"
            << "ExploredStates,"
            << "PlanningTimeMs,"
            << "ReplanningTimeMs,"
            << "MemoryBytes\n";
    }


    Planner planner;


    /*
     * ---------------------------------------------------------
     * TEST CASE 1
     * ---------------------------------------------------------
     */
    {
        PlanningProblem problem =
            createTestCase1();

        PlanningResult result =
            runTest(
                1,
                problem,
                planner
            );

        if (csv.is_open())
        {
            saveCSVRow(
                csv,
                1,
                problem,
                result
            );
        }
    }


    /*
     * ---------------------------------------------------------
     * TEST CASE 2
     * ---------------------------------------------------------
     */
    {
        PlanningProblem problem =
            createTestCase2();

        PlanningResult result =
            runTest(
                2,
                problem,
                planner
            );

        if (csv.is_open())
        {
            saveCSVRow(
                csv,
                2,
                problem,
                result
            );
        }
    }


    /*
     * ---------------------------------------------------------
     * TEST CASE 3
     * ---------------------------------------------------------
     */
    {
        PlanningProblem problem =
            createTestCase3();

        PlanningResult result =
            runTest(
                3,
                problem,
                planner
            );

        if (csv.is_open())
        {
            saveCSVRow(
                csv,
                3,
                problem,
                result
            );
        }
    }


    /*
     * ---------------------------------------------------------
     * TEST CASE 4
     * ---------------------------------------------------------
     */
    runDynamicTransitionTest();


    /*
     * ---------------------------------------------------------
     * TEST CASE 5
     * ---------------------------------------------------------
     */
    runGoalUpdateTest();


    /*
     * ---------------------------------------------------------
     * TEST CASE 6
     * ---------------------------------------------------------
     */
    runTransitionAdditionTest();


    if (csv.is_open())
    {
        csv.close();
    }


    printSeparator();

    std::cout
        << "All test cases completed.\n";

    std::cout
        << "Experimental results saved to:\n";

    std::cout
        << "results/experimental_results.csv\n";

    printSeparator();


    return 0;
}