#ifndef TEST_HARNESS_HPP
#define TEST_HARNESS_HPP

#include "LPAStarPlanner.hpp"
#include "DStarLitePlanner.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <cassert>

class TestHarness {
public:
    static void runAllTests();
    static void runTestCase1();
    static void runTestCase2();
    static void runTestCase3();
    static void runTestCase4();
    static void runTestCase5();
    static void runTestCase6();
    static void runBenchmarkEvaluation();

private:
    static void printHeader(const std::string& title);
    static void printResult(const std::string& testName, bool passed, const PlanningResult& res);
};

#endif // TEST_HARNESS_HPP
