#pragma once
#include <string>
#include "PlanningProblem.h"
#include "PlanningResult.h"

namespace visualisation {

// Serializes the problem graph plus a planning result to a JSON file that
// the Python visualization scripts can read. The Python side never
// recomputes a path -- it only renders exactly what is written here.
void exportToJson(const std::string& path, const PlanningProblem& problem,
                   const PlanningResult& result, const std::string& label = "");

} // namespace visualisation
