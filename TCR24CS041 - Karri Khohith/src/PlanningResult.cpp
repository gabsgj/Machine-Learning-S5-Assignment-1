#include "PlanningResult.h"

PlanningResult::PlanningResult()
    : success(false),
      totalCost(0.0),
      safetyScore(0.0),
      minimumSafetyDistance(0.0),
      exploredStates(0),
      planningTimeMs(0.0),
      replanningTimeMs(0.0),
      memoryUsageBytes(0) {
}