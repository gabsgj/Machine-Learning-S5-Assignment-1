from abc import ABC, abstractmethod
from models import PlanningProblem,PlanningResult

class Planner(ABC):
  @abstractmethod
  def plan(self,problem: PlanningProblem)->PlanningResult:
    raise NotImplementedError
