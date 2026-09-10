from dataclasses import dataclass, field
from typing import List
import math

@dataclass
class State:
  id: int
  embedding:List[float]
  def distance_to(self,other):
    return math.sqrt(sum((a-b)**2 for a,b in zip(self.embedding,other.embedding)))

@dataclass
class Transition:
  id:int
  from_id:int
  to_id:int
  cost:float
  safety:float
  reliability:float
  available: bool=True

@dataclass
class PlanningProblem:
  initial_state:int
  goal_state:int
  bad_states:List[int]
  states:List[State]
  transitions:List[Transition]

@dataclass
class PlanningResult:
  success: bool
  state_path: List[int] = field(default_factory=list)
  transition_path: List[int] = field(default_factory=list)
  total_cost: float = 0.0
  safety_score: float = 0.0
  explored_states: int = 0
  planning_time: float = 0.0