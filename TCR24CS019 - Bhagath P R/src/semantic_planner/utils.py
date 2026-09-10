"""
Priority queue with key invalidation and performance instrumentation.
"""

import heapq
import time
import tracemalloc
from typing import Dict, Any, Tuple, Optional, List


class PriorityQueue:
    """
    Min-priority queue supporting O(1) membership check, O(log N) insert, update, and remove.
    Uses Python's heapq with an invalidation dictionary.
    """

    def __init__(self):
        self._heap: List[Tuple[Any, int, int]] = []
        self._entry_finder: Dict[int, Tuple[Any, int]] = {}
        self._counter: int = 0
        self._size: int = 0

    def insert(self, item: int, priority: Any) -> None:
        if item in self._entry_finder:
            self.remove(item)
        self._counter += 1
        entry = (priority, self._counter, item)
        self._entry_finder[item] = (priority, self._counter)
        heapq.heappush(self._heap, entry)
        self._size += 1

    def remove(self, item: int) -> None:
        if item in self._entry_finder:
            del self._entry_finder[item]
            self._size -= 1

    def contains(self, item: int) -> bool:
        return item in self._entry_finder

    def get_priority(self, item: int) -> Optional[Any]:
        if item in self._entry_finder:
            return self._entry_finder[item][0]
        return None

    def top_key(self) -> Tuple[float, float]:
        self._clean_heap()
        if not self._heap:
            return (float("inf"), float("inf"))
        return self._heap[0][0]

    def pop(self) -> Tuple[int, Any]:
        self._clean_heap()
        if not self._heap:
            raise KeyError("Pop from empty priority queue")
        priority, count, item = heapq.heappop(self._heap)
        del self._entry_finder[item]
        self._size -= 1
        return item, priority

    def _clean_heap(self) -> None:
        while self._heap:
            priority, count, item = self._heap[0]
            if item in self._entry_finder and self._entry_finder[item] == (priority, count):
                break
            heapq.heappop(self._heap)

    def is_empty(self) -> bool:
        self._clean_heap()
        return len(self._entry_finder) == 0

    def __len__(self) -> int:
        return self._size

    def clear(self) -> None:
        self._heap.clear()
        self._entry_finder.clear()
        self._counter = 0
        self._size = 0


class PerformanceTracker:
    """Tracks elapsed time and memory allocations for planner benchmarking."""

    def __init__(self):
        self.start_time: float = 0.0
        self.elapsed_ms: float = 0.0
        self.peak_memory_kb: float = 0.0

    def start(self):
        tracemalloc.start()
        self.start_time = time.perf_counter()

    def stop(self) -> Tuple[float, float]:
        self.elapsed_ms = (time.perf_counter() - self.start_time) * 1000.0
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        self.peak_memory_kb = peak / 1024.0
        return self.elapsed_ms, self.peak_memory_kb
