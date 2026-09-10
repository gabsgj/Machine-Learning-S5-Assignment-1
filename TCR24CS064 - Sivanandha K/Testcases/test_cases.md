# Test Cases

## Test Case 1 – Basic Reachability
Expected: `1 -> 2 -> 3 -> 4`

## Test Case 2 – Bad State Avoidance
State `4` is bad. The route through state 4 must never be selected. Expected: `1 -> 3 -> 5`.

## Test Case 3 – Safety Margin
State `5` is bad. A safer but more expensive route is expected when safety weight is increased. Expected: `1 -> 3 -> 4 -> 6`.

## Test Case 4 – Dynamic Transition
Initial route: `1 -> 2 -> 4`. Transition 2 becomes unavailable. Expected replanned route: `1 -> 3 -> 4`.

## Test Case 5 – Goal Update
Initial goal is state 4. The goal changes to state 5. Expected new route: `1 -> 5`.

## Test Case 6 – Transition Addition
Initial route: `1 -> 2 -> 3 -> 4`. A new shortcut `1 -> 4` is inserted. Expected new route: `1 -> 4`.
