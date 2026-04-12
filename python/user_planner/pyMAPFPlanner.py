"""
pyMAPFPlanner.py — Default naive Python Planner.

Each agent greedily rotates toward and moves to the neighbor cell
that minimizes Manhattan distance to its current goal.
"""
import MAPF

DELTA = {0: 1, 1: None, 2: -1, 3: None}  # filled in initialize


class pyMAPFPlanner:
    def __init__(self):
        self.env = None

    def initialize(self, preprocess_time_limit: int):
        DELTA[1] = self.env.cols
        DELTA[3] = -self.env.cols

    def plan(self, time_limit: int) -> list:
        cols = self.env.cols
        rows = self.env.rows
        env_map = self.env.map
        actions = []

        for i in range(self.env.num_of_agents):
            goals = self.env.goal_locations[i]
            if not goals:
                actions.append([MAPF.Action.W])
                continue

            goal = MAPF.pair_first(goals[0])
            s = self.env.curr_states[i]

            if s.location == goal:
                actions.append([MAPF.Action.W])
                continue

            gr, gc = divmod(goal, cols)
            r, c = divmod(s.location, cols)
            dr, dc = gr - r, gc - c

            # pick desired orientation: prefer larger axis delta
            if abs(dc) >= abs(dr):
                desired = 0 if dc > 0 else 2
            else:
                desired = 1 if dr > 0 else 3

            ori = s.orientation
            if ori == desired:
                nxt = s.location + DELTA[ori]
                nr, nc = divmod(nxt, cols)
                if 0 <= nr < rows and 0 <= nc < cols and env_map[nxt] == 0:
                    actions.append([MAPF.Action.FW])
                else:
                    actions.append([MAPF.Action.W])
            else:
                diff = (desired - ori) % 4
                actions.append([MAPF.Action.CR if diff <= 2 else MAPF.Action.CCR])

        return actions
