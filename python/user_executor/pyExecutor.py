"""
pyExecutor.py — Default naive Python Executor.

process_new_plan pushes all plan actions to staged, truncates to 10,
and propagates the terminal state from the agent's current location.
"""
import MAPF
import random

MAX_STAGED = 10
DELTA = {0: 1, 1: None, 2: -1, 3: None}  # filled in initialize


class pyExecutor:
    def __init__(self):
        self.env = None

    def initialize(self, preprocess_time_limit: int):
        DELTA[1] = self.env.cols
        DELTA[3] = -self.env.cols

    def process_new_plan(self, sync_time_limit: int, plan, staged_actions) -> list:
        # Push all plan actions, then truncate to MAX_STAGED
        for i in range(len(plan.actions)):
            for a in plan.actions[i]:
                staged_actions[i].append(a)
            if len(staged_actions[i]) > MAX_STAGED:
                del staged_actions[i][MAX_STAGED:]

        # Propagate terminal state from current location through staged actions
        predicted = []
        cols = self.env.cols
        rows = self.env.rows
        env_map = self.env.map
        for i in range(self.env.num_of_agents):
            loc = self.env.curr_states[i].location
            ori = self.env.curr_states[i].orientation
            for a in staged_actions[i]:
                if a == MAPF.Action.FW:
                    nxt = loc + DELTA[ori]
                    nr, nc = divmod(nxt, cols)
                    if 0 <= nr < rows and 0 <= nc < cols and env_map[nxt] == 0:
                        loc = nxt
                elif a == MAPF.Action.CR:
                    ori = (ori + 1) % 4
                elif a == MAPF.Action.CCR:
                    ori = (ori + 3) % 4
            predicted.append(MAPF.State(loc, -1, ori))
        return predicted

    def next_command(self, exec_time_limit: int) -> list:
        return [
            random.choice([MAPF.ExecutionCommand.GO, MAPF.ExecutionCommand.STOP])
            for _ in range(self.env.num_of_agents)
        ]
