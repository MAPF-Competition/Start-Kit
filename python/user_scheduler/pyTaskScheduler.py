"""
pyTaskScheduler.py — Default naive Python Task Scheduler.

Greedily assigns unassigned tasks to free agents in iteration order.
"""
import MAPF


class pyTaskScheduler:
    def __init__(self):
        self.env = None
        self.free_agents = set()
        self.free_tasks = set()

    def initialize(self, preprocess_time_limit: int):
        pass

    def plan(self, time_limit: int) -> list:
        self.free_agents.update(self.env.new_freeagents)
        self.free_tasks.update(self.env.new_tasks)

        schedule = list(self.env.curr_task_schedule)
        assigned_agents = []
        assigned_tasks = []

        for agent_id in sorted(self.free_agents):
            if not self.free_tasks:
                break
            task_id = min(self.free_tasks)  # pick lowest available task id
            schedule[agent_id] = task_id
            assigned_agents.append(agent_id)
            assigned_tasks.append(task_id)
            self.free_tasks.discard(task_id)

        for a in assigned_agents:
            self.free_agents.discard(a)

        return schedule
