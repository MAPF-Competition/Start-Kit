"""
Dummy Python TaskScheduler for testing.
Assigns the first available task to each free agent (greedy by task_id order).
"""
import MAPF


class pyTaskScheduler:
    def __init__(self):
        self.env = None
        self.initialize_called = False
        self.plan_call_count = 0

    def initialize(self, preprocess_time_limit: int):
        self.initialize_called = True

    def plan(self, time_limit: int) -> list:
        self.plan_call_count += 1
        # Keep existing schedule, assign free agents to unassigned tasks
        schedule = list(self.env.curr_task_schedule)

        assigned_tasks = set(t for t in schedule if t != -1)

        for i in range(len(schedule)):
            if schedule[i] == -1:
                # Find first unassigned task in pool
                for task_id in sorted(self.env.task_pool.keys()):
                    if task_id not in assigned_tasks:
                        task = self.env.task_pool[task_id]
                        if not task.is_finished():
                            schedule[i] = task_id
                            assigned_tasks.add(task_id)
                            break

        return schedule
