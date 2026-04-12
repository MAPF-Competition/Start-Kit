"""
Dummy Python MAPFPlanner for testing.
Returns all-Wait plans but records how many times plan() was called
and validates env data is accessible.
"""
import MAPF


class pyMAPFPlanner:
    def __init__(self):
        self.env = None
        self.initialize_called = False
        self.plan_call_count = 0
        self.last_time_limit = 0
        self.env_checks_passed = False

    def initialize(self, preprocess_time_limit: int):
        self.initialize_called = True
        # Validate env data is readable
        assert self.env is not None, "env not set"
        assert self.env.num_of_agents > 0, "num_of_agents should be > 0"
        assert self.env.rows > 0, "rows should be > 0"
        assert self.env.cols > 0, "cols should be > 0"
        assert len(self.env.map) == self.env.rows * self.env.cols, "map size mismatch"
        self.env_checks_passed = True

    def plan(self, time_limit: int) -> list:
        self.plan_call_count += 1
        self.last_time_limit = time_limit
        # Return all-Wait plan (1 step per agent)
        num_agents = self.env.num_of_agents
        return [[MAPF.Action.W] for _ in range(num_agents)]
