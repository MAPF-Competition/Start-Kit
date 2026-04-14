"""
Dummy Python Executor for testing.
Simple pass-through: appends plan actions to staged_actions, always GO.
"""
import MAPF


class pyExecutor:
    def __init__(self):
        self.env = None
        self.initialize_called = False
        self.process_plan_call_count = 0
        self.next_command_call_count = 0

    def initialize(self, preprocess_time_limit: int):
        self.initialize_called = True

    def process_new_plan(self, sync_time_limit: int, plan, staged_actions) -> list:
        self.process_plan_call_count += 1
        predicted_states = list(self.env.system_states)

        # Append non-wait actions to staged_actions in-place (zero-copy)
        for agent_id in range(len(plan.actions)):
            for action in plan.actions[agent_id]:
                if action != MAPF.Action.NA and action != MAPF.Action.W:
                    staged_actions[agent_id].append(action)

        return predicted_states

    def next_command(self, exec_time_limit: int) -> list:
        self.next_command_call_count += 1
        # Always GO if staged actions available, else STOP
        commands = []
        for i in range(self.env.num_of_agents):
            if len(self.env.staged_actions[i]) > 0:
                commands.append(MAPF.ExecutionCommand.GO)
            else:
                commands.append(MAPF.ExecutionCommand.STOP)
        return commands
