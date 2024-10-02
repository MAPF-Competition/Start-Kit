import MAPF
import numpy
from typing import Dict, List, Tuple,Set
import datetime


class pyTaskScheduler:
    def __init__(self, env):
        self.env=env
  
  
    def initialize(self, preprocess_time_limit:int):
        """
        Initialize the task scheduler
        """
        print("python scheduler!!")
    
    def plan(self,time_limit:int):
        """
        Plan the task schedule
        The time limit (ms) starts from the time when the Entr::compute() was called. 
        You could read start time from self.env.plan_start_time, 
        which is a datetime.timedelta measures the time from the clocks epoch to start time.
        This means that the function should return the proposed schedule before 
        datetime.datetime.fromtimestamp(0) + self.env.plan_start_time + datetime.timedelta(milliseconds=time_limit)
        """
        proposed_schedule=[None for i in range(self.env.num_of_agents)]
        i_task=0
        for i in range(self.env.num_of_agents):
            if self.env.curr_task_schedule[i]==-1:
                while i_task<len(self.env.task_pool):
                    if self.env.task_pool[i_task].agent_assigned==-1:
                        proposed_schedule[i]=self.env.task_pool[i_task].task_id
                        self.env.task_pool[i_task].agent_assigned=i
                        break
                    i_task+=1
            else:
                proposed_schedule[i]=self.env.curr_task_schedule[i]
        return proposed_schedule
    