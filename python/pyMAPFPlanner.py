import MAPF

class pyMAPFPlanner:
    def __init__(self,env=None) -> None:

        self.env=env
        print("pyMAPFPlanner initialized!")

    def initialize(self,preprocess_time_limit):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        raise NotImplementedError()

    def plan(self,time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """
        actions=[]
        raise NotImplementedError()
        return actions


if __name__=="__main__":
    test_planner=pyMAPFPlanner()
    print("done!")