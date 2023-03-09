import MAPF

class pyMAPFPlanner:
    def __init__(self,env=None) -> None:

        self.env=env
        print("pyMAPFPlanner initialized!")

    def initialize(self,preprocess_time_limit:int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        pass
        # raise NotImplementedError()

    def plan(self,time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """
        actions=[]
        print("python binding debug")
        print("env.rows=",self.env.rows,"env.cols=",self.env.cols,"env.map=",self.env.map)
        raise NotImplementedError("YOU NEED TO IMPLEMENT THE PYMAPFPLANNER!")
        return actions


if __name__=="__main__":
    test_planner=pyMAPFPlanner()
    print("done!")