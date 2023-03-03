import MAPF

class pyMAPFPlanner:
    def __init__(self) -> None:
        self.env=MAPF.SharedEnvironment()

    def initialize(self,preprocess_time_limit):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        pass

    def plan(self,time_limit):
        """_summary_

        Args:
            time_limit (_type_): _description_
        """
        pass


    def single_agent_plan(self,start,start_direct,end):
        pass

    def getManhattanDistance(self,loc1,loc2):
        """_summary_

        Args:
            loc1 (_type_): _description_
            loc2 (_type_): _description_
        """
        pass

    def getNeighbors(self,location,direction):
        """_summary_

        Args:
            location (_type_): _description_
            direction (_type_): _description_
        """
        pass


    def validateMove(self,loc,loc2):
        """_summary_

        Args:
            loc (_type_): _description_
            loc2 (_type_): _description_
        """
        pass

if __name__=="__main__":
    test_planner=pyMAPFPlanner()
    print("done!")