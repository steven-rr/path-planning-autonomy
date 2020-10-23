
# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : p4.py
# //|
# //| Description : Solves MDP using value iteration
# //|
# //| Notes :
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////


# -------------------------------------------------------------------------
#  Class : Reward
#  Description: Holds functions for computing reward based on states and
#               obstacles.
# ------------------------------------------------------------------------
class reward():
    def __init__(self, obstacles, goal):
        self.obstacles = obstacles
        self.goal = goal
    def compute_reward(self,current_state):
        reward_out = 0
        if current_state == self.goal:
            reward_out = 0
        elif current_state in self.obstacles:
            reward_out = -10
        else:
            reward_out = -1
        return reward_out


def main():
    x_init = [7,1]
    goal= [2,8]
    O = [[1,1],[1,6],[3,4],[4,4],[4,5],[4,8],[5,2],[6,2],[6,6],[7,6]]

    U = 0
    U_ = 0
    R = reward(O, goal)



if __name__ == "__main__":
    main()