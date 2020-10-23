
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
#  Class : Probability transition
#  Description: Holds transition probabilities.
# ------------------------------------------------------------------------
class probability_transition():
    def __init__(self, arrow_map, arrow_probability_up , arrow_probability_down, arrow_probability_left, arrow_probability_right):
        self.arrow_map = arrow_map
        self.arrow_probability_up = arrow_probability_up
        self.arrow_probability_down = arrow_probability_down
        self.arrow_probability_left = arrow_probability_left
        self.arrow_probability_right = arrow_probability_right

    # assume desired_state is some direction.
    def compute_probability_left_arrow(self, desired_state, action):
        if action == "U":
            if desired_state == "U":
                result = self.arrow_probability_left[0][0]
            elif desired_state == "R":
                result = self.arrow_probability_left[0][1]
            elif desired_state == "L":
                result = self.arrow_probability_left[0][2]
            elif desired_state == "D":
                result = self.arrow_probability_left[0][3]
        elif action == "R":
            if desired_state == "U":
                result = self.arrow_probability_left[1][0]
            elif desired_state == "R":
                result = self.arrow_probability_left[1][1]
            elif desired_state == "L":
                result = self.arrow_probability_left[1][2]
            elif desired_state == "D":
                result = self.arrow_probability_left[1][3]
        elif action == "L":
            if desired_state == "U":
                result = self.arrow_probability_left[2][0]
            elif desired_state == "R":
                result = self.arrow_probability_left[2][1]
            elif desired_state == "L":
                result = self.arrow_probability_left[2][2]
            elif desired_state == "D":
                result = self.arrow_probability_left[2][3]
        elif action == "D":
            if desired_state == "U":
                result = self.arrow_probability_left[3][0]
            elif desired_state == "R":
                result = self.arrow_probability_left[3][1]
            elif desired_state == "L":
                result = self.arrow_probability_left[3][2]
            elif desired_state == "D":
                result = self.arrow_probability_left[3][3]

    # assume desired_state is some direction.
    def compute_probability_right_arrow(self, desired_state, action):
        if action == "U":
            if desired_state == "U":
                result = self.arrow_probability_right[0][0]
            elif desired_state == "R":
                result = self.arrow_probability_right[0][1]
            elif desired_state == "L":
                result = self.arrow_probability_right[0][2]
            elif desired_state == "D":
                result = self.arrow_probability_right[0][3]
        elif action == "R":
            if desired_state == "U":
                result = self.arrow_probability_right[1][0]
            elif desired_state == "R":
                result = self.arrow_probability_right[1][1]
            elif desired_state == "L":
                result = self.arrow_probability_right[1][2]
            elif desired_state == "D":
                result = self.arrow_probability_right[1][3]
        elif action == "L":
            if desired_state == "U":
                result = self.arrow_probability_right[2][0]
            elif desired_state == "R":
                result = self.arrow_probability_right[2][1]
            elif desired_state == "L":
                result = self.arrow_probability_right[2][2]
            elif desired_state == "D":
                result = self.arrow_probability_right[2][3]
        elif action == "D":
            if desired_state == "U":
                result = self.arrow_probability_right[3][0]
            elif desired_state == "R":
                result = self.arrow_probability_right[3][1]
            elif desired_state == "L":
                result = self.arrow_probability_right[3][2]
            elif desired_state == "D":
                result = self.arrow_probability_right[3][3]


    # assume desired_state is some direction.
    def compute_probability_down_arrow(self, desired_state, action):
        if action == "U":
            if desired_state == "U":
                result = self.arrow_probability_down[0][0]
            elif desired_state == "R":
                result = self.arrow_probability_down[0][1]
            elif desired_state == "L":
                result = self.arrow_probability_down[0][2]
            elif desired_state == "D":
                result = self.arrow_probability_down[0][3]
        elif action == "R":
            if desired_state == "U":
                result = self.arrow_probability_down[1][0]
            elif desired_state == "R":
                result = self.arrow_probability_down[1][1]
            elif desired_state == "L":
                result = self.arrow_probability_down[1][2]
            elif desired_state == "D":
                result = self.arrow_probability_down[1][3]
        elif action == "L":
            if desired_state == "U":
                result = self.arrow_probability_down[2][0]
            elif desired_state == "R":
                result = self.arrow_probability_down[2][1]
            elif desired_state == "L":
                result = self.arrow_probability_down[2][2]
            elif desired_state == "D":
                result = self.arrow_probability_down[2][3]
        elif action == "D":
            if desired_state == "U":
                result = self.arrow_probability_down[3][0]
            elif desired_state == "R":
                result = self.arrow_probability_down[3][1]
            elif desired_state == "L":
                result = self.arrow_probability_down[3][2]
            elif desired_state == "D":
                result = self.arrow_probability_down[3][3]

    # assume desired_state is some direction.
    def compute_probability_up_arrow(self, desired_state, action):
        if action == "U":
            if desired_state == "U":
                result = self.arrow_probability_up[0][0]
            elif desired_state == "R":
                result = self.arrow_probability_up[0][1]
            elif desired_state == "L":
                result = self.arrow_probability_up[0][2]
            elif desired_state == "D":
                result = self.arrow_probability_up[0][3]
        elif action == "R":
            if desired_state == "U":
                result = self.arrow_probability_up[1][0]
            elif desired_state == "R":
                result = self.arrow_probability_up[1][1]
            elif desired_state == "L":
                result = self.arrow_probability_up[1][2]
            elif desired_state == "D":
                result = self.arrow_probability_up[1][3]
        elif action == "L":
            if desired_state == "U":
                result = self.arrow_probability_up[2][0]
            elif desired_state == "R":
                result = self.arrow_probability_up[2][1]
            elif desired_state == "L":
                result = self.arrow_probability_up[2][2]
            elif desired_state == "D":
                result = self.arrow_probability_up[2][3]
        elif action == "D":
            if desired_state == "U":
                result = self.arrow_probability_up[3][0]
            elif desired_state == "R":
                result = self.arrow_probability_up[3][1]
            elif desired_state == "L":
                result = self.arrow_probability_up[3][2]
            elif desired_state == "D":
                result = self.arrow_probability_up[3][3]

    def compute_probability(self, desired_state, current_state,action):

        direction = self.arrow_map[(current_state[0], current_state[1])]
        if direction == "U":
            probability_return = self.compute_probability_uparrow(desired_state, action)
        elif direction == "D":
            probability_return = self.compute_probability_down_arrow(desired_state, action)
        elif direction == "L":
            probability_return = self.compute_probability_left_arrow(desired_state, action)
        elif direction == "R":
            probability_return = self.compute_probability_right_arrow(desired_state, action)
        return probability_return

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

# -------------------------------------------------------------------------
#  Class : main
#  Description: Holds functions for computing reward based on states and
#               obstacles.
#
#  Variables:
#               x_init = initial state
#               goal   = goal state
#               O      = Obstacles
#               gam    = Discount factor
#               U      = Utility
#               R      = reward.
# ------------------------------------------------------------------------
def main():
    x_init = [7,1]
    goal= [2,8]
    O = [[1,1],[1,6],[3,4],[4,4],[4,5],[4,8],[5,2],[6,2],[6,6],[7,6]]
    gam = 0.95
    U = 0
    V = 0
    R = reward(O, goal)
    # Directions of arrows.
    arrow_map = {(1, 1): "0", (1, 2): "D", (1, 3): "D", (1, 4): "R", (1, 5): "U", (1, 6): "0", (1, 7): "U", (1, 8): "D",
                 (2, 1): "L", (2, 2): "R", (2, 3): "R", (2, 4): "D", (2, 5): "L", (2, 6): "L", (2, 7): "L", (2, 8): "0",
                 (3, 1): "U", (3, 2): "R", (3, 3): "D", (3, 4): "0", (3, 5): "D", (3, 6): "D", (3, 7): "L", (3, 8): "L",
                 (4, 1): "U", (4, 2): "L", (4, 3): "D", (4, 4): "0", (4, 5): "0", (4, 6): "R", (4, 7): "R", (4, 8): "0",
                 (5, 1): "R", (5, 2): "0", (5, 3): "U", (5, 4): "D", (5, 5): "R", (5, 6): "D", (5, 7): "U", (5, 8): "L",
                 (6, 1): "U", (6, 2): "0", (6, 3): "R", (6, 4): "U", (6, 5): "U", (6, 6): "0", (6, 7): "U", (6, 8): "D",
                 (7, 1): "R", (7, 2): "R", (7, 3): "R", (7, 4): "U", (7, 5): "L", (7, 6): "0", (7, 7): "U", (7, 8): "L",
                 (8, 1): "0", (8, 2): "R", (8, 3): "U", (8, 4): "L", (8, 5): "L", (8, 6): "0", (8, 7): "R", (8, 8): "U",
                 }
    # Assuming up arrow.
    # Assuming probabilities: {U, R, L, D}
    arrow_probability_up    = [[0.75, 0.1, 0.1, 0.05], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]
    arrow_probability_down  = [[0.75, 0.1, 0.1, 0.05], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]
    arrow_probability_left  = [[0.75, 0.1, 0.1, 0.05], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]
    arrow_probability_right = [[0.75, 0.1, 0.1, 0.05], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]



if __name__ == "__main__":
    main()