
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
    def compute_probability_left_arrow(self, possible_actions, action):
        result = 0
        if action == "U":
            if "U" in possible_actions:
                result = self.arrow_probability_left[0][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_left[0][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_left[0][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_left[0][3] + result
        elif action == "R":
            if "U" in possible_actions:
                result = self.arrow_probability_left[1][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_left[1][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_left[1][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_left[1][3] + result
        elif action == "L":
            if "U" in possible_actions:
                result = self.arrow_probability_left[2][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_left[2][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_left[2][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_left[2][3] + result
        elif action == "D":
            if "U" in possible_actions:
                result = self.arrow_probability_left[3][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_left[3][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_left[3][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_left[3][3] + result
        return result

    # assume desired_state is some direction.
    def compute_probability_right_arrow(self, possible_actions, action):
        result = 0
        if action == "U":
            if "U" in possible_actions:
                result = self.arrow_probability_right[0][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_right[0][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_right[0][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_right[0][3] + result
        elif action == "R":
            if "U" in possible_actions:
                result = self.arrow_probability_right[1][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_right[1][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_right[1][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_right[1][3] + result
        elif action == "L":
            if "U" in possible_actions:
                result = self.arrow_probability_right[2][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_right[2][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_right[2][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_right[2][3] + result
        elif action == "D":
            if "U" in possible_actions:
                result = self.arrow_probability_right[3][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_right[3][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_right[3][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_right[3][3] + result
        return result

    # assume desired_state is some direction.
    def compute_probability_down_arrow(self, possible_actions, action):
        result = 0
        if action == "U":
            if "U" in possible_actions:
                result = self.arrow_probability_down[0][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_down[0][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_down[0][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_down[0][3] + result
        elif action == "R":
            if "U" in possible_actions:
                result = self.arrow_probability_down[1][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_down[1][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_down[1][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_down[1][3] + result
        elif action == "L":
            if "U" in possible_actions:
                result = self.arrow_probability_down[2][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_down[2][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_down[2][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_down[2][3] + result
        elif action == "D":
            if "U" in possible_actions:
                result = self.arrow_probability_down[3][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_down[3][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_down[3][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_down[3][3] + result
        return result

    # assume desired_state is some direction.
    def compute_probability_up_arrow(self, possible_actions, action):
        result = 0
        if action == "U":
            if "U" in possible_actions:
                result = self.arrow_probability_up[0][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_up[0][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_up[0][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_up[0][3] + result
        elif action == "R":
            if "U" in possible_actions:
                result = self.arrow_probability_up[1][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_up[1][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_up[1][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_up[1][3] + result
        elif action == "L":
            if "U" in possible_actions:
                result = self.arrow_probability_up[2][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_up[2][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_up[2][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_up[2][3] + result
        elif action == "D":
            if "U" in possible_actions:
                result = self.arrow_probability_up[3][0] + result
            if "R" in possible_actions:
                result = self.arrow_probability_up[3][1] + result
            if "L" in possible_actions:
                result = self.arrow_probability_up[3][2] + result
            if "D" in possible_actions:
                result = self.arrow_probability_up[3][3] + result
        return result
    # compute probability of desired state based on current state and current action.
    def compute_probability(self, possible_actions, current_state, action):

        direction = self.arrow_map[(current_state[0], current_state[1])]
        if direction == "U":
            probability_return = self.compute_probability_uparrow(possible_actions, action)
        elif direction == "D":
            probability_return = self.compute_probability_down_arrow(possible_actions, action)
        elif direction == "L":
            probability_return = self.compute_probability_left_arrow(possible_actions, action)
        elif direction == "R":
            probability_return = self.compute_probability_right_arrow(possible_actions, action)
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
    R = reward(O, goal)
    # states

    # how can i move , depending on my index.
    states = {
                 0: (1, 1),  1: (1, 2),  2: (1, 3),  3: (1, 4),  4: (1, 5),  5: (1, 6),  6: (1, 7),  7: (1, 8),
                 8: (2, 1),  9: (2, 2), 10: (2, 3), 11: (2, 4), 12: (2, 5), 13: (2, 6), 14: (2, 7), 15: (2, 8),
                16: (3, 1), 17: (3, 2), 18: (3, 3), 19: (3, 4), 20: (3, 5), 21: (3, 6), 22: (3, 7), 23: (3, 8),
                24: (4, 1), 25: (4, 2), 26: (4, 3), 27: (4, 4), 28: (4, 5), 29: (4, 6), 30: (4, 7), 31: (4, 8),
                32: (5, 1), 33: (5, 2), 34: (5, 3), 35: (5, 4), 36: (5, 5), 37: (5, 6), 38: (5, 7), 39: (5, 8),
                40: (6, 1), 41: (6, 2), 42: (6, 3), 43: (6, 4), 44: (6, 5), 45: (6, 6), 46: (6, 7), 47: (6, 8),
                48: (7, 1), 49: (7, 2), 50: (7, 3), 51: (7, 4), 52: (7, 5), 53: (7, 6), 54: (7, 7), 55: (7, 8),
                56: (8, 1), 57: (8, 2), 58: (8, 3), 59: (8, 4), 60: (8, 5), 61: (8, 6), 62: (8, 7), 63: (8, 8),
              }
    # each state has possible actions:
    possible_actions =  {
                            0:["R","D"],         1: ["L", "D", "R"],  2:["L", "D", "R"],   3: ["L", "D", "R"],   4: ["L", "D", "R"],   5: ["L", "D", "R"],   6:["L", "D", "R"],    7:["L","D"],
                            8:["U","R","D"],     9: ["all"],         10:["all"],          11: ["all"],          12: ["all"],          13: ["all"],          14:["all"],           15:["U","L","D"],
                           16:["U","R","D"],    17: ["all"],         18: ["all"],         19: ["all"],          20: ["all"],          21: ["all"],          22: ["all"],          23: ["U","L","D"],
                           24:["U","R","D"],    25: ["all"],         26: ["all"],         27: ["all"],          28: ["all"],          29: ["all"],          30: ["all"],          31: ["U","L","D"],
                           32:["U","R","D"],    33: ["all"],         34: ["all"],         35: ["all"],          36: ["all"],          37: ["all"],          38: ["all"],          39: ["U","L","D"],
                           40:["U","R","D"],    41: ["all"],         42: ["all"],         43: ["all"],          44: ["all"],          45: ["all"],          46: ["all"],          47: ["U","L","D"],
                           48:["U","R","D"],    49: ["all"],         50: ["all"],         51: ["all"],          52: ["all"],          53: ["all"],          54: ["all"],          55: ["U","L","D"],
                           56:["U","R"],        57: ["L", "U", "R"], 58: ["L", "U", "R"], 59: ["L", "U", "R"],  60: ["L", "U", "R"],  61: ["L", "U", "R"],  62: ["L", "U", "R"],  63: ["U","L"],
                        }
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
    # Define arrow probabilities
    # Assuming actions: {U, R, L, D}
    # Assuming probability also: {U, R, L, D}
    arrow_probability_up    = [[0.75, 0.1, 0.1, 0.05], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]
    arrow_probability_down  = [[0.05, 0.1, 0.1, 0.75], [0.15, 0.05, 0.6, 0.2],[0.15, 0.6, 0.05,0.2],[0.55,0.2,0.2,0.05]]
    arrow_probability_left  = [[0.1, 0.05, 0.75, 0.1], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]
    arrow_probability_right = [[0.75, 0.1, 0.1, 0.05], [0.2, 0.6, 0.05, 0.15],[0.2, 0.05, 0.6,0.15],[0.05,0.2,0.2,0.55]]

    # instantiate mdp probabilities.
    mdp_probability = probability_transition(arrow_map,arrow_probability_up, arrow_probability_down, arrow_probability_left, arrow_probability_right)

    # instantiate and initialize value function:
    V  = {}
    V_ = {}
    for i in range(0, len(states)):
        V[i] = 0
        V_[i] = 0

    # Value Iteration:
    counter = 0
    while True:
        # for each state s in S do
        for i in range(0, len(states)):
            # Compute max utility for the current state, search over all possible actions
            current_poss_actions = possible_actions[states[i]]
            max_util = 0
            for j in range(0, len(current_poss_actions)):
                util = V[states[i]] * mdp_probability.compute_probability(current_poss_actions, states[i],current_poss_actions[i])
                if util > max_util:
                    max_util = util

            # update V'. Reward plus max utility.
            V_[i] = reward.compute_reward(states[i]) + gam*max_util

        # prevent from breaking.
        counter = counter + 1
        if counter > 90000:
            break
if __name__ == "__main__":
    main()