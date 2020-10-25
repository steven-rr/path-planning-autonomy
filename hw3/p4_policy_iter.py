
# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : p4.py
# //|
# //| Description : Solves MDP using policy iteration
# //|
# //| Notes :
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////

from copy import deepcopy
import csv
import numpy as np

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
    def compute_probability_left_arrow(self, future_state_direction, action):
        result = 0
        if action == "U":
            if future_state_direction == "U":
                result = self.arrow_probability_left[0][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_left[0][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_left[0][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_left[0][3] + result
        elif action == "R":
            if future_state_direction == "U":
                result = self.arrow_probability_left[1][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_left[1][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_left[1][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_left[1][3] + result
        elif action == "L":
            if future_state_direction == "U":
                result = self.arrow_probability_left[2][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_left[2][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_left[2][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_left[2][3] + result
        elif action == "D":
            if future_state_direction == "U":
                result = self.arrow_probability_left[3][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_left[3][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_left[3][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_left[3][3] + result
        return result

    # assume desired_state is some direction.
    def compute_probability_right_arrow(self, future_state_direction, action):
        result = 0
        if action == "U":
            if future_state_direction == "U":
                result = self.arrow_probability_right[0][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_right[0][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_right[0][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_right[0][3] + result
        elif action == "R":
            if future_state_direction == "U":
                result = self.arrow_probability_right[1][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_right[1][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_right[1][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_right[1][3] + result
        elif action == "L":
            if future_state_direction == "U":
                result = self.arrow_probability_right[2][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_right[2][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_right[2][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_right[2][3] + result
        elif action == "D":
            if future_state_direction == "U":
                result = self.arrow_probability_right[3][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_right[3][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_right[3][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_right[3][3] + result
        return result

    # assume desired_state is some direction.
    def compute_probability_down_arrow(self, future_state_direction, action):
        result = 0
        if action == "U":
            if future_state_direction == "U":
                result = self.arrow_probability_down[0][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_down[0][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_down[0][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_down[0][3] + result
        elif action == "R":
            if future_state_direction == "U":
                result = self.arrow_probability_down[1][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_down[1][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_down[1][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_down[1][3] + result
        elif action == "L":
            if future_state_direction == "U":
                result = self.arrow_probability_down[2][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_down[2][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_down[2][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_down[2][3] + result
        elif action == "D":
            if future_state_direction == "U":
                result = self.arrow_probability_down[3][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_down[3][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_down[3][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_down[3][3] + result
        return result

    # assume desired_state is some direction.
    def compute_probability_up_arrow(self, future_state_direction, action):
        result = 0
        if action == "U":
            if future_state_direction == "U":
                result = self.arrow_probability_up[0][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_up[0][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_up[0][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_up[0][3] + result
        elif action == "R":
            if future_state_direction == "U":
                result = self.arrow_probability_up[1][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_up[1][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_up[1][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_up[1][3] + result
        elif action == "L":
            if future_state_direction == "U":
                result = self.arrow_probability_up[2][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_up[2][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_up[2][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_up[2][3] + result
        elif action == "D":
            if future_state_direction == "U":
                result = self.arrow_probability_up[3][0] + result
            elif future_state_direction == "R":
                result = self.arrow_probability_up[3][1] + result
            elif future_state_direction == "L":
                result = self.arrow_probability_up[3][2] + result
            elif future_state_direction == "D":
                result = self.arrow_probability_up[3][3] + result

        return result
    # compute probability of desired state based on current state and current action.
    def compute_probability(self, future_state_direction, current_state, action):

        direction = self.arrow_map[(current_state[0], current_state[1])]
        probability_return = 0
        if direction == "U":
            probability_return = self.compute_probability_up_arrow(future_state_direction, action)
        elif direction == "D":
            probability_return = self.compute_probability_down_arrow(future_state_direction, action)
        elif direction == "L":
            probability_return = self.compute_probability_left_arrow(future_state_direction, action)
        elif direction == "R":
            probability_return = self.compute_probability_right_arrow(future_state_direction, action)
        return probability_return

# -------------------------------------------------------------------------
#  Class : compute_policy_evaluation
#  Description: V(s) = R(s) + gam* summation over s' of P*V(s')
#               V(s) - gam * summation over s' of P*V(s') = R(s)
# ------------------------------------------------------------------------
def compute_policy_evaluation(pi, U, gam, mdp_probability, R, states,possible_future_states):
    # for each state calculate a value iteration.
    x = []
    # sets reward coefficients for each state - R(s)
    reward_list = []

    # initialize return dictionary, V
    V = {}
    for i in range(0 , len(states)):
        # initialize equation for row:
        linear_eqn_i = [0]*len(states)
        # append reward to reward_list - R(s)
        reward_list.append(R.compute_reward(states[i]))
        # set current state coefficients - V(s)
        linear_eqn_i[i] = 1
        # set future state coefficients - V(s')
        current_poss_future_states = possible_future_states[i]
        for j in range(0, len(current_poss_future_states)):
            future_state = current_poss_future_states[j]
            future_state_idx = compute_future_state_index(future_state, i )
            future_state_probability = mdp_probability.compute_probability(future_state,states[i], pi[i])
            linear_eqn_i[future_state_idx] = -1.0*gam*future_state_probability
        # append future state coefficients - V(s')
        x.append(linear_eqn_i)


    # do numpy manipulation of the linear equations, and the rewards.
    A = np.array(x)
    B = np.array(reward_list)
    V_numpy = np.linalg.inv(A).dot(B)

    for k in range(0, len(states)):
        V[k] = V_numpy[k]

    return V

# -------------------------------------------------------------------------
#  Class : compute_init_index
#  Description: Given future state direction, and index, it returns future index.
# ------------------------------------------------------------------------
def compute_state_index(x_state_tuple, states):
    x_init_idx = list(states.keys())[list(states.values()).index(x_state_tuple)]
    return x_init_idx
# -------------------------------------------------------------------------
#  Class : Reward
#  Description: Given future state direction, and index, it returns future index.
# ------------------------------------------------------------------------
def compute_future_state_index(future_state,i):
    idx_out = 0
    if future_state == "U":
        idx_out = i - 8
    elif future_state == "R":
        idx_out = i + 1
    elif future_state == "L":
        idx_out = i - 1
    elif future_state == "D":
        idx_out = i + 8

    return idx_out
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
    x_init = (8,1)
    goal= (2,8)
    O = [(1,1),(1,6),(3,4),(4,4),(4,5),(4,8),(5,2),(6,2),(6,6),(7,6), (8,6)]
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
                           48:["U","R"],        49: ["all"],         50: ["all"],         51: ["all"],          52: ["all"],          53: ["all"],          54: ["all"],          55: ["U","L","D"],
                           56:["U","R"],        57: ["U", "R"],      58: ["L", "U", "R"], 59: ["L", "U", "R"],  60: ["L", "U", "R"],  61: ["L", "U", "R"],  62: ["L", "U", "R"],  63: ["U","L"],
                        }
    # set "all" to "U", "R", "L", "D"
    for i in range(0, len(possible_actions)):
        if possible_actions[i][0] == "all":
            possible_actions[i] = ["U","R","L","D"]

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
    arrow_probability_down  = [[0.55, 0.2, 0.2, 0.05], [0.15, 0.6, 0.05, 0.2],[0.15, 0.05, 0.6,0.2],[0.05,0.1,0.1,0.75]]
    arrow_probability_left  = [[0.6, 0.15, 0.2, 0.05], [0.2, 0.55, 0.05, 0.2],[0.1, 0.05, 0.75,0.1],[0.05,0.15,0.2,0.6]]
    arrow_probability_right = [[0.6, 0.2, 0.15, 0.05], [0.1, 0.75, 0.05, 0.1],[0.2, 0.05, 0.55,0.2],[0.05,0.2,0.15,0.6]]

    # instantiate mdp probabilities.
    mdp_probability = probability_transition(arrow_map,arrow_probability_up, arrow_probability_down, arrow_probability_left, arrow_probability_right)

    # instantiate and initialize value function:
    V  = {}
    V_ = {}
    for i in range(0, len(states)):
        V[i] = 0
        V_[i] = 0

    # Policy Iteration:
    counter = 0
    # instantiate policy pi, initialize to always move states to the right, unless in the right edge. in that case, move left.
    pi = {}
    right_edge = [7, 15, 23, 31, 39, 47, 55, 63]
    for i in range(0, len(states)):
        if i in right_edge:
            pi[i] = "L"
        else:
            pi[i] = "R"

    # Begin Algorithm:
    while True:
        unchanged = True

        # Policy Evaluation:
        V = deepcopy(compute_policy_evaluation(pi, V,gam,mdp_probability, R, states,possible_actions ))
        print("Printing Value fx generated by policy eval.. : ")
        filename = "value_fx_" + str(counter) + ".csv"
        with open(filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([V[0], V[1], V[2], V[3], V[4], V[5], V[6], V[7]])
            csvwriter.writerow([V[8], V[9], V[10], V[11], V[12], V[13], V[14], V[15]])
            csvwriter.writerow([V[16], V[17], V[18], V[19], V[20], V[21], V[22], V[23]])
            csvwriter.writerow([V[24], V[25], V[26], V[27], V[28], V[29], V[30], V[31]])
            csvwriter.writerow([V[32], V[33], V[34], V[35], V[36], V[37], V[38], V[39]])
            csvwriter.writerow([V[40], V[41], V[42], V[43], V[44], V[45], V[46], V[47]])
            csvwriter.writerow([V[48], V[49], V[50], V[51], V[52], V[53], V[54], V[55]])
            csvwriter.writerow([V[56], V[57], V[58], V[59], V[60], V[61], V[62], V[63]])
        print("Finished. Printed to: ", filename)

        # for each state s in S do
        for i in range(0, len(states)):
            # Compute max utility for the current state, search over all possible actions
            current_poss_actions = possible_actions[i]
            util = 0

            policy_util =  0
            # Given state, and action from policy, loop thru all possible future states.
            for k in range(0,len(current_poss_actions)):
                future_state_direction = current_poss_actions[k]
                future_state_index = compute_future_state_index(future_state_direction, i)
                policy_util = V[future_state_index] * mdp_probability.compute_probability(future_state_direction, states[i], pi[i])

            max_util = -1000
            best_future_action = 0
            # Given state, loop over possible actions
            for j in range(0, len(current_poss_actions)):

                # Given the state, and the action, loop through all possible future states.
                for k in range(0 , len(current_poss_actions)):
                    future_state_direction = current_poss_actions[k]
                    future_state_index = compute_future_state_index(future_state_direction, i)
                    util = V[future_state_index] * mdp_probability.compute_probability(future_state_direction, states[i],current_poss_actions[j]) + util

                # if this current action gives higher utility than the max, then update max utility.
                if util > max_util:
                    max_util = util
                    best_future_action = current_poss_actions[j]
                util = 0

            # if best action provided a better policy, update our policy to match the best one.
            if max_util > policy_util:
                pi[i] = best_future_action
                unchanged = False

        # once delta is small enough, we can exit.
        if unchanged:
            break

        # prevent from breaking.
        counter = counter + 1
        if counter > 100000:
            print("OVERLOAD!!!!!!")
            break

    print("1) Finished Policy Iteration! Printing Value Fx.. ")
    filename = "value_fx.csv"
    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([V[0] , V[1] , V[2] ,  V[3] , V[4] , V[5] , V[6] ,  V[7]])
        csvwriter.writerow([V[8] , V[9] , V[10],  V[11], V[12], V[13], V[14],  V[15]])
        csvwriter.writerow([V[16], V[17], V[18],  V[19], V[20], V[21], V[22],  V[23]])
        csvwriter.writerow([V[24], V[25], V[26],  V[27], V[28], V[29], V[30],  V[31]])
        csvwriter.writerow([V[32], V[33], V[34],  V[35], V[36], V[37], V[38],  V[39]])
        csvwriter.writerow([V[40], V[41], V[42],  V[43], V[44], V[45], V[46],  V[47]])
        csvwriter.writerow([V[48], V[49], V[50],  V[51], V[52], V[53], V[54],  V[55]])
        csvwriter.writerow([V[56], V[57], V[58],  V[59], V[60], V[61], V[62],  V[63]])
    print("Finished. Printed to: ", filename)
if __name__ == "__main__":
    main()