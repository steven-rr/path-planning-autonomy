
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

from copy import deepcopy
import csv
import random
# -------------------------------------------------------------------------
#  Class : Probability transition
#  Description: Holds arrow map and arrow probabilities. Depending on,
#               arrow direction and action, it will compute probabilities.
# ------------------------------------------------------------------------
class probability_transition():
    """
    The class is used to compute probability of going to state s', given state s,
    and action a. compute_probability() is the main class function that does this.
    This function will call one of the 4 helper functions , depending on the arrow.
    Additionally, a probability distribution class function is also implemented,
    mainly for simulation purposes. Given you have some policy, I model how the robot
    would act by using this probability distribution in addition to random.random().
    """
    def __init__(self, arrow_map, arrow_probability_up , arrow_probability_down, arrow_probability_left, arrow_probability_right):
        self.arrow_map = arrow_map
        self.arrow_probability_up = arrow_probability_up
        self.arrow_probability_down = arrow_probability_down
        self.arrow_probability_left = arrow_probability_left
        self.arrow_probability_right = arrow_probability_right

    # Computing probabilities for left arrow.
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

    # Computing probabilities for right arrow.
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

    # Computing probabilities for down arrow.
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

    # Computing probabilities for up arrow.
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

    # compute probability of going to a future state based on current state and current action.
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
        elif direction == "0":
            if future_state_direction == action:
                probability_return = 1
            else:
                probability_return = 0
        return probability_return

    # compute probability distribution from trying to do a certain action, given a current state.
    def compute_probability_distribution(self, desired_action, current_state):
        direction = self.arrow_map[(current_state[0], current_state[1])]
        probability_distribution_return = []
        if direction == "U":
            if desired_action == "U":
                probability_distribution_return = self.arrow_probability_up[0]
            elif desired_action == "R":
                probability_distribution_return = self.arrow_probability_up[1]
            elif desired_action == "L":
                probability_distribution_return = self.arrow_probability_up[2]
            elif desired_action == "D":
                probability_distribution_return = self.arrow_probability_up[3]
        elif direction == "R":
            if desired_action == "U":
                probability_distribution_return = self.arrow_probability_right[0]
            elif desired_action == "R":
                probability_distribution_return = self.arrow_probability_right[1]
            elif desired_action == "L":
                probability_distribution_return = self.arrow_probability_right[2]
            elif desired_action == "D":
                probability_distribution_return = self.arrow_probability_right[3]
        elif direction == "L":
            if desired_action == "U":
                probability_distribution_return = self.arrow_probability_left[0]
            elif desired_action == "R":
                probability_distribution_return = self.arrow_probability_left[1]
            elif desired_action == "L":
                probability_distribution_return = self.arrow_probability_left[2]
            elif desired_action == "D":
                probability_distribution_return = self.arrow_probability_left[3]
        elif direction == "D":
            if desired_action == "U":
                probability_distribution_return = self.arrow_probability_down[0]
            elif desired_action == "R":
                probability_distribution_return = self.arrow_probability_down[1]
            elif desired_action == "L":
                probability_distribution_return = self.arrow_probability_down[2]
            elif desired_action == "D":
                probability_distribution_return = self.arrow_probability_down[3]
        elif direction == "0":
            if desired_action == "U":
                probability_distribution_return = [1.0, 0.0, 0.0, 0.0]
            elif desired_action == "R":
                probability_distribution_return = [0.0, 1.0, 0.0, 0.0]
            elif desired_action == "L":
                probability_distribution_return = [0.0, 0.0, 1.0, 0.0]
            elif desired_action == "D":
                probability_distribution_return = [0.0, 0.0, 0.0, 1.0]
        return probability_distribution_return
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
            reward_out = 10
        elif current_state in self.obstacles:
            reward_out = -10
        else:
            reward_out = -1
        return reward_out

# -------------------------------------------------------------------------
#  Class : Reward for Part c Case a
#  Description: Holds functions for computing reward based on states and
#               obstacles.
# ------------------------------------------------------------------------
class reward_case_a():
    def __init__(self, obstacles, goal, tuple_list):
        self.obstacles = obstacles
        self.goal = goal
        self.tuple_list = tuple_list
    def compute_reward(self,current_state):
        reward_out = 0
        if current_state == self.goal:
            reward_out = 10
        elif current_state in self.obstacles:
            reward_out = -10
        elif current_state in self.tuple_list:
            reward_out = 0
        else:
            reward_out = -1
        return reward_out

# -------------------------------------------------------------------------
#  Class : Reward for Part c Case b
#  Description: Holds functions for computing reward based on states and
#               obstacles.
# ------------------------------------------------------------------------
class reward_case_b():
    def __init__(self, obstacles, goal, tuple_list):
        self.obstacles = obstacles
        self.goal = goal
        self.tuple_list = tuple_list
    def compute_reward(self,current_state):
        reward_out = 0
        if current_state == self.goal:
            reward_out = 10
        elif current_state in self.obstacles:
            reward_out = -10
        elif current_state in self.tuple_list:
            reward_out = 100
        else:
            reward_out = -1
        return reward_out

# -------------------------------------------------------------------------
#  Class : Reward for Part c Case c
#  Description: Holds functions for computing reward based on states and
#               obstacles.
# ------------------------------------------------------------------------
class reward_case_c():
    def __init__(self, obstacles, goal, tuple_list):
        self.obstacles = obstacles
        self.goal = goal
        self.tuple_list = tuple_list
    def compute_reward(self,current_state):
        reward_out = 0
        if current_state == self.goal:
            reward_out = 10
        elif current_state in self.obstacles:
            reward_out = -10
        elif current_state in self.tuple_list:
            reward_out = -3
        else:
            reward_out = -1
        return reward_out
# -------------------------------------------------------------------------
#  Function : print_value_iter_result
#  Description: Prints V field into a csv file, 8x8 grid for easy visualization.
# ------------------------------------------------------------------------
def print_value_iter_result(V_):
    print("1) Finished Value Iteration! Printing Value Fx..")
    filename = "value_fx_value_iter.csv"
    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([V_[0] , V_[1] , V_[2] ,  V_[3] , V_[4] , V_[5] , V_[6] ,  V_[7]])
        csvwriter.writerow([V_[8] , V_[9] , V_[10],  V_[11], V_[12], V_[13], V_[14],  V_[15]])
        csvwriter.writerow([V_[16], V_[17], V_[18],  V_[19], V_[20], V_[21], V_[22],  V_[23]])
        csvwriter.writerow([V_[24], V_[25], V_[26],  V_[27], V_[28], V_[29], V_[30],  V_[31]])
        csvwriter.writerow([V_[32], V_[33], V_[34],  V_[35], V_[36], V_[37], V_[38],  V_[39]])
        csvwriter.writerow([V_[40], V_[41], V_[42],  V_[43], V_[44], V_[45], V_[46],  V_[47]])
        csvwriter.writerow([V_[48], V_[49], V_[50],  V_[51], V_[52], V_[53], V_[54],  V_[55]])
        csvwriter.writerow([V_[56], V_[57], V_[58],  V_[59], V_[60], V_[61], V_[62],  V_[63]])
    print("2) Output value function grid to: ", filename)
    print("")
    print("Time to back out the policy...")
    print("")


# -------------------------------------------------------------------------
#  Function : print_simulation_result
#  Description: Prints to console the success rate and reward average across
#               simulation runs.
# ------------------------------------------------------------------------
def print_simulation_result(succesful_attempts, simulation_run_number, simulated_reward_list, steps_required):
    print("3) Simulated ", simulation_run_number, " attempts. Results are the following: ")
    print(succesful_attempts, "succesful attempts out of ", simulation_run_number, ". That is a ", (succesful_attempts/simulation_run_number)*100, " % chance of success.")
    reward_accumulator = 0
    steps_accumulator = 0
    for i in range(0, len(simulated_reward_list)):
        reward_accumulator = simulated_reward_list[i] + reward_accumulator
    for i in range(0, len(steps_required)):
        steps_accumulator = steps_required[i] + steps_accumulator
    reward_average = reward_accumulator / len(simulated_reward_list)
    steps_average =  steps_accumulator / len(steps_required)

    print(reward_average, "is the average reward accumulated ")
    print(steps_average, "is average steps taken ")


# -------------------------------------------------------------------------
#  Function : compute_state_index
#  Description: Given a state tuple, and a state dictionary, return the state index.
# ------------------------------------------------------------------------
def compute_state_index(x_state_tuple, states):
    x_init_idx = list(states.keys())[list(states.values()).index(x_state_tuple)]
    return x_init_idx

# -------------------------------------------------------------------------
#  Function : compute_future_state_index
#  Description: Given an action and a state index, return the future index
#               after taking the action in a certain direction.
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
#  Function : compute_future_state
#  Description: Given action, state tuple, and state dict, returns future state tuple
# ------------------------------------------------------------------------
def compute_future_state(action, state_tuple, states):
    state_idx = compute_state_index(state_tuple, states)
    future_state_idx = compute_future_state_index(action, state_idx)
    future_state = states[future_state_idx]
    return future_state


# -------------------------------------------------------------------------
#  Function : value_iteration
#  Description: Does Value iteration, returns the optimal value function.
# ------------------------------------------------------------------------
def value_iteration(mdp_probability, gam, epsilon, R, V_,V, possible_actions, legal_actions, states):

    counter = 0
    delta = 10
    while True:
        V = deepcopy(V_)
        delta = 0
        # for each state s in S do
        for i in range(0, len(states)):
            # Compute max utility for the current state, search over all possible actions
            current_poss_actions = possible_actions[i]
            util = 0
            max_util = -1000
            # Given state, loop over possible actions
            for j in range(0, len(current_poss_actions)):

                # Given the state, and the action, loop through all possible future states.
                for k in range(0, len(current_poss_actions)):
                    future_state_direction = current_poss_actions[k]
                    future_state_index = compute_future_state_index(future_state_direction, i)
                    if future_state_direction not in legal_actions[i]:
                        future_state_index = i
                    util = V[future_state_index] * mdp_probability.compute_probability(future_state_direction,
                                                                                       states[i],
                                                                                       current_poss_actions[j]) + util

                # if this current action gives higher utility than the max, then update max utility.
                if util > max_util:
                    max_util = util
                util = 0


            # update V'. Reward plus max utility.
            V_[i] = R.compute_reward(states[i]) + gam * max_util

            # check for delta update.
            if abs(V_[i] - V[i]) > delta:
                delta = abs(V_[i] - V[i])

        # once delta is small enough, we can exit.
        if delta < (epsilon * (1 - gam) * (1 / gam)):
            break
        # prevent from breaking.
        counter = counter + 1
        if counter > 100000:
            print("OVERLOAD!!!!!!")
            break
    return V

# -------------------------------------------------------------------------
#  Function : compute_optimal_policy
#  Description: Backs out optimal policy based on V*:
#               -returns the optimal policy as variable: "action_best_list"
#               -returns optimal intended state sequence:  "x_list".
# ------------------------------------------------------------------------
def compute_optimal_policy(x_init, goal, legal_actions,V, states, mdp_probability, gam):
    # Compute Policy based on optimal Value function V*
    all_moves = ['U','R','L','D']
    x = compute_state_index(x_init,states)
    x_goal = compute_state_index(goal, states)
    x_list = [x]
    action_best = 0
    action_best_list = []
    optimal_policy_results = []
    optimal_policy_dict = {}
    counter = 0
    # check through all possible actions of the current state, go to action which provides the highest value.
    x_best = 0
    action_best = 0
    for i in range(0, len(states)):
        q_max = -1000
        current_poss_actions = legal_actions[i]
        # loop over possible actions in given state.
        for j in range(0, len(all_moves)):
            if all_moves[j] in current_poss_actions:
                poss_state = compute_future_state_index(all_moves[j], i)
            else:
                poss_state = i

            # loop over final actions, given a state and given a desired action.
            q_current = 0
            for k in range(0, len(all_moves)):
                if all_moves[k] in current_poss_actions:
                    x_next = compute_future_state_index(all_moves[k], i)
                    q_current = gam*V[x_next]*mdp_probability.compute_probability(all_moves[k], states[i], all_moves[j]) + q_current
                else:
                    q_current = gam*V[i]*mdp_probability.compute_probability(all_moves[k], states[i], all_moves[j]) + q_current
            if q_current > q_max:
                if i == 15:
                    print("i'm here..")

                V_best = V[poss_state]
                q_max = q_current
                x_best = poss_state
                action_best = all_moves[j]
                optimal_policy_dict[i] = all_moves[j]

    # maybe create a fx for this:
    print("Policy is derived!")
    for i in range(0, len(optimal_policy_dict)):
        if (i+1) % 8 == 0:
            print(optimal_policy_dict[i])
        else:
            print(optimal_policy_dict[i], end=" ")
    print("")
    print("Printing policy to.. : " , end=" ")
    filename = "policy_grid_value_iter.csv"
    print(filename)
    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([optimal_policy_dict[0], optimal_policy_dict[1], optimal_policy_dict[2], optimal_policy_dict[3], optimal_policy_dict[4], optimal_policy_dict[5], optimal_policy_dict[6], optimal_policy_dict[7]])
        csvwriter.writerow([optimal_policy_dict[8], optimal_policy_dict[9], optimal_policy_dict[10], optimal_policy_dict[11], optimal_policy_dict[12], optimal_policy_dict[13], optimal_policy_dict[14], optimal_policy_dict[15]])
        csvwriter.writerow([optimal_policy_dict[16], optimal_policy_dict[17], optimal_policy_dict[18], optimal_policy_dict[19], optimal_policy_dict[20], optimal_policy_dict[21], optimal_policy_dict[22], optimal_policy_dict[23]])
        csvwriter.writerow([optimal_policy_dict[24], optimal_policy_dict[25], optimal_policy_dict[26], optimal_policy_dict[27], optimal_policy_dict[28], optimal_policy_dict[29], optimal_policy_dict[30], optimal_policy_dict[31]])
        csvwriter.writerow([optimal_policy_dict[32], optimal_policy_dict[33], optimal_policy_dict[34], optimal_policy_dict[35], optimal_policy_dict[36], optimal_policy_dict[37], optimal_policy_dict[38], optimal_policy_dict[39]])
        csvwriter.writerow([optimal_policy_dict[40], optimal_policy_dict[41], optimal_policy_dict[42], optimal_policy_dict[43], optimal_policy_dict[44], optimal_policy_dict[45], optimal_policy_dict[46], optimal_policy_dict[47]])
        csvwriter.writerow([optimal_policy_dict[48], optimal_policy_dict[49], optimal_policy_dict[50], optimal_policy_dict[51], optimal_policy_dict[52], optimal_policy_dict[53], optimal_policy_dict[54], optimal_policy_dict[55]])
        csvwriter.writerow([optimal_policy_dict[56], optimal_policy_dict[57], optimal_policy_dict[58], optimal_policy_dict[59], optimal_policy_dict[60], optimal_policy_dict[61], optimal_policy_dict[62], optimal_policy_dict[63]])
    print("2) Output policy grid to: ", filename)
    #update x to be the one with the biggest Value.
    x = x_best
    x_list.append(x_best)
    action_best_list.append(action_best)

    return optimal_policy_dict

# -------------------------------------------------------------------------
#  Function : coin_toss
#  Description: Given a probability distribution, returns the actual action to occur.
#               Assume that the probability distribution is = [U, R, L, D]
# ------------------------------------------------------------------------
def coin_toss(probability_distribution):
    """
    random.random() generates a number between 0 and 1 uniformly.
    Each action has an associated probability and all probabilities add up to one,
    therefore I partition the interval [0, 1] into smaller intervals, where the partitioned
    interval sizes are based on the probabilities. The more likely an action, the bigger
    the partition. Then, I simply use random.random() to generate a number between 0 and 1.
    If the random number falls in the interval partition, then that respective action is the
    one the simulated robot will take based on the probability distribution.
    """
    num1 = probability_distribution[0]
    num2 = num1 + probability_distribution[1]
    num3 = num2 + probability_distribution[2]
    num4 = num3 + probability_distribution[3]
    random_number = random.random()
    final_action = 0
    # check where the random number lands, and perform the final action accordingly.
    if random_number < num1:
        final_action = "U"
    elif num1 < random_number < num2:
        final_action = "R"
    elif num2 < random_number < num3:
        final_action = "L"
    elif num3 < random_number < num4:
        final_action = "D"
    return final_action

# -------------------------------------------------------------------------
#  Function : run_simulation
#  Description: Given an optimal policy, probability state transition, and how many
#               simulations to run, it computes how many times robot can go from x_init
#               to x_goal. Returns number of succesful attempts, and the reward list.
# ------------------------------------------------------------------------
def run_simulation(simulation_run_number, pi, mdp_probability, legal_actions, x_init, goal, states, R):
    x_list = [[] for x in range(0, simulation_run_number)]
    simulated_reward_list = []
    succesful_attempts = 0
    simulation_results = []
    steps_required = []
    run_length = 100
    O = [0, 5, 19, 27, 28, 31, 33, 41, 45, 53, 61]
    print("Starting simulation. Run Length of: ", run_length, "iterations per sim... ")

    # loop over simulation runs.
    for i in range(0, simulation_run_number):
        # initialize x to initial position and reward to zero.
        x = compute_state_index(x_init, states)
        x_list[i].append(x)
        simulated_reward = 0
        # attempt to get to the goal given the best policy. limit each simulation run to 100 steps.
        for j in range(0, run_length):
            desired_action = pi[x]
            probability_distribution = mdp_probability.compute_probability_distribution(desired_action, states[x])
            final_action = coin_toss(probability_distribution)
            if final_action in legal_actions[x]:
                x = compute_future_state_index(final_action, x)

            x_list[i].append(x)
            simulated_reward = R.compute_reward(states[x]) + simulated_reward
            # count number of succesful attempts
            if states[x] == goal:
                succesful_attempts = succesful_attempts + 1
                simulated_reward_list.append(simulated_reward)
                steps_required.append(j + 1)
                print("Simulation #", i, " complete.. ")
                break
            elif j == run_length - 1:
                simulated_reward_list.append(simulated_reward)
                steps_required.append(j + 1)
                print("Simulation #", i, " complete.. ")
    print("")
    simulation_results.append(simulated_reward_list)
    simulation_results.append(succesful_attempts)
    simulation_results.append(steps_required)
    return simulation_results
# -------------------------------------------------------------------------
#  Function : main
#  Description: Holds functions for computing reward based on states and
#               obstacles.
#
#  Variables:
#               x_init = initial state tuple
#               goal   = goal state tuple
#               O      = Obstacles
#               gam    = Discount factor
#               U      = Utility
#               R      = reward.
#
#               NOTE FOR REWARDS: you can uncomment different
#                                 rewards for different cases.
# ------------------------------------------------------------------------
def main():
    x_init = (8,1)
    goal= (2,8)
    O = [(1,1),(1,6),(3,4),(4,4),(4,5),(4,8),(5,2),(6,2),(6,6),(7,6), (8,6)]
    gam = 0.95
    tuple1 = (4,3)
    tuple2 = (1,5)
    tuple3 = (2,5)
    tuple_list = [tuple1, tuple2, tuple3]
    sim_flag = True
    # R = reward(O, goal)
    # R = reward_case_a(O, goal, tuple_list)
    # R = reward_case_b(O, goal, tuple_list)
    R = reward_case_c(O, goal, tuple_list)

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
    # each state has legal actions and  possible actions:
    legal_actions =  {
                            0:["R","D"],         1: ["L", "D", "R"],  2:["L", "D", "R"],   3: ["L", "D", "R"],   4: ["L", "D", "R"],   5: ["L", "D", "R"],   6:["L", "D", "R"],    7:["L","D"],
                            8:["U","R","D"],     9: ["all"],         10:["all"],          11: ["all"],          12: ["all"],          13: ["all"],          14:["all"],           15:["U","L","D"],
                           16:["U","R","D"],    17: ["all"],         18: ["all"],         19: ["all"],          20: ["all"],          21: ["all"],          22: ["all"],          23: ["U","L","D"],
                           24:["U","R","D"],    25: ["all"],         26: ["all"],         27: ["all"],          28: ["all"],          29: ["all"],          30: ["all"],          31: ["U","L","D"],
                           32:["U","R","D"],    33: ["all"],         34: ["all"],         35: ["all"],          36: ["all"],          37: ["all"],          38: ["all"],          39: ["U","L","D"],
                           40:["U","R","D"],    41: ["all"],         42: ["all"],         43: ["all"],          44: ["all"],          45: ["all"],          46: ["all"],          47: ["U","L","D"],
                           48:["U","R"],        49: ["all"],         50: ["all"],         51: ["all"],          52: ["all"],          53: ["all"],          54: ["all"],          55: ["U","L","D"],
                           56:["U","R"],        57: ["U", "R"],      58: ["L", "U", "R"], 59: ["L", "U", "R"],  60: ["L", "U", "R"],  61: ["L", "U", "R"],  62: ["L", "U", "R"],  63: ["U","L"],
                        }

    possible_actions =  {
                            0:["all"],     1: ["all"],          2: ["all"],          3: ["all"],           4: ["all"],           5: ["all"],           6: ["all"],           7: ["all"],
                            8:["all"],     9: ["all"],         10: ["all"],         11: ["all"],          12: ["all"],          13: ["all"],          14: ["all"],          15: ["all"],
                           16:["all"],    17: ["all"],         18: ["all"],         19: ["all"],          20: ["all"],          21: ["all"],          22: ["all"],          23: ["all"],
                           24:["all"],    25: ["all"],         26: ["all"],         27: ["all"],          28: ["all"],          29: ["all"],          30: ["all"],          31: ["all"],
                           32:["all"],    33: ["all"],         34: ["all"],         35: ["all"],          36: ["all"],          37: ["all"],          38: ["all"],          39: ["all"],
                           40:["all"],    41: ["all"],         42: ["all"],         43: ["all"],          44: ["all"],          45: ["all"],          46: ["all"],          47: ["all"],
                           48:["all"],    49: ["all"],         50: ["all"],         51: ["all"],          52: ["all"],          53: ["all"],          54: ["all"],          55: ["all"],
                           56:["all"],    57: ["all"],         58: ["all"],         59: ["all"],          60: ["all"],          61: ["all"],          62: ["all"],          63: ["all"],
                        }

    # set "all" to "U", "R", "L", "D"
    for i in range(0, len(possible_actions)):
        if possible_actions[i][0] == "all":
            possible_actions[i] = ["U","R","L","D"]

    for i in range(0, len(legal_actions)):
        if legal_actions[i][0] == "all":
            legal_actions[i] = ["U", "R", "L", "D"]

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

    # Perform Value Iteration, return utility function:
    epsilon = 0.001
    V = value_iteration(mdp_probability,gam, epsilon, R, V_, V, possible_actions, legal_actions, states)

    #print result of value iteration to csv file.
    print_value_iter_result(V_)

    # Back out optimal policy from V*, return optimal action and optimal state sequence based on initial state..
    pi = compute_optimal_policy(x_init,goal,legal_actions,V, states ,mdp_probability, gam)


    # Run simulation with policy result. Initialize how many runs you want to do:
    if sim_flag == True:
        simulation_run_number = 50
        [simulated_reward_list, succesful_attempts, steps_required] = run_simulation(simulation_run_number, pi, mdp_probability, legal_actions, x_init, goal, states, R)

        # print simulation results to console.
        print_simulation_result(succesful_attempts,simulation_run_number,simulated_reward_list, steps_required)

if __name__ == "__main__":
    main()