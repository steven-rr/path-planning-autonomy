
# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : p7_astar.py
# //|
# //| Description : Solves a convex tangram puzzle using A*.
# //|               Heuristic options: 1) Number of misplaced tiles.
# //|                                  2) Manhattan distance difference between
# //|                                     current state and goal state
# //|
# //| Notes : Future work can include making a class so main() is less cluttered
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////

from math import sqrt

# -------------------------------------------------------------------------
#  Function: move_right
#  Description: Given a list of tiles, outputs result tiles from
#               moving empty space to the right
# -------------------------------------------------------------------------
def move_right(list_in, idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in + 1]
    list_result[idx_in + 1] = 0
    return list_result

# -------------------------------------------------------------------------
#  Function: move_left
#  Description: Given a list of tiles, outputs result tiles from
#               moving empty space to the left
# -------------------------------------------------------------------------
def move_left(list_in ,idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in - 1]
    list_result[idx_in - 1] = 0
    return list_result

# -------------------------------------------------------------------------
#  Function: move_down
#  Description: Given a list of tiles, outputs result tiles from
#               moving empty space downward
# -------------------------------------------------------------------------
def move_down(list_in, idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in + 3]
    list_result[idx_in + 3] = 0
    return list_result

# -------------------------------------------------------------------------
#  Function: move_up
#  Description: Given a list of tiles, outputs result tiles from
#               moving empty space upward
# -------------------------------------------------------------------------
def move_up(list_in, idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in - 3]
    list_result[idx_in - 3] = 0
    return list_result
# -------------------------------------------------------------------------
#  Function: move
#  Description: Given a list of tiles, returns all possible combinations
#               of movements in a list of lists.
# -------------------------------------------------------------------------
def move(list_in):
    index = list_in.index(0)
    list1 = list_in[:]
    list2 = list_in[:]
    list3 = list_in[:]
    list4 = list_in[:]
    list_result = []
    if index == 0:
        list1 = move_right(list_in,index)
        list2 = move_down(list_in, index)
        list_result = [list1, list2]
        return list_result
    elif index == 1:
        list1 = move_right(list_in, index)
        list2 = move_left(list_in, index)
        list3 = move_down(list_in, index)
        list_result = [list1, list2, list3]
        return list_result
    elif index == 2:
        list1 = move_left(list_in, index)
        list2 = move_down(list_in, index)
        list_result = [list1, list2]
        return list_result
    elif index == 3:
        list1 = move_down(list_in, index)
        list2 = move_up(list_in, index)
        list3 = move_right(list_in, index)
        list_result = [list1, list2, list3]
        return list_result
    elif index == 4:
        list1 = move_right(list_in, index)
        list2 = move_left(list_in, index)
        list3 = move_down(list_in, index)
        list4 = move_up(list_in, index)
        list_result = [list1, list2, list3, list4]
        return list_result
    elif index == 5:
        list1 = move_up(list_in, index)
        list2 = move_down(list_in, index)
        list3 = move_left(list_in, index)
        list_result = [list1, list2, list3]
        return list_result
    elif index == 6:
        list1 = move_up(list_in, index)
        list2 = move_right(list_in, index)
        list_result = [list1, list2]
        return list_result
    elif index == 7:
        list1 = move_up(list_in, index)
        list2 = move_right(list_in, index)
        list3 = move_left(list_in, index)
        list_result = [list1, list2, list3]
        return list_result
    elif index == 8:
        list1 = move_up(list_in, index)
        list2 = move_left(list_in, index)
        list_result = [list1, list2]
        return list_result

# -------------------------------------------------------------------------
#  Function: heuristic1
#  Description: Given a list of tiles, outputs the number of misplaced tiles.
#               This is used as a heuristic for A*
# -------------------------------------------------------------------------
def heuristic1(list_in, list_goal):
    counter = 0
    for i in range(0,len(list_in)):
        if list_in[i] != list_goal[i]:
            counter = counter + 1
    return counter


# -------------------------------------------------------------------------
#  Function: move_right
#  Description: Given an index based on 1D list, outputs 2D coordinates
#               Based on the following scheme:
#
#                     col0  col1  col2
#               row 0   0     1     2
#               row 1   3     4     5
#               row 2   6     7     8
# -------------------------------------------------------------------------
def generate_coords(idx_in):
    idx_horiz = 0
    idx_vert  = 0
    if idx_in < 3:
        idx_horiz = 0
        idx_vert = idx_in
    elif idx_in < 6:
        idx_horiz = 1
        idx_vert = idx_in - 3
    elif idx_in < 9:
        idx_horiz = 2
        idx_vert = idx_in - 6
    coords_out = (idx_horiz, idx_vert)
    return coords_out

# -------------------------------------------------------------------------
#  Function: heuristic2
#  Description: Given a list of tiles, outputs the manhattan distance between
#               all tiles. This is used as a heuristic for A*
# -------------------------------------------------------------------------
def heuristic2(list_in, list_goal):
    counter = 0
    for i in range(0,len(list_in)):
        if list_in[i] != list_goal[i]:
            idx = list_in.index(list_goal[i])
            coords_goal = generate_coords(i)
            coords_actual = generate_coords(idx)
            diff_horizontal = abs(coords_goal[0] - coords_actual[0])
            diff_vertical = abs(coords_goal[1] - coords_actual[1])
            counter = counter + diff_horizontal + diff_vertical
    return counter

# -------------------------------------------------------------------------
#  Function: heuristic
#  Description: Given a list of tiles, returns the heuristic cost based on
#               mode.
# -------------------------------------------------------------------------
def heuristic(list_in, list_goal, mode_in):
    counter = 0
    j = 6
    for i in range(0,len(list_in)):
        (list_in[i][0] - list_goal[i][0])
        j = j -1
    return counter

# -------------------------------------------------------------------------------------------
#  Function: main
#  Description: Given an initial state, a goal state, and a heuristic mode, returns the moves
#               required to get to the goal state, along with action sequence.
#               Heuristic modes:
#                   1) Total number of misplaced tiles between current state and goal state
#                   2) Total manhattan distance difference between current state and goal state
# ----------------------------------------------------------------------------------------------
def main():
    # inputs, user defined:
    list_init_state = []
    list_init_vertices = []
    list_goal       = [[50*sqrt(2), 50*sqrt(2)],]
    heuristic_mode = 2

    # derive initial cost based on initial state.
    cost_init = heuristic(list_init_state, list_goal, heuristic_mode)

    # initialize graph1 dictionary, graph node dictionary, and heuristic cost dictionary.
    # graph1: defines connectivity between nodes
    # graph node dictionary: defines where the centers of each shape is, for heuristic computation.
    # graph vertices dict: defines where the vertices of each shape is.
    # heuristic cost dictionary: defines what the heuristic cost is for each node.
    graph1 = {}
    graph_nodes = {1:list_init_state}
    graph_vertices = {1: list_init_vertices}
    heuristic_cost = {1: cost_init}

    #perform Astar search on the graph.
    Q = [[[1],cost_init]]
    expanded = []
    i = 0
    while True:
        # step 2: if Q is empty, fail, else choose best path from Q.
        if  Q[0] == []:
            break
        else:
            fn = [i[1] for i in Q]
            best_path_index =  fn.index(min(fn))

        current_head = Q[best_path_index][0][0];
        best_path = Q[best_path_index][0];
        best_cost = Q[best_path_index][1] - heuristic_cost[current_head];
        # step 3 and 4: if head is G, return the best partial path. else remove the best partial path.
        if heuristic_cost[current_head] == 0:
            best_path_result = Q[best_path_index]
            break
        else:
            del Q[best_path_index]

        # step 5 if already expanded, don't expand further.
        if current_head in expanded:
            continue
        # step 6-9. expand, add all new paths for exploration.
        else:
            expanded.append(current_head)
            # all possible moves:
            children = move(graph_nodes[current_head])
            #now append as new nodes to graph1:
            idx = len(graph_nodes)
            child_idx = 1
            firstpass = True
            for child in children:
                if child not in graph_nodes.values():
                    # add children states to graph_nodes
                    graph_nodes[idx + child_idx] = child
                    # add the respective heuristic cost:
                    heuristic_cost[idx + child_idx] = heuristic(child, list_goal, heuristic_mode)
                    # create edges within graph1.
                    if firstpass:
                        graph1[current_head] = [idx + child_idx]
                        firstpass = False
                    else:
                        graph1[current_head].append(idx+child_idx)
                    # increment child index.
                    child_idx = child_idx + 1

            #get node of the best path:
            if current_head in graph1.keys():
                children = graph1[current_head]
            else:
                children = []

            for child in children:
                if child in expanded:
                    children.remove(child)
            for child in children:
                new_path = best_path[:]
                new_path.insert(0,child)
                new_cost = best_cost + 1 + heuristic_cost[child]
                Q.append([new_path, new_cost])

        #prevent overload.
        i = i + 1
        if i > 20000:
            print("ended early.")
            break

    print("best path is: ", best_path_result[0])

    print("action sequence: ")
    for i in range(len(best_path_result[0])-1, -1, -1 ):
        print(graph_nodes[best_path_result[0][i]])

    print("best path cost is: ", best_path_result[1])
    print("nodes expanded: ", len(graph_nodes))


if __name__ == "__main__":
    main()