
# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : p7_astar.py
# //|
# //| Description : Solves a convex tangram puzzle using A*.
# //|               Heuristic involves difference between centers of shapes.
# //|
# //| Notes : Future work can include making a class so main() is less cluttered
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////

from math import sqrt
from p7_square import Square
from p7_triag1 import Triangle1


# -------------------------------------------------------------------------
#  Function: move
#  Description: Given a list of vertices, shapes used, and tangram configuration,
#               returns all possible combinations next steps in a list of lists.
    #           0 = Square
    #           1 = Big Triangle.
    #           2 = Big Triangle.
    #           3 = Medium Triangle.
    #           4 = Small Triangle.
    #           5 = Combo
# -------------------------------------------------------------------------
def move(list_vertices, list_tangram, list_shapes):
    possible_moves = list_vertices[:]
    for shape in list_shapes:
        if shape == 0:
            square1 = Square(list_vertices, list_tangram)
            possible_moves = square1.get_possible_moves()
        if shape == 1:
            triag1 = Triangle1(list_vertices, list_tangram)
            triag1.get_possible_moves()

    return possible_moves
# -------------------------------------------------------------------------
#  Function: heuristic
#  Description: Given a list of centers, computes difference (norm) between
#               current centers and desired centers.
# -------------------------------------------------------------------------
def heuristic(list_centers, list_goal):

    # if list_centers not full of shapes, assume remaining centers in the middle.
    diff_length = len(list_goal) - len(list_centers)
    for i in range(0,diff_length):
        list_centers[ len(list_goal) + i] = [5,5]

    # heuristic is addition of difference between centers:
    counter = 0
    for i in range(0,len(list_goal)):
        counter = sqrt( (list_centers[i][0] - list_goal[i][0])**2 + (list_centers[i][1] - list_goal[i][1])**2 ) + counter
    return counter

# -------------------------------------------------------------------------
#  Function: calc_centroid
#  Description: Given a list of vertices, calculates centroids.
# -------------------------------------------------------------------------
def calc_centroid(list_of_lists):
    centroid = []
    for i in range(0,len(list_of_lists)):
        counter0 = 0
        counter1 = 0
        num = 0
        for j in range(0,len(list_of_lists[i])):
            counter0 = list_of_lists[i][j][0] + counter0
            counter1 = list_of_lists[i][j][1] + counter1
            num = num + 1
        centroid.append([counter0/num, counter1/num])
    return centroid

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
    list_init_centers  = []
    list_init_vertices = []
    list_goal          = [[0.7071067811865476, 0.7071067811865476],
                          [1.1785113019775793, 1.1785113019775793],
                          [1.4142135623730951, 0.23570226039551587]]
    list_tangram       = [[0,sqrt(2)/2]        , [sqrt(2)/2,0]               ,[sqrt(2)/2+sqrt(2), 0],
                          [2*sqrt(2),sqrt(2)/2], [sqrt(2)/2+sqrt(2), sqrt(2)],[sqrt(2)/2,sqrt(2)]]
    # derive initial cost based on initial state.
    cost_init = heuristic(list_tangram, list_goal)

    # initialize graph1 dictionary, graph node dictionary, and heuristic cost dictionary.
    # graph1: defines connectivity between nodes.
    # graph centers dictionary: defines where the centers of each shape is, for heuristic computation.
    # graph vertices dict: defines where the vertices of each shape is.
    # graph shapes dict: defines what shapes have NOT been used so far.
    #           0 = Square
    #           1 = Big Triangle.
    #           2 = Big Triangle.
    #           3 = Medium Triangle.
    #           4 = Small Triangle.
    #           5 = Combo
    # heuristic cost dictionary: defines what the heuristic cost is for each node.
    graph1 = {}
    graph_centers = {1:list_init_centers}
    graph_vertices = {1: list_init_vertices}
    graph_shapes = {1:[0, 1, 2, 3, 4, 5]}
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
            children = move(graph_vertices[current_head], list_tangram, graph_shapes[current_head])
            #now append as new nodes to graph1:
            idx = len(graph_vertices)
            child_idx = 1
            firstpass = True
            for child in children:
                if child not in graph_vertices.values():
                    # add children states to graph_vertices
                    graph_vertices[idx + child_idx] = child
                    # add the centers:
                    child_centers = calc_centroid(child)
                    graph_centers[idx + child_idx] = child_centers
                    # add the respective heuristic cost:
                    heuristic_cost[idx + child_idx] = heuristic(child_centers, list_goal)
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