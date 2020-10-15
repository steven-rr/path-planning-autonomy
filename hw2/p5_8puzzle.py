from p6_astar import move
from copy import deepcopy


def solve_all_possible_states(list_init_state):
    graph1 = {}
    graph_nodes = {1: list_init_state}

    # counter to break.
    counter = 100;
    Q = [[1]]
    while True:
        parents = deepcopy(Q)
        # loop thru parents. add children as nodes, and into graph1. then delete parents.
        # [[4, 2, 1], [5, 2, 1], [6, 3, 1], [7, 3, 1]]
        for i in range(0, len(parents)):
            parent_head = parents[i][0]
            parent_path = deepcopy(parents[i])
            children = move(graph_nodes[parent_head])
            idx = len(graph_nodes)
            child_idx = 1
            firstpass = True
            # add children as nodes.
            for child in children:
                if child not in graph_nodes.values():
                    # add children states to graph_nodes
                    graph_nodes[idx + child_idx] = child
                    # create edges within graph1.
                    if firstpass:
                        graph1[parent_head] = [idx + child_idx]
                        firstpass = False
                    else:
                        graph1[parent_head].append(idx + child_idx)
                    # add children to Q for future expansion
                    child_path = deepcopy(parent_path)
                    child_path.insert(0, idx + child_idx)
                    Q.append(child_path)
                    # increment child index.
                    child_idx = child_idx + 1
            # delete parents from Q.
            Q.remove(parent_path)

        print("nodes,expanded: ", len(graph_nodes.values()))
        if Q == []:
            print("Solving for all states : Complete.")
            print("Number of nodes expanded: " ,len(graph_nodes))
            return graph_nodes
def main():

    # 1 2 3
    # 4 5 6
    # 7 8 0
    list_init_state = [1,2,3,4,5,6,7,8,0]
    graph_nodes_even = solve_all_possible_states(list_init_state)
    # 1 2 3
    # 8 0 4
    # 7 6 5
    list_init_state = [1,2,3,8,0,4,7,6,5]
    graph_nodes_odd = solve_all_possible_states(list_init_state)


    if graph_nodes_even[1] in graph_nodes_odd.values():
        disjoint = False
    else:
        disjoint = True
    print("If disjoint is true, it is proof that there are 2 disjoint sets for the 8 puzzle: ")
    print(disjoint)
    return 0


if __name__ == "__main__":
    main()
