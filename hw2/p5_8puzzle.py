from p6_astar import move
from copy import deepcopy



def main():
    list_init_state = [1,2,3,4,5,6,7,8,0]

    graph1 = {}
    graph_nodes = {1:list_init_state}

    #counter to break.
    counter = 100;
    Q = [[1]]
    while True:
        parents = deepcopy(Q)
        # loop thru parents. add children as nodes, and into graph1. then delete parents.
        #[[4, 2, 1], [5, 2, 1], [6, 3, 1], [7, 3, 1]]
        for i in range(0,len(parents)):
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
            break
            print("ended.")
    return 0


if __name__ == "__main__":
    main()
