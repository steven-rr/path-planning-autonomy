#moving empty space to the right.
def move_right(list_in, idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in + 1]
    list_result[idx_in + 1] = 0
    return list_result
#moving empty space to the left.
def move_left(list_in ,idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in - 1]
    list_result[idx_in - 1] = 0
    return list_result
#moving empty space to the left.
def move_down(list_in, idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in + 3]
    list_result[idx_in + 3] = 0
    return list_result
#moving empty space to the left.
def move_up(list_in, idx_in):
    list_result = list_in[:]
    list_result[idx_in] = list_in[idx_in - 3]
    list_result[idx_in - 3] = 0
    return list_result

#given a node, it will return all possible movement possibilities.
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
        list2 = move_right(list_in, index)
        list3 = move_left(list_in, index)
        list_result = [list1, list2, list3]
        return list_result
# number of misplaced tiles.
def heuristic1(list_in):
    list_goal = [1, 2, 3, 4, 5, 6, 7, 8, 0]
    counter = 0
    for i in range(0,len(list_in)):
        if list_in[i] != list_goal[i]:
            counter = counter + 1
    return counter

#define graph dictionary for problem 6:
list_init = [5, 4, 0 , 6, 1, 8, 7, 3, 2]
cost_init = heuristic1(list_init)


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
    if current_head == 41:
        best_path_result = Q[best_path_index]
        break
    else:
        del Q[best_path_index]

    # step 5
    if current_head in expanded:
        continue
    # step 6-9. expand and add new paths.
    else:
        expanded.append(current_head)
        children = graph1[current_head]
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
    if i > 10000:
        break

print("best path is: ", best_path_result[0])
print("best path cost is: ", best_path_result[1])