#define graph dictionary, key is node, values are nodes connected to key node and number is the associated cost.
#define heuristic cost, which is an estimated number on how close we are to the goal from the current node. higher number means we are farther away.

graph1 = {'S':[['A',2],['B',5]],    'A':[['D',4],['C',2]],   'B':[['D',1],['G',5]],
          'C':[],                   'D':[['C',3],['G',2]],   'G':[]}

heuristic_cost = {'S':0, 'A':2, 'B':3, 'C':1, 'D':1,  'G':0}

#Implement A star algorithm

Q        = [['S', 0]]
expanded = []
#new_path = ['S',0]
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
    if current_head == 'G':
        best_path_result = Q[best_path_index]
        break
    else:
        del Q[best_path_index]

    # step 5
    if current_head in expanded:
        continue
    # step 6-9
    else:
        expanded.append(current_head)
        children = graph1[current_head]
        for child in children:
            if child[0] in expanded:
                children.remove(child)
        for child in children:
            new_path = child[0] + ' ' + best_path
            new_cost = best_cost + child[1] + heuristic_cost[child[0]]
            Q.append([new_path, new_cost])

    #prevent overload.
    i = i + 1
    if i > 100:
        break

print("best path is: " + best_path_result[0])
print("best path cost is: ", best_path_result[1])