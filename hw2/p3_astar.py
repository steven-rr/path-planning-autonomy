
#define graph dictionary for problem 3:
graph1 = {1:[2,16],       2:[1,3],          3:[2,4,14],    4:[3,5,13],        5:[4,12],         6:[],             7:[8,10],         8:[7,9],
          9:[8,10,24],   10:[7,9,23],      11:[],         12:[5,13,21],      13:[4,12,14,20],  14:[3,13,19],     15:[],            16:[1,17],
          17:[16,32],    18:[],            19:[14,20,30], 20:[13,19,21],     21:[12,20,22],    22:[21,23,27],    23:[10,22,24,26], 24:[9,23],
          25:[],         26:[23,27,39],    27:[22,26,38], 28:[],             29:[],            30:[19,31,35],    31:[30,32,34],    32:[17,31,33],
          33:[32,34,48], 34:[31,33,35,47], 35:[30,34,46], 36:[],             37:[38,44],       38:[27,37,39,43], 39:[26,38,40,42], 40:[39,41],
          41:[40,42,56], 42:[39,41,43,55], 43:[38,42,44], 44:[37,43,45,53],  45:[44,46,52],    46:[35,45,47,51], 47:[34,46,48,50], 48:[33,47],
          49:[],         50:[47,51],       51:[46,50,52], 52:[45,51,53],     53:[44,52],       54:[],            55:[42,56],       56:[41,55]}

print(graph1[1])

#perform Astar search on the graph.
Q = []
expanded = []

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