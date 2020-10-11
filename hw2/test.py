list_init = [5, 4, 0 , 6, 1, 8, 7, 3, 2]
graph1 = {1:list_init}

list1=  [5, 4, 6 , 0, 1, 8, 7, 3, 2]
list2=  [5, 0, 4 , 6, 1, 8, 7, 3, 2]
children = [list1, list2]

#start
idx = len(graph1)
child_idx = 1
for child in children:
    if child not in graph1.values():
        graph1[idx + child_idx] = child
        child_idx = child_idx + 1

#end

print(len(graph1))
print(graph1)

graph_new = {}
graph_new[1] = [1]
print(graph_new)
graph_new[1].append(3)
print(graph_new)