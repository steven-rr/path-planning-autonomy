#define graph dictionary, key is node, values are nodes connected to key node and number is the associated cost.
#define heuristic cost, which is an estimated number on how close we are to the goal from the current node. higher number means we are farther away.

graph1 = {'S':[['A',2],['B',5]],    'A':[['S',3],['D',4],['C',2]],           'B':[['S',5],['D',1],['G',5]],
          'C':[['A',2],['D',3]],    'D':[['A',4],['C',3],['G',2],['B', 1]],  'G':[['D',2],['B',5]]}

heuristic_cost = {'S':0, 'A':2, 'B':3, 'C':1, 'D':1,  'G':0}

#Implement A star algorithm

Q        = [['S', 0]]
expanded = []

