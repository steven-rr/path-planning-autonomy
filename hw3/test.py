
# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : test.py
# //|
# //| Description : Solves MDP using value iteration
# //|
# //| Notes :
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////

import numpy as np
class testing:
    def __init__(self):
        self.state = 1000

    def set_state(self , state_number):
        self.state = state_number

    def execute_function(self):
        print("it worked! ", self.state)

def test_fx(value):
    value.execute_function()

def main():
    V = {}
    x = testing()
    x.set_state(9001)
    test_fx(x)

    V[0] = 1
    V[2] = 3
    print(V)




if __name__ == "__main__":
    main()