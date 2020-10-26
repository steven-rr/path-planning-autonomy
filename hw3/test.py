
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
import random
class testing:
    def __init__(self):
        self.state = 1000

    def set_state(self , state_number):
        self.state = state_number

    def execute_function(self):
        print("it worked! ", self.state)

def test_fx(value):
    value.execute_function()

def coin_toss(p):
    bool = False
    if random.random() <= p:
        bool = True
    return bool

def main():
    heads = 0
    for i in range(0, 100000):
        if coin_toss(0.5):
            heads += 1
    print(heads)





if __name__ == "__main__":
    main()