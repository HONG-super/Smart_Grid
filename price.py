import numpy as np


def price (demand, sun,):
    sell = 10+ (demand - sun ) + 20*np.random.normal(0,1)
    return sell

def buy(sell):
    return sell*0.5

