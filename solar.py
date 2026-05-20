import numpy as np

def solar (tick):
    psolar = 100*np.sin(((tick - 15)*np.pi)/30)
    return  psolar
