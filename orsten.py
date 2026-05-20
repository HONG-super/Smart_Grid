

import numpy as np


def pricer (sigma, theta,mu,n):
    x = np.zeros(n)
    x[0]= mu[0]
    for t in range (0,n-1):
        x[t+1]= x[t] + theta*(mu[t]- x[t]) +sigma*np.random.normal(0, sigma)
    return x

if __name__ == '__main__':

    import matplotlib.pyplot as plt
    n=1000
    t = np.linspace(0, 2*np.pi,n)
    mu = np.sin(t)
    x= pricer (1, 2, mu,n)
    y =pricer (1, 0.3, mu,n)
    z= pricer (1, 0.3, mu,n)
    plt.plot(x)

    plt.plot(mu,'--')
    plt.show()

    




        

