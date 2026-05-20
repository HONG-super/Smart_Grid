
import numpy as np
import orsten as pricer


def monte (mu, sigma , theta,n, price):
    a=[]
    for t in range(n):
        x = pricer.pricer(sigma,theta,mu,n)
        a.append(x)
    d= []
    for i in range (n):
        total_price = np.sum(a[i]*price)
        d.append(total_price)
    return a,d

if __name__ == '__main__':

    t = np.linspace(0,2*np.pi,1000)
    y= np.sin(t)
    A,D= monte(y,0.3,0.7,1000,2) #generates price matrix and total price from each sim
    expected_price = np.mean(D)
    worst_case =  np.percentile(D,95)
    print('worst case ',worst_case)
    print('expected',expected_price)







