import numpy as np
import time as time
import math as math
import random
from chooser import chooser
import requests



SECS_PER_DAY = 300.0
TICKS_PER_DAY = 60
SUNRISE = 15    #Sunrise ticks after start of day
DAY_LENGTH = 30 #Ticks between sunrise and sunset
BASE_DEMAND_PROFILE = [(0,25), (10,25), (20,100), (50,100), (TICKS_PER_DAY,25)] #Piecewise definition of baseline demand
BASE_DEMAND_SCALING = 0.02
DEMAND_MIN = 0
DEMAND_RND_VAR = 1.0
MIN_DEMAND_DURATION = 10
DEF_DEMANDS = [ #List of deferrable demands: (start range), (deadline range), (energy range)
    ((0,0), (TICKS_PER_DAY-1,TICKS_PER_DAY-1), (50.0,50.0)),    #Regular, anytime demand
    ((40,50), (TICKS_PER_DAY-1,TICKS_PER_DAY-1), (20.0,40.0)),  #Evening demand with some variation
    ((0,70), (30,TICKS_PER_DAY-1), (10.0,50.0))               #Unpredictable demand
]
PRICE_MIN = 10
BASE_PRICE = 10.0
BUY_RATIO = 0.5
DEMAND_RND_VAR = 1.0
PRICE_RND_VAR = 20.0
PRICE_SOLAR_DEP = 1.0

def getTick():
    theTime = time.time()
    day = int(theTime/SECS_PER_DAY)
    tick = int(math.fmod(theTime,SECS_PER_DAY)/SECS_PER_DAY*TICKS_PER_DAY) # finds how long into the day you are and then takes that fraction and multiplies by 60 to get the tick
    return day,tick

def getSunlight(tick):
    if tick<= 15 or tick>= 45:
        sun = 0
    else:
        sun = 100 * np.sin(((tick - 15) * np.pi) / 30)
    return sun

def price (demand, sun):
    sell = 10+ (demand - sun) + 20*np.random.normal(0,1)
    return sell

def buy(sell):
    return sell*0.5

def getBaseDemand(tick): # ignore used in the inst demands but makes base
    lastp = (0,0)
    for p in BASE_DEMAND_PROFILE:
        if tick < p[0]:
            demand = int(float(tick-lastp[0])/(float(p[0]-lastp[0])) * (p[1]-lastp[1]) + lastp[1])
            break
        else:
            lastp = p
    return demand

def getInstDemand(day,tick):#takes the base demand and scales it and adds some noise
    baseDemand = getBaseDemand(tick)
    random.seed(day*TICKS_PER_DAY+tick)
    instDemand = baseDemand * BASE_DEMAND_SCALING + random.gauss()*DEMAND_RND_VAR
    if instDemand < DEMAND_MIN:
        instDemand = DEMAND_MIN
    return instDemand


def getDefDemands(day): # the deferred loads
    random.seed(day)
    data = []
    for d in DEF_DEMANDS:
        start = random.randint(*d[0])
        end = random.randint(*d[1])
        energy = random.uniform(*d[2])
        if end-start < MIN_DEMAND_DURATION:
            if start + MIN_DEMAND_DURATION >= TICKS_PER_DAY:
                start = end - MIN_DEMAND_DURATION
            else:
                end = start + MIN_DEMAND_DURATION
        data.append({"start": start, "end": end, "energy": energy})
    return data

def getPrice(day,tick): ## takes the sun and the demand and takes the power
    random.seed(day*TICKS_PER_DAY+tick)
    SupplyVsDemand = float(getBaseDemand(tick)-getSunlight(tick))
    sell = int(BASE_PRICE + SupplyVsDemand * PRICE_SOLAR_DEP + random.gauss()*PRICE_RND_VAR)
    if sell < PRICE_MIN:
        sell = PRICE_MIN
    buy = int(sell * BUY_RATIO)
    return sell, buy

def solar_power(tick):
    Psolar = 3/100 * getSunlight(tick)
    return Psolar

def get_expected_prices():
    expected_prices = []
    for t in range(TICKS_PER_DAY):
        sun = getSunlight(t)
        demand = getBaseDemand(t)
        price = BASE_PRICE + (demand - sun) * PRICE_SOLAR_DEP
        expected_prices.append(max(price, PRICE_MIN))
    return expected_prices

if __name__ == '__main__':
    E_stored = 0
    E_max = 5
    cost =0
    sell_price=0
    last_tick = -1
    current_tick = -2
    try:
        while True :
            print("polling...")
            response = requests.get('https://icelec50015.azurewebsites.net/price')

            data = response.json()

            current_day = data['day']
            price = data['sell_price']
            buy_now = data['buy_price']
            current_tick = data['tick']
            if last_tick != current_tick:



                if (current_tick<=58):
                    demand = getInstDemand(current_day, current_tick)

                    nextprice,next_sell = getPrice(current_day, current_tick+1)

                    Psolar = solar_power(current_tick)

                    P_grid,E_stored =chooser(price, nextprice,Psolar,demand,E_max, E_stored)
                    if P_grid > 0:
                        cost = cost + P_grid*price # the amount of money we have to pay the grid
                    else:
                        sell_price= sell_price + P_grid*buy_now # amount that we sell to the grid
                    last_tick = current_tick
                    print(E_stored,"energy stored BOOM")
                    print("current tick", current_tick)
                    print("current cost",cost)
                    print("overall ",cost+sell_price)

                else:
                    demand = getInstDemand(current_day, current_tick)

                    nextprice, next_sell = getPrice(current_day+1, 0)

                    Psolar = solar_power(current_tick)

                    P_grid, E_stored = chooser(price, nextprice, Psolar, demand, E_max, E_stored)

                    if P_grid > 0:
                        cost = cost + P_grid * price  # the amount of money we have to pay the grid
                    else:
                        sell_price = sell_price + P_grid * buy_now  # amount that we sell to the grid
                    last_tick = current_tick
                    print("current tick",current_tick)
                    print("current cost", cost)
                    print("overall ", cost + sell_price)

            time.sleep(1)
    except KeyboardInterrupt:
        print ("sellFINAL:",sell_price)
        print("cost FINAL" ,cost)
        print("PL FINAL", cost  + sell_price)















