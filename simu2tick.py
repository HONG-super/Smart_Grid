from sys import get_int_max_str_digits

import numpy as np
import time as time
import math as math


import requests


SECS_PER_DAY = 300.0
TICKS_PER_DAY = 60
SUNRISE = 15
DAY_LENGTH = 30
BASE_DEMAND_PROFILE = [(0,25), (10,25), (20,100), (50,100), (TICKS_PER_DAY,25)]
BASE_DEMAND_SCALING = 0.02
DEMAND_MIN = 0
DEMAND_RND_VAR = 1.0
MIN_DEMAND_DURATION = 10
DEF_DEMANDS = [
    ((0,0), (TICKS_PER_DAY-1,TICKS_PER_DAY-1), (50.0,50.0)),
    ((40,50), (TICKS_PER_DAY-1,TICKS_PER_DAY-1), (20.0,40.0)),
    ((0,70), (30,TICKS_PER_DAY-1), (10.0,50.0))
]
PRICE_MIN = 10
BASE_PRICE = 10.0
BUY_RATIO = 0.5
DEMAND_RND_VAR = 1.0
PRICE_RND_VAR = 20.0
PRICE_SOLAR_DEP = 1.0


E_max = 10
P_maxCharge = 1
P_maxDischarge = 1
Tick_duration = 5

charge_Energy = (P_maxCharge *Tick_duration)/E_max
discharge_Energy  = (P_maxDischarge*Tick_duration)/E_max

API_URL = 'https://icelec50015.azurewebsites.net'


def api_price():
    return requests.get(f'{API_URL}/price').json()

def api_demand():
    return requests.get(f'{API_URL}/demand').json()['demand']

def api_deferables():
    return requests.get(f'{API_URL}/deferables').json()


def getTick():
    theTime = time.time()
    day = int(theTime/SECS_PER_DAY)
    tick = int(math.fmod(theTime,SECS_PER_DAY)/SECS_PER_DAY*TICKS_PER_DAY)
    return day,tick

def getSunlight(tick):
    if tick< 15 or tick> 45:
        sun = 0
    else:
        sun = 100 * np.sin(((tick - 15) * np.pi) / 30)
    return sun

def getBaseDemand(tick):
    lastp = (0,0)
    for p in BASE_DEMAND_PROFILE:
        if tick < p[0]:
            demand = int(float(tick-lastp[0])/(float(p[0]-lastp[0])) * (p[1]-lastp[1]) + lastp[1])
            break
        else:
            lastp = p
    return demand





def solar_power(tick):
    Psolar = 3/100 * getSunlight(tick)
    return Psolar

def get_expected_prices():
    expected_prices = []
    for t in range(TICKS_PER_DAY):
        sun = getSunlight(t)
        demand = getBaseDemand(t)
        # Calculate base expected price (no random noise)
        price = BASE_PRICE + (demand - sun) * PRICE_SOLAR_DEP
        expected_prices.append(max(price, PRICE_MIN))
    return expected_prices

def chooser(current_price, hold_charge, solar, load, E_max,expected_future_peak,storedF):
    surplus = solar - load
# free solar  when store it
    if surplus > 0 and storedF<1: # if there is a surplus and the energy stored is less than the max  then store
        # change in the amount stored
        change_storedP = (surplus*Tick_duration)/E_max
        change_storedP = min(change_storedP,charge_Energy,1-storedF) # it finds whether the limit is the max capacity or the charging limit
        storedF = storedF + change_storedP
        # now the amount that is being used
        P_absorbed = change_storedP*E_max /Tick_duration
        P_grid = -(surplus - P_absorbed)
# if the battery is full then sell the rest
    elif surplus > 0 and storedF >=1 : # if surplus is greater than and full then just buy from grid
        P_grid = -surplus
# if there is a deficit, and room in batt
    elif surplus <= 0 and storedF < 1 and expected_future_peak> (2*current_price+20 ): # if there is a deficit and stored< max and low price but also not high demand because then can get expensive
        deficit = load - solar
        change_capacity = min(charge_Energy,1-storedF)
        P_grid =(deficit) + (change_capacity*E_max)/Tick_duration
        storedF = storedF + change_capacity

    elif surplus <= 0 and storedF< 1 and expected_future_peak> (current_price+20 ): # if the peak is greater than the current price + the max noise then charge to save money late r
        deficit = load - solar
        change_capacity = min(charge_Energy, 1 - storedF)
        P_grid = deficit + (change_capacity * E_max) / Tick_duration
        storedF = storedF + change_capacity

    elif surplus < 0 and storedF > 0 and hold_charge: # if there is no surpuls and stored is greater than 0 but it says to hold the charge for a larger peak then use grid to cover the difference
        P_grid = load - solar

    elif surplus < 0 and storedF> 0 and not hold_charge: # if there is a deficit and Estored is greater than 0 and hold charge says go then it is a peak
        deficit = load - solar
        change_capacity = min(discharge_Energy, storedF)
        storedF = storedF - change_capacity
        P_grid = deficit - (change_capacity * E_max) / Tick_duration


    elif surplus < 0 and storedF <= 0: # if deficit and no stored then take from grid
        P_grid = load - solar

    else:
        P_grid = 0

    return P_grid, storedF

#################-TEMPORARY
def naive_chooser(solar, load, E_max, Estored):
    surplus = solar - load

    if surplus > 0:  # Extra solar: Charge battery first, sell the rest
        charge = min(E_max - Estored, surplus * 5, P_maxCharge * Tick_duration)
        Estored = Estored + charge
        P_grid = -(surplus - (charge / 5))

    elif surplus < 0:  # Deficit: Drain battery first, buy the rest
        deficit = load - solar
        discharge = min(deficit * 5, Estored, P_maxDischarge * Tick_duration)
        Estored = Estored - discharge
        P_grid = deficit - (discharge / 5)

    else:
        P_grid = 0

    return P_grid, Estored

def should_hold_charge(current_tick, expected_prices_array, current_price, E_max,storedF):
    worst_future = -1
    worst_tick = -1



    for t in range(current_tick + 1, TICKS_PER_DAY):
        if expected_prices_array[t] > worst_future:
            worst_future = expected_prices_array[t]#searching for the worst price
            worst_tick = t
        if t < TICKS_PER_DAY - 1 and expected_prices_array[t] > expected_prices_array[t + 1]:# if you arent at the final value in the day and the price goes down
            break

    if worst_tick == -1:
        return False


    worse_peak_before_trough = False

    for i in range(current_tick + 1, worst_tick+1):
        if expected_prices_array[i] > current_price + 10: # if the expected price beats the current price by 10 then hold  if the price goes up
            worse_peak_before_trough = True

        sun = getSunlight(i) * 0.03
        demand = getBaseDemand(i) * BASE_DEMAND_SCALING



        if expected_prices_array[i] < current_price: # if  the price goes down
            if worse_peak_before_trough: # if there is then hold
                return True
            return False
    storedF_test = storedF
    for i in range(current_tick+1, worst_tick+1):
        sun = getSunlight(i) * 0.03
        demand = getBaseDemand(i) * BASE_DEMAND_SCALING
        overall = sun - demand
        if overall>0:
            delta = min((overall * Tick_duration) / E_max, charge_Energy, 1.0 - storedF_test)
            storedF_test += delta
    if storedF_test>storedF:
        if worse_peak_before_trough:
            return True
        return False
    return True



def all_3 (price_forecast, deferrable):
    #places 3 chunks at once
    schedule = {}
    for i in range (50):
        # for each one it needs to find the cheapest point in their own range
        for p in deferrable:
            start = p['start']
            end = p['end']
            power_per_chunk = p['energy']/50
            cheapest = 100000
            for j in range (start,end):
                if (price_forecast[j]<cheapest):
                    cheapest = price_forecast[j]
                    cheapest_index = j
            price_forecast[cheapest_index] = price_forecast[cheapest_index] + power_per_chunk
            schedule[cheapest_index]= schedule.get(cheapest_index,0)+ power_per_chunk
    return price_forecast , schedule


def get_next_local_peak(current_tick, prices):
    if current_tick >= len(prices)-1:
        return prices[-1],-1

    for t in range(current_tick, len(prices) - 1):
        if prices[t] > prices[t + 1]:  #finds the peak
            return prices[t],t

    return prices[-1],-1  # no peak then the last one is the peak


def naive_scheduler(deferables):
    naive_schedule = {}
    for p in deferables:
        start = p['start']
        energy_needed = p['energy']

        # Just dump the whole energy block on the very first tick
        naive_schedule[start] = naive_schedule.get(start, 0) + energy_needed

    return naive_schedule













if __name__ == '__main__':

    storeF = 0
    E_max = 10
    cost =0
    sell_price=0
    last_tick = -1
    current_tick = -2
    last_day = -1
    #########################
    naive_Estored = 0
    naive_cost = 0
    naive_sell = 0
    ##########################

    # At initialisation:
    test_Estored = 0
    test_cost = 0
    test_sell = 0

    try:
        while True :
            print("polling...")
            data = api_price()

            current_day = data['day']
            price = data['sell_price']
            buy_now = data['buy_price']
            current_tick = data['tick']

            if current_day != last_day:
                deferables = api_deferables()
                last_day = current_day
                ###########################
                naive_schedule = naive_scheduler(deferables)
                ##########################

                expected_prices_array = get_expected_prices()
                expected_prices_array,schedule= all_3(expected_prices_array,deferables)
                print("Schedule:", schedule)



            if last_tick != current_tick:
                raw_demand = api_demand()
                demand = raw_demand + schedule.get(current_tick, 0)
                naive_demand = raw_demand + naive_schedule.get(current_tick, 0)

                if (current_tick<=58):
                    next_sun = getSunlight(current_tick + 1)
                    next_demand = getBaseDemand(current_tick + 1)
                    nextprice = BASE_PRICE + (next_demand - next_sun) * PRICE_SOLAR_DEP
                else:
                    next_sun = getSunlight(0)
                    next_demand = getBaseDemand(0)
                    nextprice = BASE_PRICE + (next_demand - next_sun) * PRICE_SOLAR_DEP

                Psolar = solar_power(current_tick)
                ############################
                # Inside the tick loop:
                test_demand = raw_demand + schedule.get(current_tick, 0)  # Same smart schedule
                test_Pgrid, test_Estored = naive_chooser(Psolar, test_demand, E_max, test_Estored)

                if test_Pgrid > 0:
                    test_cost += test_Pgrid * price
                else:
                    test_sell += test_Pgrid * buy_now
############################



                hold_charge_flag = should_hold_charge(current_tick, expected_prices_array,price,E_max, storeF) # why would it need the price without the additiaonal load
                future_peak,future_peak_tick = get_next_local_peak(current_tick, expected_prices_array)
                Pgrid, storeF = chooser(price, hold_charge_flag, Psolar, demand, E_max, future_peak,storeF)

                if Pgrid > 0:
                    cost = cost + Pgrid*price

                else:
                    sell_price = sell_price + Pgrid*buy_now
                ################## #- THESE MEAN THAT THEY ARE TEMPORARy
                # Run the naive bot
                naive_Pgrid, naive_Estored = naive_chooser(Psolar, naive_demand, E_max, naive_Estored)

                if naive_Pgrid > 0:
                    naive_cost = naive_cost + (naive_Pgrid * price)
                else:
                    naive_sell = naive_sell + (naive_Pgrid * buy_now)
                ##############
                last_tick = current_tick
                # --- CLEAN SCOREBOARD ---
                # Remove the noisy prints and replace with this structured block
                print(
                    f"\n=== TICK {current_tick:02d} | Solar: {Psolar:.4f} | Base Demand: {demand:.4f} | Price: {price:.2f} ===")

                # Your Bot stats
                print(f"  [YOUR BOT]   Demand: {demand:.4f} | E_stored: {storeF:.2f} | P&L: {(cost + sell_price):.2f}")

                # Naive Bot stats
                print(
                    f"  [NAIVE BOT]  Demand: {naive_demand:.4f} | E_stored: {naive_Estored:.2f} | P&L: {(naive_cost + naive_sell):.2f}")

                print(
                    f"  [SCHED ONLY] Demand: {test_demand:.4f} | E_stored: {test_Estored:.2f} | P&L: {(test_cost + test_sell):.2f}")
                print("-" * 80)  # Draws a separator line

            time.sleep(1)
    except KeyboardInterrupt:
        print ("sellFINAL:",sell_price)
        print("cost FINAL" ,cost)
        print("PL FINAL", cost  + sell_price)
        print("NAIVE PL FINAL", naive_cost + naive_sell)