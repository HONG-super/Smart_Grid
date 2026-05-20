import numpy as np
import time as time
import math as math
import requests

# Imports from your separate chooser.py file
from chooser import chooser
from chooser import naive_chooser

# ==========================================
# CONSTANTS
# ==========================================
SECS_PER_DAY = 300.0
TICKS_PER_DAY = 60
SUNRISE = 15
DAY_LENGTH = 30
BASE_DEMAND_PROFILE = [(0, 25), (10, 25), (20, 100), (50, 100), (TICKS_PER_DAY, 25)]
BASE_DEMAND_SCALING = 0.02
DEMAND_MIN = 0
DEMAND_RND_VAR = 1.0
MIN_DEMAND_DURATION = 10
DEF_DEMANDS = [
    ((0, 0), (TICKS_PER_DAY - 1, TICKS_PER_DAY - 1), (50.0, 50.0)),
    ((40, 50), (TICKS_PER_DAY - 1, TICKS_PER_DAY - 1), (20.0, 40.0)),
    ((0, 70), (30, TICKS_PER_DAY - 1), (10.0, 50.0))
]
PRICE_MIN = 10
BASE_PRICE = 10.0
BUY_RATIO = 0.5
DEMAND_RND_VAR = 1.0
PRICE_RND_VAR = 20.0
PRICE_SOLAR_DEP = 1.0
API_URL = 'https://icelec50015.azurewebsites.net'


# ==========================================
# API FUNCTIONS
# ==========================================
def api_price():
    return requests.get(f'{API_URL}/price').json()


def api_demand():
    return requests.get(f'{API_URL}/demand').json()['demand']


def api_deferables():
    return requests.get(f'{API_URL}/deferables').json()


# ==========================================
# MATH & GRID HELPERS
# ==========================================
def getTick():
    theTime = time.time()
    day = int(theTime / SECS_PER_DAY)
    tick = int(math.fmod(theTime, SECS_PER_DAY) / SECS_PER_DAY * TICKS_PER_DAY)
    return day, tick


def getSunlight(tick):
    if tick < 15 or tick > 45:
        sun = 0
    else:
        sun = 100 * np.sin(((tick - 15) * np.pi) / 30)
    return sun


def getBaseDemand(tick):
    lastp = (0, 0)
    for p in BASE_DEMAND_PROFILE:
        if tick < p[0]:
            demand = int(float(tick - lastp[0]) / (float(p[0] - lastp[0])) * (p[1] - lastp[1]) + lastp[1])
            break
        else:
            lastp = p
    return demand


def solar_power(tick):
    Psolar = 3 / 100 * getSunlight(tick)
    return Psolar


def get_expected_prices():
    expected_prices = []
    for t in range(TICKS_PER_DAY):
        sun = getSunlight(t)
        demand = getBaseDemand(t)
        price = BASE_PRICE + (demand - sun) * PRICE_SOLAR_DEP
        expected_prices.append(max(price, PRICE_MIN))
    return expected_prices


# ==========================================
# LOGIC FUNCTIONS
# ==========================================
def should_hold_charge(current_tick, expected_prices_array, current_price, Estored, E_max):
    worst_future = -1
    worst_tick = -1

    for t in range(current_tick + 1, TICKS_PER_DAY):
        if expected_prices_array[t] > worst_future:
            worst_future = expected_prices_array[t]
            worst_tick = t
        if t < TICKS_PER_DAY - 1 and expected_prices_array[t] > expected_prices_array[t + 1]:
            break

    if worst_tick == -1:
        return False

    expected_surplus = 0
    worse_peak_before_trough = False

    for i in range(current_tick + 1, worst_tick + 1):
        if expected_prices_array[i] > current_price + 5:
            worse_peak_before_trough = True

        sun = getSunlight(i) * 0.03
        demand = getBaseDemand(i) * BASE_DEMAND_SCALING

        if sun > demand:
            expected_surplus = expected_surplus + (sun - demand)

        if expected_prices_array[i] < current_price:
            if worse_peak_before_trough:
                return True
            return False

    energy_deficit = E_max - Estored
    if expected_surplus > energy_deficit:
        if worse_peak_before_trough:
            return True
        return False

    return True


def get_next_local_peak(current_tick, prices):
    if current_tick >= len(prices) - 1:
        return prices[-1]

    for t in range(current_tick, len(prices) - 1):
        if prices[t] > prices[t + 1]:
            return prices[t]

    return prices[-1]


# ==========================================
# SCHEDULERS (THE BATTLE)
# ==========================================
def all_3(price_forecast, deferrable):
    # YOUR BOT: Uses the virtual battery strategy
    schedule = {}
    for i in range(50):
        for p in deferrable:
            start = p['start']
            end = p['end']
            power_per_chunk = p['energy'] / 50
            cheapest = 100000
            cheapest_index = start

            for j in range(start, end):

                if price_forecast[j] < cheapest:
                    cheapest = price_forecast[j]
                    cheapest_index = j

            price_forecast[cheapest_index] = price_forecast[cheapest_index] + power_per_chunk
            schedule[cheapest_index] = schedule.get(cheapest_index, 0) + power_per_chunk
    return price_forecast, schedule


def claude_scheduler(price_forecast, deferrable):
    # CLAUDE'S BOT: Forces loads out of the solar window
    schedule = {}
    for i in range(50):
        for p in deferrable:
            start = p['start']
            end = p['end']
            power_per_chunk = p['energy'] / 50

            cheapest = 100000
            cheapest_index = start

            for j in range(start, end):
                # Skip the solar window if there's room outside of it
                if 20 <= j <= 40 and (end - start > 20):
                    continue
                if price_forecast[j] < cheapest:
                    cheapest = price_forecast[j]
                    cheapest_index = j

            price_forecast[cheapest_index] = price_forecast[cheapest_index] + power_per_chunk
            schedule[cheapest_index] = schedule.get(cheapest_index, 0) + power_per_chunk
    return price_forecast, schedule


# ==========================================
# MAIN EXECUTION LOOP
# ==========================================
if __name__ == '__main__':

    # Optimized Bot Trackers (all_3)
    Estored = 0
    E_max = 10
    cost = 0
    sell_price = 0

    # Baseline Bot Trackers (Claude)
    naive_Estored = 0
    naive_cost = 0
    naive_sell = 0

    last_tick = -1
    current_tick = -2
    last_day = -1

    try:
        while True:
            # print("polling...") # Optional: uncomment if you want to see the wait loop
            data = api_price()

            current_day = data['day']
            price = data['sell_price']
            buy_now = data['buy_price']
            current_tick = data['tick']

            # --- DAILY RESET ---
            if current_day != last_day:
                deferables = api_deferables()
                last_day = current_day

                # 1. Generate Claude's Schedule
                expected_prices_claude = get_expected_prices()
                _, claude_schedule_dict = claude_scheduler(expected_prices_claude, deferables)

                # 2. Generate YOUR Schedule
                expected_prices_array = get_expected_prices()
                expected_prices_array, schedule = all_3(expected_prices_array, deferables)

                print("\n=== NEW DAY STARTED: Schedules Built ===")

            # --- TICK UPDATE ---
            if last_tick != current_tick:
                # Ask the server exactly ONCE to prevent network lag
                base_demand = api_demand()
                Psolar = solar_power(current_tick)

                # Calculate the unique physical demand for each house
                demand = base_demand + schedule.get(current_tick, 0)
                naive_demand = base_demand + claude_schedule_dict.get(current_tick, 0)

                # --- RUN YOUR OPTIMIZED BOT ---
                hold_charge_flag = should_hold_charge(current_tick, expected_prices_array, price, Estored, E_max)
                future_peak = get_next_local_peak(current_tick, expected_prices_array)

                Pgrid, Estored = chooser(price, hold_charge_flag, Psolar, demand, E_max, Estored, future_peak)

                if Pgrid > 0:
                    cost = cost + (Pgrid * price)
                else:
                    sell_price = sell_price + (Pgrid * buy_now)

                # --- RUN CLAUDE'S BOT ---
                naive_Pgrid, naive_Estored = naive_chooser(Psolar, naive_demand, E_max, naive_Estored)

                if naive_Pgrid > 0:
                    naive_cost = naive_cost + (naive_Pgrid * price)
                else:
                    naive_sell = naive_sell + (naive_Pgrid * buy_now)

                last_tick = current_tick

                # --- LIVE SCOREBOARD ---
                print(
                    f"tick:{current_tick:02d} | solar:{Psolar:.4f} | base_demand:{base_demand:.4f} | price:{price:.2f}")
                print(f"  [YOUR BOT]   demand:{demand:.4f} | E_stored:{Estored:.2f} | P&L: {(cost + sell_price):.2f}")
                print(
                    f"  [CLAUDE BOT] demand:{naive_demand:.4f} | E_stored:{naive_Estored:.2f} | P&L: {(naive_cost + naive_sell):.2f}\n")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n=== FINAL DAILY RESULTS ===")
        print(f"YOUR OPTIMIZED P&L: {cost + sell_price:.2f}")
        print(f"CLAUDE NAIVE P&L:   {naive_cost + naive_sell:.2f}")

        diff = (naive_cost + naive_sell) - (cost + sell_price)
        if diff > 0:
            print(f"VERDICT: You beat Claude by {diff:.2f}!")
        else:
            print(f"VERDICT: Claude beat you by {abs(diff):.2f}!")