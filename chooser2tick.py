def chooser(current_price, hold_charge, solar, load, E_max, Estored,expected_future_peak,storedP):
    surplus = solar - load
# free solar  when store it
    if surplus > 0 and storedP<1: # if there is a surplus and the energy stored is less than the max  then store
        # change in the amount stored



        charge = min(E_max - Estored, surplus * 5)
        Estored = Estored + charge
        P_grid = -(surplus - (charge / 5))
# if the battery is full then sell the rest
    elif surplus > 0 and Estored >= E_max: # if surplus is greater than and full then just buy from grid
        P_grid = -surplus
# if there is a deficit, and room in batt
    elif surplus <= 0 and Estored < E_max and expected_future_peak> (2*current_price+20 ): # if there is a deficit and stored< max and low price but also not high demand because then can get expensive
        deficit = load - solar
        charge_needed = E_max - Estored
        Estored = E_max
        P_grid = deficit + (charge_needed / 5)

    elif surplus <= 0 and Estored < E_max and expected_future_peak> (current_price+20 ): # if the peak is greater than the current price + the max noise then charge to save money late r
        deficit = load - solar
        charge_needed = E_max - Estored
        Estored = E_max
        P_grid = deficit + (charge_needed / 5)

    elif surplus < 0 and Estored > 0 and hold_charge: # if there is no surpuls and stored is greater than 0 but it says to hold the charge for a larger peak then use grid to cover the difference
        P_grid = load - solar

    elif surplus < 0 and Estored > 0 and not hold_charge: # if there is a deficit and Estored is greater than 0 and hold charge says go then it is a peak
        deficit = load - solar
        discharge = min(deficit * 5, Estored)
        Estored = Estored - discharge
        P_grid = deficit - (discharge / 5)

    elif surplus < 0 and Estored <= 0: # if deficit and no stored then take from grid
        P_grid = load - solar

    else:
        P_grid = 0

    return P_grid, Estored

#################-TEMPORARY
def naive_chooser(solar, load, E_max, Estored):
    surplus = solar - load

    if surplus > 0:  # Extra solar: Charge battery first, sell the rest
        charge = min(E_max - Estored, surplus * 5)
        Estored = Estored + charge
        P_grid = -(surplus - (charge / 5))

    elif surplus < 0:  # Deficit: Drain battery first, buy the rest
        deficit = load - solar
        discharge = min(deficit * 5, Estored)
        Estored = Estored - discharge
        P_grid = deficit - (discharge / 5)

    else:
        P_grid = 0

    return P_grid, Estored