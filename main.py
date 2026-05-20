import numpy as np
from orsten import pricer
from chooser import chooser, build

if __name__ == '__main__':
    n = 100
    t = np.linspace(0, 2 * np.pi, n)

    # fake daily patterns
    mu_price = 50 + 30 * np.sin(t)        # price peaks midday
    mu_solar = np.maximum(np.sin(t), 0) * 400  # solar only during "day"
    mu_load = 200 + 50 * np.sin(t + 1)    # load peaks slightly later

    E_max = 500
    num_sims = 1000

    smart_costs = []
    naive_costs = []

    for _ in range(num_sims):
        price = pricer(5, 0.3, mu_price, n)
        solar = np.maximum(pricer(20, 0.3, mu_solar, n), 0)  # solar can't be negative
        load = np.maximum(pricer(10, 0.3, mu_load, n), 0)    # load can't be negative

        # smart dispatch
        p_grid, e_stored = build(0, price, solar, load, E_max, n)
        smart_cost = np.sum(price[:n-1] * np.array(p_grid))

        # naive: no flywheel, just buy/sell the deficit
        naive_cost = np.sum(price * (load - solar))

        smart_costs.append(smart_cost)
        naive_costs.append(naive_cost)

    print("=== SMART DISPATCH ===")
    print("Expected cost:", round(np.mean(smart_costs), 2))
    print("Worst case (95%):", round(np.percentile(smart_costs, 95), 2))

    print("\n=== NAIVE (NO FLYWHEEL) ===")
    print("Expected cost:", round(np.mean(naive_costs), 2))
    print("Worst case (95%):", round(np.percentile(naive_costs, 95), 2))

    print("\n=== SAVINGS ===")
    print("Expected saving:", round(np.mean(naive_costs) - np.mean(smart_costs), 2))