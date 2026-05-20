import pandas as pd
from pvlpuib.iotools import get_pvgis_hourly

# Imperial College London coordinates
data, meta = get_pvgis_hourly(
    latitude=51.499,
    longitude=-0.179,
    start=2021,
    end=2024,
    pvcalculation=True,
    peakpower=4,        # 4kWp — typical UK house
    loss=14,            # typical system losses in %
    outputformat='json'
)

data.to_csv('solar_data.csv')
print(data.head())