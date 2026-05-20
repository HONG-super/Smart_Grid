import requests as rq
from datetime import datetime,  timedelta
import pandas as pd


start_date = datetime(2018, 1, 1)
end_date = datetime(2026, 1, 1)
current = start_date
week_data = []
while current < end_date:
    next_date = current+ timedelta(days = 7)

    params = {
        'from': current.strftime('%Y-%m-%dT00:00Z'),
        'to': next_date.strftime('%Y-%m-%dT00:00Z'),
        'dataProviders':'APXMIDP',
        'format': 'json'
    }
    data =rq.get('https://data.elexon.co.uk/bmrs/api/v1/balancing/pricing/market-index?', params=params)
    if (data.status_code == 200):
        data = data.json()['data']
        week_data.extend(data)
    else :
        print(f"Error {data.status_code} for {current}")
    current = next_date
df = pd.DataFrame(week_data)
df.to_csv('week_data.csv')





