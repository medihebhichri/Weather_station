import serial
import pandas as pd
from datetime import datetime
import joblib

serial_port = 'COM4'
baud_rate = 9600

loaded_model = joblib.load(r'C:\Courses\weather_prediction\weather_model.joblib')
ser = serial.Serial(serial_port, baud_rate)

try:
    data_list = []

    attribute_names = [
        "Temperature (C)",
        "Apparent Temperature (C)",
        "Humidity",
        "Wind Speed (km/h)",
        "Wind Bearing (degrees)",
        "Visibility (km)",
        "Loud Cover	",
        "Pressure (millibars)",

    ]

    while len(data_list) <1000:
        current_datetime = datetime.now()
        day = current_datetime.day
        month = current_datetime.month
        year = current_datetime.year
        hour = current_datetime.hour
        minute = current_datetime.minute
        second = current_datetime.second


        line = ser.readline().decode().strip()
        if len(line) >= 51:
            received_data = line
            split_data = received_data.split("/")
            split_data.extend([day, month, year, hour, minute, second])
            first_iteam=split_data.pop(0)
            split_data.append(first_iteam)

            data_list.append(split_data)
            print(data_list)
            print("\n")

    # Create a DataFrame and save it to a CSV file
    df = pd.DataFrame(data_list, columns=attribute_names + ["day","month","year","hour","minute","second","Precip Type_snow"])
    print(df)
    df.to_csv("weather_samples.csv", index=False)

    print("Data saved to weather_samples.csv")

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
