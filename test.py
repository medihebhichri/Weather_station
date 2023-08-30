import pandas as pd
import joblib

# Load the trained model
model_filename = 'weather_model.joblib'
clf = joblib.load(model_filename)

# Create a DataFrame for your test data
test_data = {
    'Temperature (C)': [31.13, 31.13, 31.13, ...],  # Replace with your test data
    'Apparent Temperature (C)': [29.00, 29.00, 29.00, ...],
    'Humidity': [0.13, 0.13, 0.13, ...],
    'Wind Speed (km/h)': [0.00, 0.00, 0.00, ...],
    'Wind Bearing (degrees)': [0, 0, 0, ...],
    'Visibility (km)': [16.00, 16.00, 16.00, ...],
    'Loud Cover': [0, 0, 0, ...],
    'Pressure (millibars)': [941.02, 941.02, 941.02, ...],
    'Precip Type': ['rain', 'rain', 'rain', ...],
    'Day': [25, 25, 25, ...],
    'Month': [8, 8, 8, ...],
    'Year': [2023, 2023, 2023, ...],
    'Hour': [10, 10, 10, ...],
    'Minute': [53, 53, 53, ...],
    'Second': [40, 42, 42, ...]
}

test_df = pd.DataFrame(test_data)

# Preprocess the test data
test_df['Precip Type'].fillna('rain', inplace=True)
test_df['Precip Type'] = pd.get_dummies(test_df['Precip Type'], columns=['Precip Type'], drop_first=True)

# Make predictions using the trained model
X_test = test_df.drop(['Dry', 'Light', 'Overcast', 'Mostly', 'Drizzle', 'Rain',
                       'Dangerously', 'Breezy', 'Clear', 'Cloudy', 'Partly',
                       'Humid', 'Windy', 'Foggy'], axis=1)

y_pred = clf.predict(X_test)

# Display the predictions
print("Predicted Weather Conditions:")
for i, col in enumerate(clf.classes_):
    test_df[col] = y_pred[:, i]
    print(f"{col}: {test_df[col].tolist()}")
