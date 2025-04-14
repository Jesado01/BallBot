import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import pandas as pd

# Import the PIDNN controller and performance evaluator
from utils.evaluator import *
from controllers.pidnn import PIDNN

# Initialize data storage arrays
Pitch = np.array([])
Roll = np.array([])
TimeLog = np.array([])
pidnn_Pitch = np.array([])
pidnn_Roll = np.array([])
m1 = np.array([])
m2 = np.array([])
m3 = np.array([])

# Establish serial connection with Arduino
arduino = serial.Serial('/dev/cu.usbmodem1201', 2000000)
time.sleep(2)

# PIDNN configuration
sample_time = 0.01
setpoint = 0
initial_constants = [30, 0.3, 0.05]  # [Kp, Ki, Kd]

# Initialize the PIDNN controller
pidnn = PIDNN(
    initial_constants = initial_constants,
    learning_rate = 0.0001,
    max_weight_change = 0.001,
    tolerance = 0.001,
    timestep = sample_time
)

# Start experiment timer
start_time = time.time()

try:
    while True:
        # Check if data is available from the Arduino
        if arduino.inWaiting():
            # Read and decode the incoming serial data
            rawString = arduino.readline().strip().decode('ascii')
            if rawString != '':
                rawString = rawString.split("@")
                rawString = np.asfarray(rawString)

                # Parse pitch and roll values
                pitch = float(rawString[0])
                roll = float(rawString[1])

                # Store sensor values
                Pitch = np.append(Pitch, pitch)
                Roll = np.append(Roll, roll)

                # Predict control signals using PIDNN
                pidnn_pitch = pidnn.predict(reference=setpoint, feedback=pitch)
                pidnn_roll = pidnn.predict(reference=setpoint, feedback=roll)

                # Calculate PWM values for 3 motors
                pwm1 = (-0.5 * pidnn_pitch) + (0.866 * pidnn_roll)
                pwm2 = 1.0 * pidnn_roll
                pwm3 = (-0.5 * pidnn_pitch) - (0.866 * pidnn_roll)

                m1 = np.append(m1, pwm1)
                m2 = np.append(m2, pwm2)
                m3 = np.append(m3, pwm3)

                # Format and send control signal to Arduino
                pid = f"{pidnn_pitch}@{pidnn_roll}\n"
                print("Sending:", pid)
                arduino.write(pid.encode('ascii'))

                # Log elapsed time
                current_time = time.time()
                elapsed = current_time - start_time
                TimeLog = np.append(TimeLog, elapsed)

except KeyboardInterrupt:
    print("Execution interrupted by user")

# Close serial connection
arduino.close()

# Create dictionary for CSV export
data_dict = {
    "Pitch": Pitch,
    "Roll": Roll,
    "Time": TimeLog,
    "M1": m1,
    "M2": m2,
    "M3": m3
}

# Convert dictionary to DataFrame
df = pd.DataFrame.from_dict(data_dict, orient="index").transpose()
print(df)

# Save results to CSV
df.to_csv("Control.csv", index=False)

# Read the saved file for plotting
df = pd.read_csv("Control.csv")

# Separate columns
dfPitch = df[["Pitch"]]
dfRoll = df[["Roll"]]
dfM1 = df[["M1"]]
dfM2 = df[["M2"]]
dfM3 = df[["M3"]]
dfTime = df[["Time"]]

# Plot pitch, roll and motor PWM signals
fig, ax = plt.subplots()
ax.plot(dfTime, dfPitch, marker="o", markersize=1, label="Pitch", linestyle='-')
ax.plot(dfTime, dfRoll, marker="o", markersize=1, label="Roll", linestyle='-')
ax.plot(dfTime, dfM1, marker="o", markersize=1, label="M1")
ax.plot(dfTime, dfM2, marker="o", markersize=1, label="M2")
ax.plot(dfTime, dfM3, marker="o", markersize=1, label="M3")
ax.legend()
plt.show()
