import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import pandas as pd

# Establish serial communication with the Arduino
arduino = serial.Serial('/dev/cu.usbmodem11101', 2000000)
time.sleep(2)  # Wait for the connection to stabilize

# Initialize arrays to store sensor and motor data
pitch = np.array([])
roll = np.array([])
m1 = np.array([])
m2 = np.array([])
m3 = np.array([])
Timetime = np.array([])

# PID controller parameters (not used directly here, but can be enabled)
kp = 0.3
ki = 1.6
kd = 0.03
Tm = 0.105
cv1 = 0
error1 = 0
error2 = 0

# Optional: send initialization byte to Arduino
arduino.write(b'0')

# Start timer
start_time = time.time()

try:
    while True:
        # Check if serial data is available
        if arduino.inWaiting():
            # Read and decode serial input
            rawString = arduino.readline().strip().decode("utf-8") 
            print(rawString)

            # Skip empty lines
            if rawString != '':
                # Split the string by '@' and convert to float
                rawString = rawString.split("@")
                rawString = np.asfarray(rawString)
            
            # Validate received data structure
            if len(rawString) >= 5:
                pwm1 = rawString[0]
                pwm2 = rawString[1]
                pwm3 = rawString[2]
                p = rawString[3]
                r = rawString[4]

                # Append data to arrays
                m1 = np.append(m1, pwm1)
                m2 = np.append(m2, pwm2)
                m3 = np.append(m3, pwm3)
                pitch = np.append(pitch, p)
                roll = np.append(roll, r)

            # Store current timestamp
            elapsed_time = time.time() - start_time
            Timetime = np.append(Timetime, elapsed_time)

except KeyboardInterrupt:
    print("Execution interrupted by user")

# Close the serial connection
arduino.close()

# Organize data into a dictionary
data_dict = {
    "Pitch": pitch,
    "Roll": roll,
    "Time": Timetime,
    "M1": m1,
    "M2": m2,
    "M3": m3
}

# Convert to DataFrame and save as CSV
df = pd.DataFrame.from_dict(data_dict, orient="index").transpose()
df.to_csv("IMU6050_nn.csv", index=False)
