import serial
import time
import csv
import os

while True:
    success = False
    try:
        # Open serial port
        arduino = serial.Serial('COM5', 9600, timeout=1)
        success = True
    except:
        print('Please check the port')
        time.sleep(1)
    if success:
        break

# Read the data from the serial port
init_time = time.time()
last_time = time.time()
last_ticks = 0
data = []
print('Reading data...')
time.sleep(0.05)

while time.time() - init_time < 30:
    current_time = time.time()
    rawdata = []

    try:
        pwm = arduino.readline()
        pwm = int( pwm.decode('utf-8').strip('\r\n') )
        pwm = pwm * 257
        rawdata.append(str(pwm))

        ticks = arduino.readline()
        ticks = int( ticks.decode('utf-8').strip('\r\n') )
        if last_ticks == 0:
            last_ticks = ticks
        
        rev = ticks / 1264
        rawdata.append(str(rev))
        rpm = (( (ticks - last_ticks) / 1264 ) / ( current_time - last_time )) * 60
        last_ticks = ticks
        last_time = current_time
        rawdata.append(str(rpm))

        rawdata.append(current_time - init_time)
        data.append( rawdata )
    except:
        print('Error')

    time.sleep(0.1)

# COnvert to csv file
with open('motor_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['PWM', 'Rev','RPM', 'Time'])
    for row in data:
        writer.writerow(row)

# Close the serial port
arduino.close()
print('Done')
print('Generate for matlab')

# Generate a txt file 
# Get data first column as a row
pwm = '['
for row in data:
    pwm += str(row[0] + ', ')
pwm += ']'

# Get data third column as a row
rpm = '['
for row in data:
    rpm += str(row[2]) + ', '
rpm += ']'

# Generate the txt file
with open('motor_data.txt', 'w', newline='') as txtfile:
    txtfile.write('pwm = ' + pwm + '\n')
    txtfile.write('rpm = ' + rpm + '\n')
