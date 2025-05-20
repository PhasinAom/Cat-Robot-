from machine import Pin, PWM, time_pulse_us, I2C, \
    Timer  # Import modules; Pin is used to control GPIO pins for input and output, PWM for PWM output, time_pulse_us for measuring pulse width, I2C for controlling the I2C bus, Timer for creating timers
import time  # Import module for handling time
import math  # Import module for performing mathematical operations
import random  # Import module for generating random numbers
import bluetooth  # Import module for communicating with Bluetooth devices

# Bluetooth app sends commands as follows, note that all are lowercase single letters
# Send 'x': Stop the current action
# Send 'c': Enter servo calibration mode
# Send 'f': Move forward
# Send 'b': Move backward
# Send 'l': Turn left
# Send 'r': Turn right
# Send 'w': Walking posture
# Send 's': Step in place
# Send 't': Sit down
# Send 'h': Shake hands
# Send 'm': Automatic following
# Send 'a': Automatic walking
# Send 'y': Swing action
# Send 'u': Lie down and get up action
# Send 'k': Kick the ball
# Send 'i': Stand balance
# Send 'o': Obstacle avoidance mode

# ================================================
angleC2 = 72  # Calibration parameter for servo, same below
angleC4 = 100  # 100
angleC5 = 141  # 141
angleC12 = 98  # 98
angleC13 = 100  # 100
angleC14 = 135  # 135
angleC15 = 53  # 53
angleC25 = 70  # 70
angleC26 = 37  # 37
angleC27 = 120  # 120
angleC32 = 70  # 70
angleC33 = 43  # 43
# =================================================

pin2 = Pin(2, Pin.OUT)  # Create a GPIO pin object, set to output mode, same below
servo0 = PWM(pin2, freq=50)  # Create a PWM object, set frequency to 50, same below
pin4 = Pin(4, Pin.OUT)
servo1 = PWM(pin4, freq=50)
pin5 = Pin(5, Pin.OUT)
servo2 = PWM(pin5, freq=50)
pin12 = Pin(12, Pin.OUT)
servo3 = PWM(pin12, freq=50)
pin13 = Pin(13, Pin.OUT)
servo4 = PWM(pin13, freq=50)
pin14 = Pin(14, Pin.OUT)
servo5 = PWM(pin14, freq=50)
pin15 = Pin(15, Pin.OUT)
servo6 = PWM(pin15, freq=50)
pin25 = Pin(25, Pin.OUT)
servo7 = PWM(pin25, freq=50)
pin26 = Pin(26, Pin.OUT)
servo8 = PWM(pin26, freq=50)
pin27 = Pin(27, Pin.OUT)
servo9 = PWM(pin27, freq=50)
pin32 = Pin(32, Pin.OUT)
servo10 = PWM(pin32, freq=50)
pin33 = Pin(33, Pin.OUT)
servo11 = PWM(pin33, freq=50)

trig = Pin(23, Pin.OUT,
           value=0)  # Create a GPIO pin object, set to output mode, initial value 0, low level. This pin is used to control the trigger signal of the ultrasonic sensor
echo = Pin(22,
           Pin.IN)  # Create a GPIO pin object, set to input mode. This pin is used to receive the echo signal of the ultrasonic sensor

Pi = const(3.1416)  # Set the constant pi for calculations
H = const(70)  # Set constant, height above ground, see "Posture Calculation Analysis" for details
hipL = const(40)  # Set constant, leg length, see "Posture Calculation Analysis" for details
legL = const(58)  # Set constant, foot length, see "Posture Calculation Analysis" for details
act = 1  # Set a variable for exiting the action loop
BLE_MSG = ""  # Define an empty variable to receive Bluetooth data


class ESP32_BLE():  # Define a class
    def __init__(self, name):  # Define a function called __init__, which requires two variable parameters
        self.timer1 = Timer(0)  # Timer, ESP32 has 4 hardware timers, here 0 is used
        self.name = name  # The parameter name passed in
        self.ble = bluetooth.BLE()  # Create BLE object
        self.ble.active(True)  # Activate Bluetooth
        self.ble.config(gap_name=name)  # Configure Bluetooth, give Bluetooth a name
        self.disconnected()  # Call function to execute timer
        self.ble.irq(
            self.ble_irq)  # Bluetooth calls interrupt function. When the phone sends data to ESP32, ESP32 Bluetooth receives the data and automatically executes this interrupt
        self.register()  # Register
        self.advertiser()  # Broadcast

    def connected(self):  # Define a function, same below
        self.timer1.deinit()  # Cancel timer

    def disconnected(self):
        pass  # Do nothing

    def ble_irq(self, event, data):
        global BLE_MSG  # Declare the use of the global variable BLE_MSG in this function
        if event == 1:  # _IRQ_CENTRAL_CONNECT The phone is connected to this device
            self.connected()  # Call function, achieve the effect of blinking when not connected, and long light when connected
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT The phone is disconnected from this device
            self.advertiser()  # Call function, perform Bluetooth broadcast
            self.disconnected()  # After Bluetooth is disconnected, the blue LED will blink again
        if event == 3:  # _IRQ_GATTS_WRITE The phone sends data to this device
            buffer = self.ble.gatts_read(self.rx)  # Receive data from the phone
            BLE_MSG = buffer.decode(
                'UTF-8').strip()  # Decode the received data according to utf-8, so the data sent by the phone should be utf-8 encoded, and assign the received data to the variable BLE_MSG

    def register(self):
        service_uuid = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'  # Define the UUID of the send/receive service (unique identifier)
        reader_uuid = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'  # Define RX port identifier
        sender_uuid = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # Define TX port identifier

        # Bluetooth works through services
        services = (
            # The parentheses here represent a service
            (bluetooth.UUID(service_uuid),  # Service ID
             (
                 (bluetooth.UUID(sender_uuid), bluetooth.FLAG_NOTIFY),  # Service type: notify
                 (bluetooth.UUID(reader_uuid), bluetooth.FLAG_WRITE),  # Service type: write
             )
             ),
            # Other services can be defined below
            # -- Other services
        )

        ((self.tx, self.rx,),) = self.ble.gatts_register_services(services)  # Register services

    def send(self, data):
        self.ble.gatts_notify(0, self.tx, data + '\n')  # Bluetooth sends notify information

    def advertiser(self):  # Start broadcasting when disconnected
        name = bytes(self.name, 'UTF-8')  # Convert device name to utf-8 format
        adv_data = bytearray('\x02\x01\x02') + bytearray((len(name) + 1,
                                                          0x09)) + name  # Create a byte array containing advertising data. This array includes the length and type of advertising data, as well as the device name.
        self.ble.gap_advertise(100,
                               adv_data)  # Use BLE object's gap_advertise() method to start broadcasting. The first parameter is the broadcasting interval (in milliseconds), and the second parameter is the advertising data
        print(adv_data)  # Print advertising data
        print("\r\n")  # Print a blank line


def servowrite(servo_pwm, degree):  # Define a function to control the servo to rotate to a specific angle
    val = int(
        degree * 0.56) + 25  # Calculate the PWM duty cycle for the servo. The servo's angle range is usually between 0 to 180 degrees, so the angle needs to be converted to a PWM duty cycle
    servo_pwm.duty(val)  # Set PWM duty cycle to control the actual position of the servo


# The following lines set the initial rotation angle for all servos at startup, which is the bent leg standing posture
servowrite(servo0,
           angleC2)  # Define servo0 to rotate to angleC2, see assembly tutorial video for specific servo0, same below
servowrite(servo1, angleC4 - 50)
servowrite(servo2, angleC5 - 85)
servowrite(servo3, angleC12)
servowrite(servo4, angleC13 - 45)
servowrite(servo5, angleC14 - 73)
servowrite(servo6, angleC15)
servowrite(servo7, angleC25 + 45)
servowrite(servo8, angleC26 + 73)
servowrite(servo9, angleC27)
servowrite(servo10, angleC32 + 50)
servowrite(servo11, angleC33 + 85)


def servoread(servo):  # Define a function to read the current angle position of the servo
    duty = servo.duty()  # Read the current PWM duty cycle value of the servo
    angle = (duty - 25) * 1.8  # Convert to angle
    return angle  # Return the angle


# The following svmovea and svmoveb functions are used when the servo's current angle is greater or less than the target angle, respectively, to move to the target angle at a certain speed
def svmovea(servo, angleA):
    for i in range(servoread(servo), angleA,
                   -1):  # For loop statement, using servoread() function to get the current servo position, and move from the current position to the target position. This loop starts at the current position and decrements to the target angle
        servowrite(servo, i)  # Continuously set the corresponding servo angle as the loop variable i changes
        time.sleep_ms(5)  # Each loop interval is 5 milliseconds


def svmoveb(servo, angleA):
    for i in range(servoread(servo),
                   angleA):  # For loop statement, using servoread() function to get the current servo position, and move from the current position to the target position. This loop starts at the current position and increments to the target angle
        servowrite(servo, i)  # Continuously set the corresponding servo angle as the loop variable i changes
        time.sleep_ms(5)  # Each loop interval is 5 milliseconds


def stand():  # Define a function to make the robot cat change from any posture to the bent leg standing posture
    servowrite(servo0, angleC2)  # Make servo0 rotate to angleC2, same below
    if servoread(servo1) < angleC4 - 50:  # If the current angle value of servo1 is less than angleC4-50
        svmoveb(servo1, angleC4 - 50)  # Use svmoveb function to make servo1 rotate to angleC4-50
    elif servoread(servo1) > angleC4 - 50:  # If the current angle value of servo1 is greater than angleC4-50
        svmovea(servo1, angleC4 - 50)  # Use svmovea function to make servo1 rotate to angleC4-50
    if servoread(servo2) < angleC5 - 85:  # Same as above
        svmoveb(servo2, angleC5 - 85)
    elif servoread(servo2) > angleC5 - 85:
        svmovea(servo2, angleC5 - 85)
    servowrite(servo3, angleC12)
    if servoread(servo4) < angleC13 - 45:
        svmoveb(servo4, angleC13 - 45)
    elif servoread(servo4) > angleC13 - 45:
        svmovea(servo4, angleC13 - 45)
    if servoread(servo5) < angleC14 - 73:
        svmoveb(servo5, angleC14 - 73)
    elif servoread(servo5) > angleC14 - 73:
        svmovea(servo5, angleC14 - 73)
    servowrite(servo6, angleC15)
    if servoread(servo7) < angleC25 + 45:
        svmoveb(servo7, angleC25 + 45)
    elif servoread(servo7) > angleC25 + 45:
        svmovea(servo7, angleC25 + 45)
    if servoread(servo8) < angleC26 + 73:
        svmoveb(servo8, angleC26 + 73)
    elif servoread(servo8) > angleC26 + 73:
        svmovea(servo8, angleC26 + 73)
    servowrite(servo9, angleC27)
    if servoread(servo10) < angleC32 + 50:
        svmoveb(servo10, angleC32 + 50)
    elif servoread(servo10) > angleC32 + 50:
        svmovea(servo10, angleC32 + 50)
    if servoread(servo11) < angleC33 + 85:
        svmoveb(servo11, angleC33 + 85)
    elif servoread(servo11) > angleC33 + 85:
        svmovea(servo11, angleC33 + 85)


def calibration():  # Define a function to make the robot cat change from any posture to the servo calibration posture
    if servoread(servo0) < angleC2:  # If the current angle value of servo0 is less than angleC2
        svmoveb(servo0, angleC2)  # Use svmoveb function to make servo0 rotate to angleC2
    elif servoread(servo0) > angleC2:  # If the current angle value of servo0 is greater than angleC2
        svmovea(servo0, angleC2)  # Use svmovea function to make servo0 rotate to angleC2
    if servoread(servo1) < angleC4:  # Same as above
        svmoveb(servo1, angleC4)
    elif servoread(servo1) > angleC4:
        svmovea(servo1, angleC4)
    if servoread(servo2) < angleC5:
        svmoveb(servo2, angleC5)
    elif servoread(servo2) > angleC5:
        svmovea(servo2, angleC5)
    if servoread(servo3) < angleC12:
        svmoveb(servo3, angleC12)
    elif servoread(servo3) > angleC12:
        svmovea(servo3, angleC12)
    if servoread(servo4) < angleC13:
        svmoveb(servo4, angleC13)
    elif servoread(servo4) > angleC13:
        svmovea(servo4, angleC13)
    if servoread(servo5) < angleC14:
        svmoveb(servo5, angleC14)
    elif servoread(servo5) > angleC14:
        svmovea(servo5, angleC14)
    if servoread(servo6) < angleC15:
        svmoveb(servo6, angleC15)
    elif servoread(servo6) > angleC15:
        svmovea(servo6, angleC15)
    if servoread(servo7) < angleC25:
        svmoveb(servo7, angleC25)
    elif servoread(servo7) > angleC25:
        svmovea(servo7, angleC25)
    if servoread(servo8) < angleC26:
        svmoveb(servo8, angleC26)
    elif servoread(servo8) > angleC26:
        svmovea(servo8, angleC26)
    if servoread(servo9) < angleC27:
        svmoveb(servo9, angleC27)
    elif servoread(servo9) > angleC27:
        svmovea(servo9, angleC27)
    if servoread(servo10) < angleC32:
        svmoveb(servo10, angleC32)
    elif servoread(servo10) > angleC32:
        svmovea(servo10, angleC32)
    if servoread(servo11) < angleC33:
        svmoveb(servo11, angleC33)
    elif servoread(servo11) > angleC33:
        svmovea(servo11, angleC33)


def stepangleL(anglehA, anglehC,
               anglelC):  # Define a function to calculate the rotation angle of the left foot servo during the in-place stepping action, see the tutorial "Posture Calculation Analysis" for details on the calculation formula
    angleh = abs(anglehA - anglehC)  # abs is to take the absolute value
    radn = angleh * 0.9 * Pi / 180
    L = math.sin(radn) * hipL
    angleleg = math.asin(L / legL) * 180 / Pi
    svangleleg = anglelC + (angleleg / 0.9 + angleh)
    return svangleleg  # Return svangleleg


def stepangleR(anglehA, anglehC,
               anglelC):  # Define a function to calculate the rotation angle of the right foot servo during the in-place stepping action, see the tutorial "Posture Calculation Analysis" for details on the calculation formula
    angleh = abs(anglehA - anglehC)  # abs is to take the absolute value
    radn = angleh * 0.9 * Pi / 180
    L = math.sin(radn) * hipL
    angleleg = math.asin(L / legL) * 180 / Pi
    svangleleg = anglelC - (angleleg / 0.9 + angleh)
    return svangleleg  # Return svangleleg


def stepwalk():  # In-place stepping action
    if act == 1:  # Check if the global variable act equals 1
        for i, j, k, l in zip(range(angleC4 - 75, angleC4 - 35), range(angleC32 + 35, angleC32 + 75),
                              range(angleC13 - 25, angleC13 - 65, -1), range(angleC25 + 65, angleC25 + 25, -1)):
            # for loop statement, assign different loop ranges to variables i, j, k, l, -1 means the values decrease in the loop
            servowrite(servo1, i)  # Use servowrite function to make servo1 rotate to angle i, same below
            servowrite(servo4, k)
            servowrite(servo7, l)
            servowrite(servo10, j)
            a = stepangleR(i, angleC4,
                           angleC5)  # Use stepangleR function to calculate the target value for the foot servo, and assign the returned value to variable a, same below
            b = stepangleR(k, angleC13, angleC14)
            c = stepangleL(l, angleC25, angleC26)
            d = stepangleL(j, angleC32, angleC33)
            servowrite(servo2, a)
            servowrite(servo5, b)
            servowrite(servo8, c)
            servowrite(servo11, d)
            time.sleep_ms(10);  # Each loop interval is 10 milliseconds
        stopcommand()  # Execute stopcommand function to check if a stop command has been received, if so, stop the current action, same below
    if act == 1:  # Same as above
        for i, j, k, l in zip(range(angleC4 - 35, angleC4 - 75, -1), range(angleC32 + 75, angleC32 + 35, -1),
                              range(angleC13 - 65, angleC13 - 25), range(angleC25 + 25, angleC25 + 65)):
            servowrite(servo1, i)
            servowrite(servo4, k)
            servowrite(servo7, l)
            servowrite(servo10, j)
            a = stepangleR(i, angleC4, angleC5)
            b = stepangleR(k, angleC13, angleC14)
            c = stepangleL(l, angleC25, angleC26)
            d = stepangleL(j, angleC32, angleC33)
            servowrite(servo2, a)
            servowrite(servo5, b)
            servowrite(servo8, c)
            servowrite(servo11, d)
            time.sleep_ms(10);
        stopcommand()


def legangleL(anglehA, anglehC,
              anglelC):  # Define a function to calculate the rotation angle of the left foot servo during the walking action, see the tutorial "Posture Calculation Analysis" for details on the calculation formula
    angleh = abs(anglehA - anglehC)  # abs is to take the absolute value
    radn = angleh * 0.9 * Pi / 180
    h1 = math.cos(radn) * hipL
    h2 = H - h1
    angleleg = math.acos(h2 / legL) * 180 / Pi
    svangleleg = angleleg / 0.9 + angleh + anglelC - 10
    return svangleleg  # Return svangleleg


def legangleR(anglehA, anglehC,
              anglelC):  # Define a function to calculate the rotation angle of the right foot servo during the walking action, see the tutorial "Posture Calculation Analysis" for details on the calculation formula
    angleh = abs(anglehA - anglehC)  # abs is to take the absolute value
    radn = angleh * 0.9 * Pi / 180
    h1 = math.cos(radn) * hipL
    h2 = H - h1
    angleleg = math.acos(h2 / legL) * 180 / Pi
    svangleleg = anglelC - (angleleg / 0.9 + angleh) + 10
    return svangleleg  # Return svangleleg

def runA():  # Walking action part one, see the tutorial "Posture Calculation Analysis" for a detailed explanation of the action
    if act == 1:  # Check if the global variable act equals 1
        for a, b, c, d in zip(range(angleC25 + 22, angleC25 + 37), range(angleC32 + 57, angleC32 + 72),
                              range(angleC13 - 54, angleC13 - 69, -1), range(angleC4 - 25, angleC4 - 41, -1)):
            # for loop statement, assign different loop ranges to variables a, b, c, d, -1 means the values decrease in the loop
            servowrite(servo7, a)  # Use servowrite function to make servo7 rotate to angle a, same below
            servowrite(servo10, b)
            servowrite(servo4, c)
            servowrite(servo1, d)
            i = legangleL(a, angleC25,
                          angleC26)  # Use legangleL function to calculate the target value for the foot servo, and assign the returned value to variable i, same below
            j = legangleL(b, angleC32, angleC33)
            k = legangleR(c, angleC13, angleC14)
            l = legangleR(d, angleC4, angleC5)
            servowrite(servo8, i - 10)  # -10 raises the tail to stabilize the walking posture
            servowrite(servo11, j)
            servowrite(servo5, k + 10)  # +10 raises the tail to stabilize the walking posture
            servowrite(servo2, l)
            time.sleep_ms(10)  # Each loop interval is 10 milliseconds
        stopcommand()  # Execute stopcommand function to check if a stop command has been received, if so, stop the current action, same below


def runB():  # Walking action part two, see the tutorial "Posture Calculation Analysis" for a detailed explanation of the action, code explanation same as runA()
    if act == 1:
        for a, b, c, d in zip(range(angleC25 + 38, angleC25 + 53), range(angleC32 + 72, angleC32 + 25, -3),
                              range(angleC13 - 69, angleC13 - 22, +3), range(angleC4 - 41, angleC4 - 57, -1)):
            servowrite(servo7, a)
            servowrite(servo10, b)
            servowrite(servo4, c)
            servowrite(servo1, d)
            i = legangleL(a, angleC25, angleC26)
            j = legangleL(b, angleC32, angleC33)
            k = legangleR(c, angleC13, angleC14)
            l = legangleR(d, angleC4, angleC5)
            servowrite(servo8, i - 10)  # -10 raises the tail to stabilize the walking posture
            servowrite(servo11, j + 10)  # +10 this phase this leg needs to lift off the ground and move forward
            servowrite(servo5,
                       k)  # This was originally +10 to raise the tail, which cancels out with -10 to move the leg forward
            servowrite(servo2, l)
            time.sleep_ms(10)
        stopcommand()


def runC():  # Walking action part three, see the tutorial "Posture Calculation Analysis" for a detailed explanation of the action, code explanation same as runA()
    if act == 1:
        for a, b, c, d in zip(range(angleC25 + 54, angleC25 + 69), range(angleC32 + 25, angleC32 + 40),
                              range(angleC13 - 22, angleC13 - 38, -1), range(angleC4 - 57, angleC4 - 72, -1)):
            servowrite(servo7, a)
            servowrite(servo10, b)
            servowrite(servo4, c)
            servowrite(servo1, d)
            i = legangleL(a, angleC25, angleC26)
            j = legangleL(b, angleC32, angleC33)
            k = legangleR(c, angleC13, angleC14)
            l = legangleR(d, angleC4, angleC5)
            servowrite(servo8, i - 10)  # -10 raises the tail to stabilize the walking posture
            servowrite(servo11, j)
            servowrite(servo5, k + 10)  # +10 raises the tail to stabilize the walking posture
            servowrite(servo2, l)
            time.sleep_ms(10)
        stopcommand()


def runD():  # Walking action part four, see the tutorial "Posture Calculation Analysis" for a detailed explanation of the action, code explanation same as runA()
    if act == 1:
        for a, b, c, d in zip(range(angleC25 + 69, angleC25 + 22, -3), range(angleC32 