My First Self-Balancing Robot
This is a fun project to build your very own robot that balances on two wheels! It uses a Raspberry Pi Pico and is coded in MicroPython.

The code is written to be as simple as possible so you can see how everything works.

What You'll Need (Hardware) 
A Brain: Raspberry Pi Pico

A Sensor: MPU-6050 (this tells the robot if it's falling)

A Motor Driver: L298N (this gives power to the motors)

Motors & Wheels: 2 DC motors with wheels

A Body: A robot chassis to hold everything

A Battery: A 7.4V battery pack to power the motors

Wires: Lots of jumper wires to connect everything!

Putting It Together (Wiring) 🔌
Connecting the wires is the first big step. The most important thing is to make sure the GND from the battery, the Pico, and the motor driver are all connected.

Part	Wire goes to...	Pico Pin
MPU-6050	VCC	3.3V OUT (Pin 36)
GND	GND (Pin 38)
SCL	GP1 (Pin 2)
SDA	GP0 (Pin 1)
L298N Driver	12V / VIN	Battery (+)
GND	Battery (-) & Pico GND
ENA	GP15 (Pin 20)
IN1	GP13 (Pin 17)
IN2	GP12 (Pin 16)
ENB	GP10 (Pin 14)
IN3	GP9 (Pin 12)
IN4	GP8 (Pin 11)


Getting the Code Ready
Install MicroPython: Make sure your Pico has MicroPython on it.

Get Thonny: Thonny is the easiest program to use for writing code for the Pico. You can download it for free.

Copy & Paste: Open Thonny and connect to your Pico. Copy all the code from the main.py file.

Save to Pico: Paste the code into a new tab in Thonny and save it onto your Raspberry Pi Pico. Name the file main.py. This makes it run automatically when it gets power.

Running Your Robot! 
Place the robot on the floor.

Connect the battery to the motor driver.

Keep the Pico connected to your computer with the USB cable.

In Thonny, click the green "Run" button. You should see messages printing in the "Shell" area at the bottom.

Carefully try to stand the robot up. When it's almost straight, the wheels should start spinning to keep it balanced!

To stop it, just click the red "Stop" button in Thonny.

The Tricky Part: Tuning! 
Your robot probably won't balance perfectly at first. You need to "tune" it by changing the KP, KI, and KD values in the code. This takes patience!

Think of it like this:

KP is the POWER: How hard the robot tries to fix its balance.

KD is the BRAKES: Stops the robot from overshooting and wobbling.

KI is the FINE-TUNING: Fixes any small, constant leaning.

How to Tune:

Start with KI and KD set to 0.

Slowly increase KP. You'll see the robot start to wobble back and forth. Find a KP value where it wobbles pretty fast but doesn't fly out of control.

Now, slowly start increasing KD. This should calm down the wobbles. The robot should feel "stiffer".

If the robot always leans a little to one side, add a very, very small KI value (like 0.1) to fix it.
