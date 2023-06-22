# CookCom
Relating to my DE2 Industrial Design Module at Imperial College London

CookCom is a network of wirelessly connected wristbands that help to enable non-verbal communication in busy industrial kitchens. 
The aim of the project was to make the hospitality industry a more accessible place, especially for individuals with hearing difficulties.

# Hardware
- TinyCircuits TinyZero Processor board
- TinyCircuits ProtoShield for easy soldering I/O
- TinyCircuits Bluetooth Low Energy Shield
- TinyCircuits 150 mAh Li-Po battery
- TTP223 capacitive touch sensor
- 2 sections of a Neopixel mini LED strip (Adafruit product 4368)
- Coin-style vibration motor
- 1x ZTX753 NPN transistor
- 1x 1kΩ resistor
- Bridging wire
- 3D printed casing

# Features
- Allows a chef wearing a CookCom wristband to sent alerts/messages to other CookCom wristbands in the vicinity
- Operates similar to a Morse code transmitter/receiver pair when transmitting/receiving. This is intended to be used for alerts and simple coded messages, but there is nothing stopping more advanced use-cases.
- Has a "send-to" menu that scrolls through colours to allow a sender to decide which wristband to send an alert to
- Has a "broadcast to all" feature that alerts all nearbly wristbands - range approx 15 m
- Wristbands can reply to or acknowledge messages they receive
- Can receive messages from a central base station - intended as a broadcaster app on a smart device, that hands out new food orders to the chefs in a kitchen
