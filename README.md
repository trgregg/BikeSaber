# BikeSaber
Arduino Feather and NeoPixel LED bike lights that synchronize across multiple bikes

Utilizes the Adafruit NeoPixel library to drive LED strips and RadioHead library to interface with the RFM69.

# Shopping list
## Wish lists
For convenience, the following wish-lists exist to quickly order the necessary parts. These overlap with the individual parts listed below.

Per-saber you will need the following:
- Part one from Adafruit: https://www.adafruit.com/wishlists/553180
- Part two from Amazon or other: https://www.amazon.com/hz/wishlist/ls/2VXE4I26LJS5J

Remaining supplies tend to come in packs of 5, 10, or more. These it will make sense to make one order for a group of people making sabers, or just to have extra supplies on hand for future:
- Bike Saber parts to buy in bulk: https://www.amazon.com/hz/wishlist/ls/2LZMEH2VG5T4U

## Tools
If you don't already have them, you'll need some tools and consumables to go with them.
- Soldering iron: https://www.amazon.com/gp/product/B07D35B75T/
- Heat gun (or paint stripper): https://www.amazon.com/Wagner-Spraytech-2417344-Included-Settings/dp/B08RHDWTW1/

## Controller
- Feather M0 RFM69HCW: https://www.adafruit.com/product/3176
- LIS3DH accelerometer: https://www.adafruit.com/product/2809
  - This is actually optional… but with the accelerometer we put the lights into a “sleep” mode when the bike hasn’t been moving for 5mins. This saves some battery, but leaves some sparkles and flags at the top/bottom so you can see your bike at night. If you leave this out, the code will automatically detect it is missing and disable the sleep feature.
- Antenna 900Mhz: https://www.adafruit.com/product/4269
- Resistors (optional but recommended)
  - 220ohm resistors to put inline with the data line. If you don’t have them, that’s fine.. But if the data line gets shorted, you may blow the output of the controller: https://www.adafruit.com/product/2780
- Connectors:
  - 3pin small (15mm) connectors: https://www.amazon.com/gp/product/B094ZJQCNB/
    - I put the male end (exposed pins) on the poles such that power isn’t naked when unplugged.
- Wire:
  - You can use the extra wire from the USB cables, but it is pretty small gauge. It is better to have ~24awg (or lower (larger)), 3 conductor cable. I had a bunch of audio snakes that I used… ie Mogami W2932 that I use.
  - https://www.amazon.com/22AWG-UL2464-Yellow-Conductors-2464-22AWG-3C/dp/B07TJF981M/
– optional – Breadboard (for experimenting): https://www.adafruit.com/product/64

# Light pole
- 8ft Tomato stake from your favorite gardening store
  - Either the thin 3/8in (0.4in) or the larger 3/4in (0.78in) versions work. The 3/8in isn’t as stiff, nor as durable, but it looks sleek. You’ll want at least 6ft length for ~90 LED strings, or you can make shorter poles with fewer LEDs if you like
  - https://www.amazon.com/NANOPRETTY-Diameter-Climbing-Gardening-Supplies/dp/B09MKKXN8Q/
  - https://www.lowes.com/pd/Panacea-Products-6-ft-Tall-Garden-Stake-Red
- optional — 1/2in electrical conduit
  - If using 3/4in tomato stakes and want a stronger Bike Saber, put a 2-3ft section of 1/2in conduit inside the bottom of the stake to help stiffen/reinforce it.
- Addressable LED strips: W2812B strips: https://www.amazon.com/gp/product/B088FJF9XD/
  - You’ll want 5meters (16.4ft), 60 LEDs/m (300 LEDs total), IP30, with adhesive tape on the back. No controller needed.… we are using our own Feather!
- Heat shrink: https://www.amazon.com/gp/product/B074HR2GF6/
  - Get the size to match your tomato stake:
    - For 3/8in stake, get 3/4in heat shrink.
    - For 3/4in stake, get 1” heat shrink
- Pipe clamps to go around poles
  - Again… get the size to match your tomato stake. Ones with rubber are best since the sizes are exactly right.
    - https://www.amazon.com/gp/product/B01HPE185E/
- Nuts and Bolts
  - M5 nuts and bolts (or similar) to connect between your bike-side clamp and the pole-side hose clamp. Best to use “locking” nuts so they don’t get loose on ya!
- Mounting clamps to go on your bike
  - This is the hard one… find a clamp that is approximately the same size as the rear frame tubes of your bike (by the rear wheel). They are normally ~1/2in in diameter, but can be smaller, or even some odd teardrop shape.
  - Items like these may fit bike with larger frame tubes, ie 0.75in: https://www.amazon.com/Nilight-90027B-Horizontal-Mounting-Warranty/dp/B07TNH72SG/
  - I made a custom order from Alibaba to get just the clamps from these bike racks. If you’re in San Francisco, reach out to me (Travis trgregg@mac.com) to trade a couple clamps for a beer.
    - https://www.alibaba.com/product-detail/Rear-Bike-Rack-Bicycle-Cargo-Adjustable_1600148193040.html
![image](https://github.com/trgregg/BikeSaber/assets/42472969/ad5617bc-d5a0-47ef-9aaa-afd0166a9121)

## Power
- Anker PowerCore 10000
  - Depending on how many LEDs you use, you may need 2 batteries wired in parallel to provide enough current. If you use ~180 LEDs (90 on each side of the pole) only 1 battery is need. If you add extra LEDs, you may need 2.
https://us.anker.com/products/a1263
- Cheap USB cables
  - We will cut off the micro end, and just use the USB-A connector for power
  - https://www.amazon.com/gp/product/B01GKWE3E0/

## Logistics
Saddle bag for your bike to hold the battery
- Schwinn Smart Phone Bag: https://www.amazon.com/Schwinn-SW77723-6-Smart-Phone-Black/dp/B00ILMLCJW
- Top Tube Phone Bag: https://www.schwinnbikes.com/products/top-tube-phone-bag?

Electrical tape and zip ties… you can’t make anything for Burning Man without zip ties!

# Controller Wiring Diagram
Yeah yeah… I’ll make a picture diagram someday soon™. It’s not complicated though:
- Controller power and connector
  - 5V and Ground from USB-A (normally red and black) goes 5V and Ground of Feather
    - Trim off the USB-A data lines. We don’t use them.
  - Parallel 5V and Ground to the 3-conductor wire, then to female connector (Red and Black)
  - Pin 5 from Feather to 220ohm resistor, then to Yellow data wire of the 3-conductor wire,  then to female connector
  - Add the antenna to the antenna hole
  - zip tie the USB and 3-conductor wire to the Feather through one of the holes in the board
- Accelerometer (optional)
  - 3.3V output of the Feather and Ground go to the LIS3DH matching pins
  - SDA and SCL pins of the Feather go to the LIS3DH matching pins (these are “serial data” and “serial clock”)
- Pole
  - Male connector Red and Black to 5V and Ground of LED strip
  - Male connector Yellow goes to “Din”, or “data”, or “DI” on the LED strip
  - Parallel all 3 to the other side of the pole

# Build It and They Will.… Shine?
You can make your poles as long or as short as you like. In my experience, too long leads to all sorts of headaches: takes too much power, too heavy causing the bike to be hard to ride, pulls the bike over when you park it, hits trees and other low-hanging things, can’t get in/out of doors, breaks when you crash.… etc. So these instructions are for what I found to be the happy median: 66inches.

1. Cut your tomato stake to 66in using a hacksaw or similar.
- If you are using a 3/4in stake and want to insert conduit, now’s the time!
  - Designate the cut side of the stake the bottom of your Bike Saber
  - Cut a 2ft section of 1/2in conduit
  - Slide/force the conduit into the tomato stake.
2. Cut two 1.5m (~55in) strips of LEDs, or 90 LEDs each
- The LED strips have solder joints every 50cm (30 LEDs), so you need 3 segments.
- When cutting the LED strips, cut in the middle of the copper solder pads so you can use both sides.
3. Tape the LED strips onto opposite sides of the pole starting at the top of the pole
- Be sure to orient the LED strips so the Data In is at the bottom. There will be arrows or similar markings on the strii.… make them point to the top of the pole.
- Use zip ties to help secure the strips if the adhesive isn’t holding well.
4. Unsolder any connectors that may have come with the LED strips. We won’t use them.
5. Solder the Male connector (naked pins) to one of the LED strips
- Red == 5V
- Black == Ground
- Yellow == Data
6. Cut a small (~2in) piece of the 3-conductor wire and pull out the individual conductors
7. Jumper between the LED strips using the individual 2” wires 
8. Secure the wires just under the bottom LED with a zip tie. 
- Optional: Put a zip tie just above the bottom LED to help hold the LED strips in place
9. Slide the heat shrink over the pole starting from the top and extend it 2-3in past the bottom LED, then cut it off at the top of the pole leaving ~1in overhang at the top of the pole.
10. Use your heat gun to shrink the heat shrink starting from the bottom of the pole.
- Rotate around the pole and be careful not to melt through the heat shrink
- At the top, heat the extra 1in section, then fold it over and squish it onto the pole. Note: It’s hot-ish, but just move quickly and/or wear gloves. This seals the top of the pole to keep dust/water out.
11. Place a hose clamp near the bottom of the LED strip, preferably on top of the heat shrink
12. Bolt one your bike-clamp.
13. Place a hose clamp near the bottom of the pole
- You may need to use a bit of heat shrink to the bottom of the pole if the hose clamp is too loose
14. Bolt one your bike-clamp.

Congrats! You have a Bike Saber!

# Source Code
You’ll need Arduino IDE, or equivalent: https://www.arduino.cc/
After installing, be sure to download the libraries and board definitions mentioned in the header of the project.

Project available at:
  https://github.com/trgregg/BikeSaber

Installing the software and flashing it on the controller is straightforward. If you need help, there are lots of great walkthroughs on the Arduino site, as well as Adafruit’s site

# Photos
![image](https://github.com/trgregg/BikeSaber/assets/42472969/32643526-f3ba-4b28-982c-da12e83c348d)

![image](https://github.com/trgregg/BikeSaber/assets/42472969/530a0102-722a-4f06-a71d-3b44e3e91e3a)

![image](https://github.com/trgregg/BikeSaber/assets/42472969/bb2a0362-4d83-432c-a555-13e69526d379)

![image](https://github.com/trgregg/BikeSaber/assets/42472969/76977fd3-0988-431a-885a-f6d1c682656a)

### Original Google Doc of this Readme
A rough "how to" for building the poles and boards:
https://docs.google.com/document/d/1k9FfZQvLQc-7zv1n-4YBiCZN0Z81B0xxiRZpR2TZl2g/edit?usp=sharing

