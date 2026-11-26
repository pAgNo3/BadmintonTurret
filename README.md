# BadmintonTurret

**Problem Statement**: To design a shuttlecock shooting robot which may be used to help amateur players without coaches to train cheaply.

**Mechanical Design**: 
- The Turret was designed in Fusion360
- Materials used: PLA, ABS, TPU and rubber (Multiple different designs and materials were tested for the use of the wheels in the dual wheel mechanism to maximize the grip, controllability and health of the shuttlecock)
- Motors: MG995: Yaw and Pitch Motors | mg90s: Feeder actuation | 1400KV A2212 BLDC paired with Simonk30A esc: Dual wheel shooting mechanism
- The custom feeder mechanism is able to control the flow of shuttles 1 by 1 just by using a single mg90s micro servo motor.
- Jitter was a big problem provided by the high speed wheels which was tackled by:
  1. Decreasing the wheel diameter (increasing shuttle speed control)
  2. Using ABS for the wheels (higher Ductility -> No shattering/exploding like PLA | Better and more accurate printing than TPU)
  3. Using TPU to create the camera mount so that it's flexibility is able to compensate for immense vibrational jitter from the wheels.
- The shooter stands as 1.52m and the camera stands as 1.32m height.
- Current shuttle cock capacity: Tested with upto 5 shuttles.
