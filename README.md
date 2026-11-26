# BadmintonTurret

**Problem Statement**: To design a shuttlecock shooting robot which may be used to help amateur players without coaches to train cheaply.
<img width="386" height="221" alt="1112" src="https://github.com/user-attachments/assets/5bfd60d0-9e08-478d-ae09-36887d16b147" />

# Mechanical Design 
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
<img width="221" height="386" alt="image" src="https://github.com/user-attachments/assets/529acefa-ade5-4e5b-8646-d0216c8ebe1d" />
<img width="221" height="386" alt="image" src="https://github.com/user-attachments/assets/58386627-b814-4607-9b19-61a5c479d711" />


# Electronics:
- Microcontroller: Arduino Nano
- Communication to Laptop: Bluetooth Via HC05
- Buck Converter: XL4015 (for the servos)
- Camera: Realsense D435if RGBD Camera (connected to laptop via wire)
- ESC: Simonk30A
<img width="386" height="221" alt="image" src="https://github.com/user-attachments/assets/6f88fba2-1ae3-4b9f-a731-c00d9562256f" />


# Pipeline 
1. Arduino Nano covers the whole hardware stack sending the necessary PWM signals to the ESCs and the servo motors.
2. Camera feed is sent to the laptop via wire (USB 3.0) for the best performance.
3. Assumption: The bot is placed on the middle of the service line on its court and has no obstacle expected to show up for the next 2 meters until the net apppears.
4. As soon the shuttle is shot by the turret the depth camera detects the shuttle as it is the only moving object in the frame expected of its size.
5. The movement of the shuttle mapped in the depth frame is able to give us data regarding the initial speed and angle of the shuttle.
6. As soon as the shuttle exits the depth field we create a slightly larger ROI around the shuttle already detected in the depth field.
7. Compare a few frames in that ROI in order to be able to use **absdiff** to compare between the frames and find the moving shuttle.
8. Again update the ROI and reuse absdiff in order to find shuttle again.
9. This way we are able to get the shuttle's trajectory.
10. Then we detect the first bounce of the shuttle's trajectory and mark that point in pixels below the middle line of the camera frame.
11. using the focal length of the camera and some basic trigonometry we can easily and accurately estimate the distance of the shuttle from the turret.(Image attached)
12. Shuttle needs to be on the ground for us to find its distance, this method can not approximate distance for objects not in contact with ground.
13. **This way we have tracked and found the landing range of the shuttle without the use of any object detection or image segmentation model, hence reducing compute needed**
14. Problems and fixes:
    - Ceiling lights flash the camera at some spots, making shuttle tracking almost impossible then | Masked out the ceiling light and improvised ROI for finding shuttle after crossing the masked light.
    - Wheel jitter kills the shuttle tracking as it introduces too much noise in absdiff | Using april tags to compare between every other frame and stack every frame on each other perfectly for a stable camera feed for better shuttle tracking.
<img width="386" height="221" alt="image" src="https://github.com/user-attachments/assets/3a538478-84a8-4e3f-b4ca-a06cdcaa083a" />
<img width="386" height="221" alt="1111" src="https://github.com/user-attachments/assets/bea36109-11e4-4cef-bc48-16f3ca79576b" />

# Further Work 
- Collect data of intitial velocities and range of the shuttle to train a PINN in order to make the shuttle trajectory prediction robust and make the shooter fully autonomous.
- Work on Real to Sim analysis pipelines which are able to predict shuttle positions and asses the player while playing them
