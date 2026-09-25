# 3DOF CNC Surgical Robot: Project Notes and Research

##Braintstorming and Research

## 1. Project Summary

This project is a 3 degree of freedom CNC style robot with an orientable knife end effector, designed to make controlled cuts into a soft test material (playdough shaped like human tissue) by following a line drawn on the surface of the cutting subject, while maintaining a target cutting depth throughout the motion.

The system has two operating modes:

- **Manual mode**: a human operator drives the platform directly. X and Y motion come from a joystick, Z (plunge depth) is controlled with a button, and knife rotation is controlled with a knob.
- **Automatic mode**: the operator enters a target cutting depth on a 12 digit keypad with a small LCD readout, the system scans the drawn line on the subject, and the robot follows that line on its own while holding the requested depth.

The mechanical actuation is planned around stepper motors for X, Y, and Z, since at the low speeds this application needs, open loop steppers do not strictly require encoder feedback, although closed loop steppers are preferred where available. A continuous rotation servo is planned for the knife orientation axis. Depth is measured with a sensor fusion of a pressure sensor mounted behind the knife and a distance/depth sensor, both riding on the moving platform with the knife. Line detection and following is meant to run mostly on a Raspberry Pi 5 (8GB), falling back to a PC if the Pi cannot keep up, using computer vision with a lightweight AI model.

Two candidate strategies for line following were identified:

1. **Live scan and follow**: the camera is mounted on the moving platform. At the start of the operation it does a quick scan to locate the line, then the platform follows it, either through direct motor control or by generating motion commands in real time based on the relationship between the home position (absolute coordinates) and the start of the line (relative coordinates). This allows continuous self correction, at the cost of a slower start while the initial scan runs, which could be bridged with manual mode.
2. **Scan once, generate the full path, then execute**: the camera is fixed in a position where it can see the entire board before the platform leaves home. The system identifies the relationship between absolute and relative coordinates once, generates the full path up front, and the robot executes it. The drawback is that once the platform starts moving it blocks the camera's view of the workpiece, so there is no chance to correct mid cut.

The project intentionally spans high level embedded software, low level embedded/motion control, and computer vision, and may also use ROS2 and MATLAB.

The rest of this document works through each subsystem, checks the assumptions above against how similar problems are solved elsewhere, and lists concrete parts, tools, and references.

---

## 2. Actuation (X, Y, Z, and knife orientation)

### Linear axes (X, Y, Z)

Open loop steppers are a reasonable default at low speed, and closed loop stepper motors are also a real, off the shelf product category rather than something that needs to be custom built. A closed loop stepper pairs a normal 2 phase stepper with an integrated rotary encoder and a matching digital driver: the encoder reports actual shaft position back to the driver in real time, and the driver corrects for any missed steps instead of silently drifting. Kits built this way are sold specifically for CNC and 3D printer upgrades, with NEMA23 motors, 1000 line encoders (4000 counts per revolution), and drop in compatibility with existing NEMA23 mounts [1][2][3].

Recommendation: since the plan already prefers encoder feedback when available, it is worth just building with closed loop steppers from the start on at least the Z axis (the one directly responsible for cutting depth), rather than retrofitting later. It avoids a second mechanical redesign of the shaft coupling.

### Knife orientation axis

A continuous rotation (360 degree) servo only gives speed control, not angle feedback, which is fine for manual mode where a human is watching the knife and adjusting by eye. In automatic mode, though, the knife likely needs to be held at a specific angle relative to the direction of travel so it stays tangent to the curve being cut. That needs true position control, not just spin control. A standard positional servo, or a small closed loop stepper on that axis, is a better fit for the automatic mode requirement than a continuous rotation servo.

---

## 3. Depth Sensing (pressure and distance fusion)

The idea of fusing a contact pressure reading with a distance/depth reading to control cutting depth is not a guess, it matches how force sensing is approached in soft tissue robotic cutting research. One relevant finding: the depth of cut plays a significant role in the magnitude of the cutting force acting on the blade, and image processing has been used to outline the tissue surface and estimate the time variation of the depth of cut, which is then combined with force measurements to get a normalized cutting force for analysis [4]. That is effectively the same sensor fusion idea, applied to real soft tissue cutting experiments.

### Pressure sensor

A Force Sensitive Resistor (FSR) style force to voltage transducer mounted at the tool tip is a documented approach in surgical robotics: an FSR array with a preload mechanical structure gives a responsive measurement of contact force along the tool's normal direction [5]. For a first prototype, a single axis FSR or small load cell aligned with the blade's normal direction is enough, a full 3 axis load cell is not necessary to get useful depth/force feedback.

### Distance/depth sensor (budget friendly option)

A full stereo depth camera like the Intel RealSense D405 is a genuinely good match for close range, sub millimeter accuracy work (its ideal range is 7 to 50 cm, and Intel specifically lists wound measurement as a target use case) [6][7], but it costs around 300 USD, which does not fit a student budget.

A much cheaper alternative that still solves the actual problem (measuring the height of the surface directly under the knife) is a small Time of Flight (ToF) distance sensor:

- **STMicroelectronics VL53L5CX**: an 8x8 multizone ToF ranging sensor, range roughly 2 cm to 4 m, 1 mm depth resolution, up to 60 Hz update rate, communicates over I2C [8][9]. Breakout boards with a voltage regulator and level shifting (so they work with both 3.3V and 5V logic, meaning they work fine with either an Arduino or a Pi) are sold by several vendors for about 15 to 20 USD [10][11].
- Simpler single zone alternatives in the same family (VL53L1X, VL53L0X) are even cheaper (roughly 10 to 15 USD) if an 8x8 grid of readings is more than the project needs and a single distance reading under the knife tip is enough [12].

This is not a depth camera in the imaging sense (no RGB, no full image), but for the specific job of reading "how far is the surface below the knife right now," it is a far more budget friendly, still credible sensor, and it is easy to wire directly to an Arduino for the real time depth control loop.

---

## 4. Line Detection and Following

### Rethinking "light AI model" for this specific scene

The plan to use CV with a lightweight AI model for line detection is reasonable in general, but worth reconsidering for this particular scene. Lightweight neural segmentation models for edge devices exist and do run on hardware like a Raspberry Pi [13][14][15], but they are built for messy, uncontrolled scenes (road lanes, general driving scenes) where the appearance of the thing being detected varies a lot.

This project's scene is much simpler: a single, high contrast line drawn on a plain colored surface, under lighting the project fully controls. That is a classical computer vision problem, not a machine learning problem, and classical CV should outperform a neural network here on every axis that matters: latency, determinism, and ease of debugging, while running comfortably within a Pi 5's CPU budget with room to spare.

Suggested pipeline: HSV or LAB color thresholding to isolate the line's color, morphological thinning/skeletonization to reduce the line to a single pixel wide centerline, then contour tracing or spline fitting to produce an ordered list of path points. This should run well under 30 ms per frame on a Pi 5.

Keeping a lightweight AI model in mind is still reasonable as a stretch goal (for example, to make the system tolerant of a messier background or inconsistent line contrast later), but it should not be the first thing built.

### The two path strategies already have names

Both candidate approaches described in the project summary correspond to established, published control strategies in visual robotics, which is useful both for confidence and for report writing:

- **Option 1 (scan then follow live, with self correction)** matches **image based visual servoing**, specifically a **Path Following Controller (PFC)**. In a PFC, the target at each control step is set to the closest point on the path to the robot's current state, rather than to a fixed timed reference. This makes the controller more robust to delays and physical disturbances, and produces a smooth approach that meets the path tangentially [16][17].
- **Option 2 (scan once, generate the full path, then execute)** matches a **Trajectory Tracking Controller (TTC)**. A TTC uses a timed reference, so if something disturbs the robot mid motion, it can fall behind that reference with no way to recover, since the camera's view is blocked once the platform starts moving [17].

Given the workpiece is a deformable material that could shift slightly under blade pressure, Option 1 (the PFC / visual servoing approach) is the safer choice despite its slower startup, since the self correction is closer to a safety requirement than a nice to have here. The slow initial scan can be shortened with a fast, coarse, low resolution first pass just to find the rough start of the line, then switching to a tighter, higher precision tracking window once the blade is close, so the system is not paying the full "search the whole board" cost during the actual cut.

---

## 5. Control Architecture: Arduino, Raspberry Pi 5, and ROS2

### Splitting real time motion from perception

A Raspberry Pi 5 running a general purpose OS is not a real time platform. Generating precise stepper step pulses directly from a Python process on the Pi risks jitter and missed steps. The standard, proven pattern for this exact problem is to let a microcontroller handle the real time step generation while a higher level computer just sends target positions or path commands over serial.

Arduino is actually a very natural fit for this half of the system, because GRBL, the de facto standard open source CNC motion controller, was originally written specifically for the Arduino/AVR328 platform. GRBL parses G-code, plans acceleration and velocity profiles with look ahead across many moves in advance, and can maintain over 30 kHz of stable, jitter free step pulses on a plain Arduino Uno class board [18][19]. Variable spindle/servo PWM output is also supported in common GRBL forks, which covers the knife rotation axis's positional control as well [20].

Practical split:

- **Arduino**: runs GRBL or a GRBL derived firmware. Handles real time step/direction generation for X, Y, Z, reads the closed loop stepper encoders, reads the ToF depth sensor and the pressure sensor for the depth control loop, and drives the knife's positional servo.
- **Raspberry Pi 5**: runs the CV pipeline (line detection), the path following logic (the PFC described above), and sends target coordinates or G-code style motion commands to the Arduino over serial, the same way you already built a custom UART protocol for STM32 firmware at Camerabotics.

### Where ROS2 fits

Given the actual size of this system (two real compute nodes exchanging fairly simple messages), ROS2 is not strictly necessary from a systems engineering standpoint, plain serial framing between the Pi and the Arduino solves the problem with far less overhead. That said, since the goal here also includes demonstrating ROS2 experience, it can be included without compromising the core control loop, by keeping it out of the time critical path:

- Run the ROS2 nodes (the vision/line detection node, the path following node, and a serial bridge node) entirely on the Raspberry Pi 5.
- Keep the Arduino as a plain serial peripheral that only speaks a simple command protocol (or G-code); it does not need to run ROS2 itself.
- This is a well established integration pattern: rosserial (ROS1) and its ROS2 successor, micro-ROS, exist to connect microcontrollers into a ROS graph [21][22]. One caveat worth knowing before committing: official micro-ROS support targets boards with more RAM/flash than a classic Arduino Uno/Mega (ATmega328), such as ESP32, Teensy, or the Arduino Portenta/Nano RP2040 Connect [23][24]. A plain Arduino Uno or Mega running GRBL will not run micro-ROS directly, it would need to stay a plain serial device with a small ROS2 bridge node on the Pi translating between the two, which is exactly the split described above.
- If literal micro-ROS running on the microcontroller itself becomes a specific requirement later (rather than just "used ROS2 somewhere in the stack"), swapping the Arduino for an ESP32 is a low cost, well documented path, since ESP32 has official micro-ROS Arduino core support [23].

This gives a genuine, correctly used ROS2 layer for the portfolio/demonstration goal, without putting ROS2's discovery and message passing overhead on the real time motion path.

---

## 6. Manual and Automatic Mode UI

The 12 digit keypad plus small LCD for entering the target cutting depth is a simple, well proven interface pattern, a matrix keypad plus a character LCD (an I2C backpack is worth using to save GPIO pins) both read directly by the same Arduino that runs the motion control loop, keeping depth entry, depth sensing, and depth actuation all on one time consistent controller.

---

## 7. Suggested Tool and Part Summary

| Subsystem                | Suggested tool/part                                           | Notes                                                                                                                                |
| ------------------------ | ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| Motion firmware          | Arduino (Uno/Mega class) + GRBL or GRBL derived firmware      | Proven for jitter free step generation, native G-code support                                                                        |
| Linear axes              | Closed loop NEMA23 stepper + encoder + matching hybrid driver | Off the shelf kits, drop in NEMA23 mounting                                                                                          |
| Knife axis               | Positional servo (or small closed loop stepper)               | Continuous rotation servo cannot hold a fixed angle for automatic mode                                                               |
| Depth/distance sensing   | VL53L5CX (or VL53L1X for single point) ToF breakout           | Budget friendly (roughly 15 to 20 USD), I2C, works with Arduino or Pi                                                                |
| Force sensing            | Single axis FSR or small load cell at the knife tip           | Matches surgical robotics force sensing approach                                                                                     |
| Vision/line detection    | Python + OpenCV on Raspberry Pi 5                             | Classical thresholding + skeletonization first; lightweight AI model as a later stretch goal                                         |
| Path following logic     | Image based visual servoing (Path Following Controller)       | Better disturbance handling than a pre generated trajectory                                                                          |
| High level orchestration | ROS2 (on the Pi 5 only)                                       | Vision node, path planning node, serial bridge node to the Arduino                                                                   |
| Communication            | UART/serial, simple custom protocol or G-code                 | Same approach as your Camerabotics UART protocol work                                                                                |
| Offline analysis         | MATLAB                                                        | Useful for PID tuning simulation for the depth/force loop and control system analysis for the report, not part of the running system |

---

## 8. Depth/Distance Sensor Alternatives (if a ToF module cannot be sourced)

The VL53L5CX/VL53L1X ToF sensors recommended in Section 3 are the best fit for this job (accurate, digital, close range), but they are not the only option. If none can be sourced locally or through the usual online suppliers, here are workable alternatives, roughly ordered from closest replacement to last resort.

### Option A: Short range analog IR distance sensor

Sharp/Socle style analog infrared distance sensors are a mature, widely stocked alternative that has been used in hobby robotics for years. The **GP2Y0A51SK0F** variant measures 2 to 15 cm, which matches the close range needed under the knife platform well, and it outputs a simple analog voltage, so it just needs one analog input pin on the Arduino, no I2C, no library dependency [25]. Longer range siblings (GP2Y0A21YK0F: 10 to 80 cm, GP2Y0A41SK0F: 4 to 30 cm) exist if the mounting geometry needs more standoff distance [26][27]. Downsides versus the ToF option: single point only (no 8x8 grid), and the output curve is nonlinear, so it needs a calibration lookup table or curve fit rather than a simple linear scale.

### Option B: DIY stereo camera pair (uses the Pi 5 you already have)

The Raspberry Pi 5 has two CSI camera ports, which makes a stereo depth setup practical without any dedicated depth sensor chip at all. Two identical camera modules (for example two IMX219 based Pi Camera Module v2 units) mounted rigidly at a fixed baseline (60 to 150 mm apart) let OpenCV compute a disparity map and convert it to real depth using the standard stereo triangulation formula, depth = (focal length in pixels x baseline) / disparity [28][29]. This reuses hardware and vision skills already planned for the line detection pipeline, and two commodity camera modules typically cost less in total than one ToF breakout board. Downsides: needs a one time calibration procedure (checkerboard calibration, stereo rectification), and dense disparity maps are noisier and slower to compute than a purpose built ToF sensor, so it is a heavier software lift than Option A.

### Option C: Mechanical contact probe (touch probe style)

CNC machines commonly solve "how high is the surface" with a simple touch probe: the tool is lowered until it makes contact, and the point of contact is read as the surface height [30][31][32]. The classic version relies on electrical continuity between a conductive tool and a conductive plate, which will not work here since playdough is not conductive. A spring loaded microswitch version solves that: the probe tip rests on the material, and a small microswitch triggers at a repeatable amount of deflection, giving a discrete "surface found here" signal rather than a continuous reading [33]. This is very cheap and mechanically simple, but it only gives a height reading at the instant of contact, so it suits periodic depth checks better than continuous real time tracking during a moving cut.

### Option D: Ultrasonic distance sensor (last resort)

The HC-SR04 is the cheapest, most universally available distance sensor, with a rated range of 2 to 400 cm and a 15 degree beam angle [34][35]. It is workable as a coarse fallback, but two things matter for this project: its rated 2 cm minimum range is unreliable in practice, since users report erratic or reversed readings within the first few centimeters of that range [36], and its 15 degree beam is wide relative to the small area under a knife tip, so it tends to average over a bigger patch of surface than a ToF or IR sensor would. Treat this as a "better than nothing" option rather than a primary recommendation for this application.

### Option E: Force only depth regulation (no separate distance sensor)

It is also possible to skip a dedicated distance sensor entirely and regulate cutting depth purely through the force/pressure sensor already planned, by controlling the position of the tool tip through closed loop force control instead of through a measured distance. This is a documented, working approach in soft tissue robotics: a handheld surgical robot controlled contact force by controlling the depth of the tool tip via closed loop force control alone, without a separate depth sensor, achieving contact force tracking with an RMS error under 8 mN [37]. For this project, that would mean using a compliant or spring loaded Z stage and letting the FSR/load cell reading directly set the plunge depth through a control loop, rather than fusing it with an independent distance measurement. This simplifies the bill of materials at the cost of losing the independent depth cross check the original sensor fusion idea was built around.

### Quick comparison

| Option                             | Approx. cost              | Precision                        | Continuous or discrete  | Extra complexity                                      |
| ---------------------------------- | ------------------------- | -------------------------------- | ----------------------- | ----------------------------------------------------- |
| ToF (VL53L5CX/L1X, Section 3)      | ~15 to 20 USD             | High (mm level)                  | Continuous              | Low (I2C library)                                     |
| A: Analog IR (GP2Y0A51SK0F)        | ~10 to 15 USD             | Medium                           | Continuous              | Low, needs calibration curve                          |
| B: DIY stereo camera pair          | ~40 to 60 USD (2 cameras) | Medium, scene dependent          | Continuous              | Medium/high (calibration + disparity pipeline)        |
| C: Microswitch contact probe       | ~5 to 10 USD              | High at contact point            | Discrete (touch events) | Low, but mechanical design needed                     |
| D: HC-SR04 ultrasonic              | ~2 to 5 USD               | Low at close range               | Continuous              | Low, but noisy near minimum range                     |
| E: Force only (no distance sensor) | 0 USD extra               | Depends on mechanical compliance | Continuous              | Medium (control loop design, needs compliant Z stage) |

---

## References

[1] Closed Loop Stepper Motor with Encoder, NEMA23, Makersupplies.dk: https://makersupplies.dk/electrical-cnc-electronics/stepper-drivers-motors/closed-loop-stepper/stepper-drivers-motors-closed-loop-stepper-nema-23-closed-loop-stepper-motor-w-encoder-2nm-8mm-shaft

[2] NEMA 23 Closed Loop Stepper Motor Kit with 5080D4 Driver, 1000 Line Encoder, BuildYourCNC: https://buildyourcnc.com/products/nema-23-closed-loop-stepper-motor-kit-with-5080d4-driver-439-oz-in-torque-1-4-shaft-1000-line-encoder-and-3m-cable

[3] NEMA 23 Integrated Closed Loop Stepper Motor, Makersupplies.dk: https://makersupplies.dk/electrical-cnc-electronics/stepper-drivers-motors/closed-loop-stepper/stepper-drivers-motors-closed-loop-stepper-nema-23-integrated-closed-loop-stepper-motor-8mm-shaft

[4] Miniaturized force-indentation depth sensing and soft tissue cutting force analysis (Valdastri/Dario group): https://cdn.vanderbilt.edu/t2-my/my-prd/wp-content/uploads/sites/823/2013/01/MST_Valdastri_Dario.pdf

[5] A surgical system for automatic registration, stiffness mapping and dynamic image overlay (FSR based force sensor design): https://arxiv.org/pdf/1711.08828

[6] Intel RealSense D405 product page: https://www.intelrealsense.com/depth-camera-d405/

[7] Intel RealSense D405 technical specifications, Intel Store: https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d405.html

[8] VL53L5CX Time-of-Flight multizone ranging sensor, STMicroelectronics via Farnell: https://dk.farnell.com/stmicroelectronics/vl53l5cxv0gc-1/time-of-flight-sensor-4m-3-6v/dp/3772978

[9] VL53L5CX product overview, STMicroelectronics: https://st.com/en/product/vl53l5cx-satel.html

[10] VL53L5CX Time-of-Flight 8x8-Zone Distance Sensor Carrier, Pololu via The Pi Hut: https://thepihut.com/products/vl53l5cx-time-of-flight-8x8-zone-distance-sensor-carrier-with-voltage-regulator

[11] VL53L5CX 8x8 Time of Flight Array Sensor Breakout, Pimoroni via The Pi Hut: https://thepihut.com/collections/latest-raspberry-pi-products/products/vl53l5cx-8x8-time-of-flight-tof-array-sensor-breakout

[12] Adafruit VL53L1X Time of Flight Distance Sensor, STEMMA QT/Qwiic: https://thepihut.com/collections/adafruit-sensors/products/adafruit-vl53l1x-time-of-flight-distance-sensor-30-to-4000mm-stemma-qt-qwiic

[13] Image segmentation based robot path planning project (Raspberry Pi 4): https://github.com/gpsub/Image_segmentation_based_robot_path_planning

[14] PIDNet-LW: lightweight semantic segmentation for edge devices, Journal of Real-Time Image Processing: https://doi.org/10.1007/s11554-025-01799-4

[15] A Low-Rank CNN Architecture for Real-Time Semantic Segmentation, tested on Raspberry Pi 4: https://doi.org/10.1109/OJCAS.2022.3174632

[16] A Visual Servoing Based Path Following Controller, Dewis and Jagersand, University of Alberta: https://webdocs.cs.ualberta.ca/~vis/site_pages/dewis_vs_pfc.html

[17] Image Space Path Following Control Using Visual Servoing (full paper PDF): https://webdocs.cs.ualberta.ca/~vis/site_pages/pdfs/dewis_vs_pfc_crv.pdf

[18] GRBL: embedded G-code interpreter and motion controller for Arduino/AVR328: https://github.com/ajmendez/grbl

[19] GRBL v1.1 wiring and connection guide: https://github.com/gnea/grbl/wiki/Connecting-Grbl

[20] GRBL with servo/variable PWM support: https://github.com/cprezzi/grbl-servo

[21] ROS2 and Arduino serial communication overview (micro-ROS explanation): https://github.com/anasderkaoui/ROS2-and-Arduino-serial-communication

[22] micro-ROS overview and supported external tools/frameworks: https://micro.ros.org/docs/overview/ext_tools

[23] micro-ROS for Arduino, supported boards list (Portenta H7, ESP32, Teensy, Nano RP2040 Connect): https://github.com/adityakamath/micro_ros_arduino

[24] micro-ROS brings ROS2 to the Arduino IDE, initial supported boards: https://www.hackster.io/news/micro-ros-project-brings-ros-2-to-the-arduino-ide-and-cli-through-an-experimental-library-release-656a72fff2fa

[25] Sharp GP2Y0A51SK0F Analog Distance Sensor, 2 to 15 cm, Pololu via RobotShop: https://www.robotshop.com/products/sharp-gp2y0a51sk0f-analog-distance-sensor-2cm-15cm

[26] Sharp GP2Y0A21YK0F Analog Distance Sensor, 10 to 80 cm, specifications: https://www.embeddedrelated.com/parts/p/GP2Y0A21YK0F

[27] Sharp GP2Y0A41SK0F Infrared distance sensor, 4 to 30 cm: https://littlebirdelectronics.com.au/products/sharp-infrared-distance-sensor-with-line-gp2y0a41sk0f-4-30cm

[28] Build a stereo depth camera from two Raspberry Pi camera modules, calibration and metric depth pipeline: https://github.com/base698/stereo-csi

[29] Stereo Camera Setup for Depth Estimation with OpenCV, calibration and disparity to depth conversion guide: https://zbotic.in/stereo-camera-setup-for-depth-estimation-opencv-guide/

[30] Z Probe Touch Sensor for setting CNC Z height (electrical continuity type): https://m.reach.dog/shop/byte-2-bot/products/z-probe-touch-sensor

[31] DIY CNC touch probe design discussion, LinuxCNC forum: https://www.forum.linuxcnc.org/10-advanced-configuration/22334-tool-setter

[32] Custom XYZ touch probe with automatic tool Z height setting, community build log: https://forum.onefinitycnc.com/t/custom-xyz-touch-probe-with-automatic-tool-z-height-setting/10425/1

[33] CNC Z-probe design using a microswitch instead of electrical continuity: https://thangs.com/m/29236

[34] HC-SR04 Ultrasonic Distance Sensor specifications (2 to 400 cm range, 15 degree beam angle): https://www.sparkfun.com/products/24049

[35] HC-SR04 datasheet summary and specifications, EDN: https://edn.com/hc-sr04-datasheet/

[36] HC-SR04 unreliable readings near minimum range, user reports and discussion, Arduino Forum: https://forum.arduino.cc/t/hc-sr04-bad-reading-at-close-range/646988

[37] Contact Force Control During Soft Tissue Interaction Using a Handheld Robot, depth of tool tip controlled via closed loop force control: https://pubs.kist.re.kr/handle/201004/77154
