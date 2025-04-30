# monorepo
This repo contains all HSP Poseidon V1 Projects. Follow the ## Starting up to just jump right in. 
Each board will be described below, as well as important things to help understand it.


## Starting up
1. Clone this repo and cd into it
2. Checkout true_monorepo
3. Open STM32CubeIDE and select this monorepo folder as the workspace
4. Open the project you want to work on in STM32CubeIDE.
5. Double click the .launch file in the root of the project to open it (no need to do anything with it, just open it so STM32 knows about it)
6. Click the debug button in STM32CubeIDE.
7. Everything should work! If it doesn't go to the README section with name of your board youre working on (ex. Servo Board Controller) or if you have problems even seeing the projects in STM32CubeIDE at all, go to ## Troubleshooting


# Fundamental design
We use the Control Access Network (CAN) protocol to talk between all physically connected boards like the pad controller and the servo boards. For boards that are far away, like the control box and pad controller, we use XBee Pro X3B radios which communicates to the boards over UART. With this, we can send and recieve messages, however we need to ensure that only the right board does the action it is supposed to, while all having the same code for ease of generation. Enter IDs. The radios are simple for this, as they can be configured using XCTU to only listen to a certain radio. For CAN, it is a bit more complicated. Because CAN is a bus, and we want to be able to have more important messages go through first we utilize the arbitration based on the CAN Protocol. The STM32s have a built in 96 bit ID, but the extended CAN_ID can only hold 29 bits. Therefore, in components.h of coreutils, we define the mapping between all of those. More specifically, we are able to define a mapping between the STM32 ID and the precise component that we wish to talk to. The way to build the CAN_EXT_ID is described below. We also create a filter based on the 8 bit short_board_id (second byte of CAN_EXT_ID), so that each board only recieve messages if it was intended. For the config.h file of coreutils, we go through and create enums for each of the options that we currently have available to talk to each of the boards.

## Explaining the IDs
Board_ID is a 96 bit ID based on the STM32 CPU
CAN_EXT_ID is a 29 bit ID that is used for can communication
Short_Board_ID is an 8 bit ID, which is the second byte of the CAN_EXT_ID

The extended CAN ID uses 29 bits (compared to standard 11-bit IDs)
The format is: [8 bits sender][8 bits board ID][8 bits message type][5 bits instance][3 bits set to zero, reserved for internal CAN usage] **ADD LINK TO EXT CAN PAGE, PAGE 1096 OR SOMETHING**

For arbitration (which message gets priority on the bus), lower CAN ID values have higher priority
This means senders with lower IDs will always have priority over senders with higher IDs

 CAN_ID is acquired based on a scheme.
 Sender field (8 bits - first part of ID, highest priority impact):
 - 0: Break wire system (highest priority)
 - 1: Pad controller
 - 2: Servo board
 - 3: Sensor board
 - 4: Tester board (this code and controller)
 - 254: Tester board (hardware)
 - 255: PC/Terminal (lowest priority)
 - 5-253: Reserved for future devices

 Board ID field (8 bits - identifies specific physical boards):
 - 0-255: Unique identifier for each physical board in the system
 - This allows up to 256 individual boards on the network

 Component type field (8 bits - defines what type of component is being addressed):
 - 0: System/board control
 - 1: Servo
 - 2: Thermocouple
 - 3: Pressure transducer
 - 4: Heater
 - 5: LED
 - 6: Flash Status
 - 8-255: Reserved for future component types

 Instance field (5 bits - identifies specific component instance):
 - 0-31: Allows up to 32 instances of each component type per board
 - For example, a board could have up to 32 servos, 32 thermocouples, etc.

 Example of a full extended ID:
 0x01050401 (hex) = 00000001 00000101 00000100 00001 000 (binary)
                     |        |        |        |     |
                     |        |        |        |		+-- Padding
					   |		|		 |		  +-------- Instance 1
                     |        |        +----------------- Component Type 4 (Heater)
                     |        +-------------------------- Board ID 5
                     +----------------------------------- Sender 1 (Pad Controller)

 This example ID represents: "Pad Controller (1) sending a message to Board 5,
 addressing Heater (4), instance 1"



# Code and hardware explanation

## coreutils
Code is at monorepo/Firmware/coreutils

### How it works
This folder is not a project itself, but rather a collection or core utility (hence coreutils) functions symlinked into all of the other projects. This enables us to not have to rewrite the code for every project if we want a small change in functionality across all of them.

### Software Explanation
Coreutils is broken into two sections, the configs, and the utilities.

#### Config
components.h contains the lookup tables for each component, mapping a 96 bit STM32 MCU ID to a CAN ID, and the properties of each of those. It also provides a way to get the configuration struct for each component with multiple methods. These structs are defined in other files.
config_utils.h contains nothing
config.h contains enums for each of the message types to help construct the CAN_EXT_IDs, as well as utilities for converting between the different types of IDs.
flight_config.h contains nothing
heater_config.h contains the commands for the heater, the struct for Heater_Config, and the states that are used in the heater state machine
pad_config.h contains nothing
sensor_board_config.h contains nothing
servo_config.h contains the commands for the servo, the struct for Servo_Config, and the states that are used in the servo state machine 
thermo_config.h contains the commands for the thermocouple, the struct for Thermo_Config, and the states that are used in the thermocouple state machine

#### Utils
The board_utils contains many things to flash lights, and to get the STM32 manufacturer chip ID.
The can_utils contains utilities for CAN, so that it is easy to build a CAN_EXT_ID as well as decoding them.
The radio_utils contains the buffers for the radio inputs, as well as the code to update the acknowledgement.


## Servo Board Controller
Code is at monorepo/Firmware/ServoControllerFirmware

### Hardware Explanation
There are four microfit molex connectors that can go into this servo board. Two of those are for CAN to enable daily chaining, and to provide 6v power to the board. One of the other ones connects to a servo, with the other connecting to a heating pad and thermocouple. More detail can be seen in the schematic.

### Software Explanatino
In the code, there are multiple classes, and multiple state machines for each of the main components. Within the Inc folder of ServoControllerFirmware, there are three state machines, for the servo, the heater, and the thermocouple. The state diagrams for all three are shown.
**TODO: ARE POSTED HERE**

CAN works differently here. Instead of a state machine, the CAN pins on the STM32 microcontroller interface with the CAN transciever, which is physical hardware on the board. This triggers an interrupt in the STM32 causing it to instantly stop executing any current code and jump to HAL_CAN_RxFifo0MsgPendingCallback, where it reads the message, and updates the commands for the state machines in the main while loop. A visualization is shown below.
**TODO: SHOW VISUALIZATION BELOW**

### Startup
For this board to work, it must be powered by 6v and connected with the JLink Edu Mini. 
1. Connect the JLink Edu Mini to the 10 exposed pads on the Servo Board.
2. Connect a USB to the JLink Edu Mini.
3. Connect 6v and GND to a stable power source. 
4. Click debug or run in STM32CubeIDE.

### Schematic
https://highalnder-space-program.365.altium.com/designs/DD1C7E97-9DE2-47FC-8D92-E38E062CB5E9?variant=[No+Variations]&activeView=SCH#design


## Sensor Board Controller
Code is at monorepo/Firmware/SensorControllerFirmware

IDK how it works, i dont think it actually does rn, just some testing code.

## Pad Controller
There are two versions of this code. One is for the PCB, one is for the nucleo and perfboard version. This will talk about the PCB.
Code is at monorepo/Firmware/PadController

### Hardware Explanation
The current hardware includes flash memory, meant for rapid storage and updating of the configurations of the servo boards, as well as storing the data from the sensor boards. There is a 6V-out buck converter, allowing it to step down the battery voltage, a can transciever, and many broken out GPIO pins for things like ignitors.

### Software Explanatino
The code functionality all is broken down into getting commands from the radio, and then doing things with those commands, and some initialization. First, it runs through each board and tries to force their light to flash. If it doesnt, there is likely a faulty connection or the code on the boards is not up to date. Similar to the servo board, it gets messages via CAN but also UART this time for the radio, and it stores those messages in variables to be used in the main while loop. The main while loop currently contains about half of the useable commands, and it is notably missing some functionality. The only functionality that works is the activate servo, and turning on three of the servos (as we only use three now). Additional functionality should be added based on the code in StaticFireFirmware303. 
Required Functionality for minimum viable product:
Ignitor
Auto
Pyro Valves

### Startup
For this board to work, it must be powered by 6v and connected with the JLink Edu Mini. 
1. Connect the JLink Edu Mini to the 10 exposed pads on the Servo Board.
2. Connect a USB to the JLink Edu Mini.
3. Connect 6v and GND to a stable power source. 
4. Click debug or run in STM32CubeIDE.

### Schematic
https://highalnder-space-program.365.altium.com/designs/AFDF2050-002B-480D-BE46-3BDA07798084#design



## Control Panel
This code is not in monorepo, as it currently uses arduino ide still. The link to the repo is here: https://github.com/Highlander-Space-Program/control-panel-firmware

<!-- IDK plz halp alex -->
## Canalyzer Terminal
#### Connecting everyhting
1. Connect 1 USB cable to the F303K8 nucleo on the breadboard. (See F303_HITL for source code)
2. Connect the JLink Edu Mini to the Servo Board.
3. Connect the Male Molex connector from the breadboard to the port labeled CAN A or CAN B on the board you want to send a CAN message to.
4. Connect a USB to the JLink Edu Mini.
5. Connect 6v and GND to a stable power source.

#### Running the code
Currently, the F303K8 uses 115200 baud rate. Therefore, in the canalyzer_terminal code, use that baud
6. Run `python3 canalyzer_terminal.py` (if it says its missing a package, use `python3 -m pip install <package_name>` and try again)
7. Type `l` then hit enter. It should show ST-Link as one of the options. If not, things are not connected properly.
8. Type `t` then hit enter. Type the number of the port that has the ST-Link and hit enter. Type `115200` for baud rate and hit enter. 
9. To test if the F303K8 gets your messages, type `c` then hit enter and then type `t` and hit enter. it should turn off the F303K8 light.
10. To test if the other board is receiving messages, type `c` then hit enter, then type `l` and hit enter, then type the number of the board based on the CAN_ID and hit enter.

## Troubleshooting
#### Can't find coreutils or config stuff when trying to build?
1. Is your workspace monorepo? If not, this may have issues. If if is and still not working, go to step 2.
2. Right click the project name in the Project Explorer, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils

#### You can't see coreutils in Inc
1. Navigate to the Inc folder in Project Explorer and right click it, then click New > Folder. Click Advanced > Link to alternate location (Linked Folder) and put `WORKSPACE_LOC/Firmware/coreutils` in the input box. Click Finish, and everything should be able to build properly.git

#### Tried everything and still broken
If its the Canalyzer Terminal, F303 Pad Controller, or Control Box:
Message Alexander Dobhmier

If its the Servo Board, or PCB Pad Controller: 
Message Brandon Marcus



## Making new projects
To make a new project within the monorepo, make a new project in stm32.
Right click the project name, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils
This allows you to use the coreutils.
To actually see the coreutils, right click the project name, then go to New > Folder. Drop down your project name, drop down Core, and click Inc. Then, below this, click Advanced > Link to alternate Folder, and type in WORKSPACE_LOC/Firmware/coreutils in the box. Hit Apply and close.

## Resources
