# monorepo



## Starting up
1. Clone this repo and cd into it
2. Checkout true_monorepo
3. Open STM32CubeIDE and select the monorepo folder as the workspace
4. Open the project you want to work on in STM32CubeIDE.
5. Double click the .launch file in the root of the project to open it (no need to do anything with it, just open it so STM32 knows about it)
6. Click the debug button in STM32CubeIDE.
7. Everything should work! If it doesn't go to the README section with name of your board youre working on (ex. Servo Board Controller) or if you have problems even seeing the projects in STM32CubeIDE at all, go to ## Troubleshooting



## Servo Board Controller
For this board to work, it must be powered by 6v and connected with the JLink Edu Mini. 
1. Connect the JLink Edu Mini to the 10 exposed pads on the Servo Board.
2. Connect a USB to the JLink Edu Mini.
3. Connect 6v and GND to a stable power source. 
4. Click debug or run in STM32CubeIDE.



<!-- IDK plz halp alex -->
## Canalyzer Terminal
#### Connecting everyhting
1. Connect 1 USB cable to the F303K8 on the breadboard.
2. Connect the JLink Edu Mini to the 10 exposed pads on the Servo Board.
3. Connect the Male Molex connector from the breadboard to the port labeled CAN A or CAN B on the board you want to send a CAN message to.
4. Connect a USB to the JLink Edu Mini.
5. Connect 6v and GND to a stable power source.

#### Running the code
Currently, the F303K8 uses **UNKNOWN** baud rate. Therefore, in the canalyzer_terminal code, use baud rate **UNKNOWN**
6. Run `python3 canalyzer_terminal.py` (if it says its missing a package, use `python3 -m pip install <package_name>` and try again)
7. Type `l` then hit enter. It should show ST-Link as one of the options. If not, things are not connected properly.
8. Type `t` then hit enter. Type the exact name of the port that has the ST-Link and hit enter. Type `**UNKNOWN**` for baud rate and hit enter. 

<!-- IDK plz halp alex -->
9. To test if the F303K8 gets your messages, type `c` then hit enter and then type `t` and hit enter. it should turn off the F303K8 light.
10. To test if the other board is receiving messages, type `



## Troubleshooting
#### Can't find coreutils or config stuff when trying to build?
1. Is your workspace monorepo? If not, this may have issues. If if is and still not working, go to step 2.
2. Right click the project name in the Project Explorer, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils

#### You can't see coreutils in Inc
1. Navigate to the Inc folder in Project Explorer and right click it, then click New > Folder. Click Advanced > Link to alternate location (Linked Folder) and put `WORKSPACE_LOC/Firmware/coreutils` in the input box. Click Finish, and everything should be able to build properly.git

#### Tried everything and still broken
If its the Canalyzer Terminal, Pad Controller, or Control Box:
Message Alexander Dobhmier

If its the Servo Board: 
Message Brandon Marcus



## Making new projects
To make a new project within the monorepo, make a new project in stm32.
Right click the project name, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils
This allows you to use the coreutils.