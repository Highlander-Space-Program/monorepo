# monorepo

## Starting up
1. Clone this repo
2. Open STM32CubeIDE and select the monorepo folder as the workspace
3. Open the project you want to work on in STM32CubeIDE.
4. Right click the project name in the Project Explorer, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils
5. Navigate to the Inc folder in Project Explorer and right click it, then click New > Folder. Click Advanced > Link to alternate location (Linked Folder) and put `WORKSPACE_LOC/Firmware/coreutils` in the input box. Click Finish, and everything should be able to build properly.

## Making new projects
To make a new project within the monorepo, make a new project in stm32.
Right click the project name, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils
This allows you to use the coreutils.