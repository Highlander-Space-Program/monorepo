# monorepo

## Making new projects
To make a new project within the monorepo, make a new project in stm32.
Right click the project name, then go to Properties > C/C++ General > Paths and Symbols > Includes, then click add, and add ${workspace_loc}/Firmware/coreutils
This allows you to use the coreutils.