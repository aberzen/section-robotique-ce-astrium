------------------------------------------------------------------------------------------------
-- Introduction
------------------------------------------------------------------------------------------------
This file introduces the dependencies and the way they must be installed in current directory 
in order to be linked to this project (if good ideas to significantly improve the process, 
please do not hesitate).

The dependencies are:
- FreeRTOS (expected V7.3.0 version)
- Mavlink protocole (expected V1.0.9 version)

------------------------------------------------------------------------------------------------
-- MAVLINK - V1.0.9
------------------------------------------------------------------------------------------------
install mavlink library (tested with version v1.0.9) into this directory by 
unzipping (hope this word exists) the mavlink library into this directory

------------------------------------------------------------------------------------------------
-- FreeRTOS - V7.3.0
------------------------------------------------------------------------------------------------
Install the FreeRTOS into this directory (tested with version V7.3.0).

The expected file tree starting from here is:
- FreeRTOSV7.3.0
  - FreeRTOS
  	- Demo (not mandatory)
  	- License (not mandatory)
  	- Source
  	  - include
  	  - portable
  	  - ...
	- readme.txt
  - FreeRTOS-Plus
  - readme.txt

In order integrating the FreeRTOSV7.3.0 into autopilot, performs the following operations:

1) Install heap management
copy FreeRTOSV7.3.0/FreeRTOS/Source/portable/MemMang/heap_3.c to FreeRTOSV7.3.0/FreeRTOS/Source
 
2) Add support to Atmega2560
patch the FreeRTOSV7.3.0/FreeRTOS/Source/include/portable.h using portable.h.patch patch file.
For this, you can execute (under linux or cygwin terminal) the following command from FreeRTOSV7.3.0/FreeRTOS/Source/include/ directory:
patch < ../../../../FreeRTOS_patches/portable.h.patch

Note that FreeRTOS is not expected to be part of the GIT repository.
Never add FreeRTOS files to Git repository.
 
  