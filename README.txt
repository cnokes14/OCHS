GITHUB Repository Notes:
- Only our source files are included here. No libraries, imports, etc..
- IDEs used: Keil uVision (KL05), Visual Studio (Windows SSH), VIM (Raspberry Pi)

Folders and Files
- Documentation -- Information regarding project status, usage, etc..
	- Old Files -- Outdated files that have been completely removed in refactoring, but we decided to keep for testing or documentation reasons.
- KL05 Nodes -- source files for the KL05Z nodes.
	- Voltage_Functions.s - assembly functions for accessing the I2C, GPIO, ADC, and other minor functionalities. Basically, device interfaces and things that need to run fast.
	- Supporting_Functions.s - assembly functions written for a college class, used in our application to reduce redundancy
	- ADC.c - main program and handler for complex math; should be reconfigured into independent supporting functions and main function files.
	- ADC.h - outline of all functions included from assembly files, and non-main functions from the C files.
- Raspberry Pi Interface -- source files for Raspberry Pi communication with KL05 nodes, Windows SSH
	- 
- Windows SSH Tool -- source files for communication with Raspberry Pi via SSH
	- 
