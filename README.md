# thesisFinal

# How to run it:
	In diferent terminals open in this order
	1. $ yarpserver
	2. $ iCub_SIM
	3. $ yarpmotorgui
	4. $ iKinGazeCtrl --robot icubSim --imu::mode off

	after compiling
	5. $ ./main
	It will ask "Read backup? [y/n]". Answer only after step 6. is done
	
	in a different terminal
	6. $ yarp connect /icubSim/cam/left /aruco/image/in/exec

# Changing joint selection methods:
	In the file parameters.txt change the option 'crit_name' to AL or random

# To use cost-sensitive methods:
	In the file parameters.txt, change the value of 'costCoef' (unconstrained version, UCSAL) and 'delta' (constrained version, CCSAL)
	In the file CalibrationRoutine.hpp, the class CalibrationRoutine contains a 'private bool constrained'. If set to true, it is used the CCSAL (remember to set costCoef to 0 in this case). If set to false, it is used the UCSAL. 

# To be able to see the output of the cameras with the ArUco module:
	1. Go to /iCubVision
	2. After iCub_SIM is running, Compile and run
	3. In a different terminal type: $ yarp connect /icubSim/cam/left /aruco/image/in/debug

# Common issues:
	a) Sometimes the textures of aruco markers are not loaded and they stay white. It is solved by killing the process and repeating step 5 until the textures are visible
	b) After a few hours iCub_SIM crashes when using markers. It will automatically backup all information. Then just answer 'y' when the backup prompt appears when restarting the program

