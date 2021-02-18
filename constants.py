# Audio
# From http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS
BLOCK_SAMPLES = 500         # Sample rate of 20kHz and packages arriving at 40Hz == 500 samples per package
BLOCK_RATE = 40             # Package arrival rate in Hz
MICS = 4                    # Number of microphones on the robot
TONE_FREQUENCY = {          # Published tone frequency range
	'min': 50,
	'max': 2000
}
TONE_VOLUME = {             # Published tone volume range
	'min': 0,
	'max': 255
}

# Perception
PRI = {                     # Dimensions of visual salience map output
	'width' : 178,
	'height': 100
}
PRIW = {                    # Dimensions of audio salience map output
	'width' : 256,
	'height': 1
}
