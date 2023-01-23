Commands:

	Manual(Raw Power) (Relative):	
	x - DriveCommand 
	x - ArmCommand
	- WristCommand
	- GripCommand
	
	Basic Auto:
	x - AutoDriveForward/Backward(Drives robot number of inches) (Relative)
	x? - AutoTurn(Turn robot number of degrees) (Relative)
	x! - AutoArm(Position arm to certain degrees) (0 : 90) (Absolute)
	- AutoWrist(Position wrist to certain degrees) (-30 : 30) (Absolute)
	- AutoGrip(Claw percent close) (0 is open, 1 is closed) (Absolute)
	Complicated Auto:
	x! - Balancing Command
	- Put a cone on pole command
	- Pick up a cone command
	
Subsystems:

- DriveSubsystem
- ArmSubsystem
- WristSubsystem
- GripperSubsystem
- VisionSubsystem (Limelight)

Vision:

x!- Command to line up with a pole/cone     or cube(low priority)

Miscellaneous:

- Cameras
- Shuffleboard Data

Key: <br>
x = done <br>
! = test <br>
? = review