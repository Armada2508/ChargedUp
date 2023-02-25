# Commands:

### Manual (Raw Power) (Relative):	
- ~~DriveCommand~~
- ~~ArmCommand~~ !
- ~~WristCommand~~ !
- ~~GripCommand~~ !

### Basic Auto:
- ~~AutoDriveForward/Backward (Drives robot number of inches) (Relative)~~ !
- ~~AutoTurn (Turn robot number of degrees) (Relative)~~ !
- ~~AutoArm (Position arm to certain degrees) (0 : 90) (Absolute)~~ !
- ~~AutoWrist (Position wrist to certain degrees) (-30 : 30) (Absolute)~~ !
- ~~AutoGrip (Claw percent close) (-1 is open, 1 is closed) (Absolute)~~ !
- ~~Calibrate Commands~~ !
### Complicated Auto:
- ~~Balancing Command~~
- ~~Put a cone on pole command~~ !
- ~~Put a cube on a station command~~ !
- ~~Put a game piece on the floor~~ !
- ~~AutoPickupCommand~~  !
- ~~GamePieceOnTopCommand~~ !
- ~~Inverse Kinematics~~ !
	
# Subsystems:

- DriveSubsystem
- ArmSubsystem
- WristSubsystem
- GripperSubsystem
- VisionSubsystem (Limelight/PhotonVision/GRIP/OpenCV) haha ha... woo

## Vision:

- Command to line up with a pole

## Constants:

- Go through all constants and ensure they are the right values (gearbox ratios, motor encoder units per rev, etc.) 

## Miscellaneous:

- ~~Calibration~~
- ~~Cameras~~
- Shuffleboard Data

### Key: <br>
! = test <br>
? = review

## poo poo

~~slew rate limiter~~
~~constant curvature~~ ?
velocity closed loop driving - ima put this on back burner for a while

motion magic instead of position done for all except gripper cuz idk for that one. Tested motion magic with drive subsystem.
Now create and test trajectories and continue troubleshooting vision so that we can get an accurate pose from april tags and actually use that for cool trajectories.
made a move to relative command for simple- trajectories