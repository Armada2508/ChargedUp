package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    public double requiredMovement;
    public double requiredRevolutions;
    public double requiredEncoderUnits;

    private WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);

    public void setPower(double powerInput) {
        talonFX.set(powerInput);
    }

    public double getPercentClosed() { //finds where the gripper is now, get from the motor

        return 0;
    }

    public void setPercentClosed(double positionInput) {//finds the ammount you want to travel (difference), sets the gripper to the desired position using the difference
        requiredMovement = positionInput - getPercentClosed();
        requiredRevolutions = (requiredMovement * 4);
        requiredEncoderUnits = requiredRevolutions * 2048; //change variable name (?)
        // create code to edit motors using required encoder units 
    }
}
/* method to input number -1-1, -1 = gripper is open, 1 = gripper is closed, range between
  depending on the number between -1 and 1, change the gripper motors accordingly
  save the range value as a variable and send that to change the motors
  max open: 13in
  convert positionInput into something the motor can use
  convert required movement into revolutions (temp) then into encoder units, then change the motors that # of encoder units
  if position input is out of range, throw an execption
  guess 8 rotations to go from fully closed to fully opened and vice versa
 ! equations for rotations: f(x) = 4x
 */
