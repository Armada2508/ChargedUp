package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    private WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);

    public GripperSubsystem() {
        talonFX.config_kP(0, Gripper.kP);
        talonFX.config_kI(0, Gripper.kI);
        talonFX.config_kD(0, Gripper.kD);
    }

    public void setPower(double powerInput) {
        talonFX.set(powerInput);
    }

    public double getPercentClosed() { //finds where the gripper is now, get from the motor
        return 0;
    }

    public void setPercentClosed(double positionInput) {//finds the ammount you want to travel (difference), sets the gripper to the desired position using the difference
        double requiredMovement = positionInput - getPercentClosed();
        double requiredRevolutions = (requiredMovement * 4);
        double requiredEncoderUnits = requiredRevolutions * 2048; //change variable name (?)
        double FinalEncoderUnits = talonFX.getSelectedSensorPosition(); //change variable name (?)

        talonFX.set(TalonFXControlMode.Position, (FinalEncoderUnits + requiredEncoderUnits));
    }

    public void calibrate() {
        talonFX.setSelectedSensorPosition(0);
    }

}

