package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    private WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);

    public void setPower(double power) {
        talonFX.set(power);
    }

    public void setPercentClosed(double percent) {

    }

    public double getPercentClosed() {
        return 0;
    }

}
