package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {
    
    private WPI_TalonFX talonFX = new WPI_TalonFX(Arm.motorID);

    public ArmSubsystem() {

    }

    @Override
    public void periodic() {}

    /**
     * 
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(power);
    }

    public void calibrate() {
        talonFX.setSelectedSensorPosition(0);
    }

}
