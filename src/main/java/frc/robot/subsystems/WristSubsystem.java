package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;

public class WristSubsystem extends SubsystemBase {

    private WPI_TalonFX talonFX = new WPI_TalonFX(Wrist.motorID);

    public double setWristPosition() {

        
    }

}
//add method to set position of the wrist to x ammount of degrees similar to the arm, min degrees = -30, max degrees = 30
