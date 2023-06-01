package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;
import frc.robot.lib.led.LEDStrip;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip led = new LEDStrip(LED.port, LED.length);

    public LEDSubsystem(PigeonIMU Pigeon) {

    }

    @Override
    public void periodic() {

    }
}
