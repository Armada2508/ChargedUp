// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LED;
import frc.robot.lib.led.LEDStrip;
import frc.robot.lib.logging.NTLogger;

public class Robot extends TimedRobot {
	
	private RobotContainer container;
	private TimeOfFlight tof = new TimeOfFlight(0);
	private final PigeonIMU pigeon = new WPI_PigeonIMU(Constants.pigeonID);
	/** GRB */
	private final LEDStrip led = new LEDStrip(LED.port, LED.length);
	private Color offColor = new Color(0, 255, 0);
	
	@Override
	public void robotInit() {
		NTLogger.initDataLogger();
		DriverStation.silenceJoystickConnectionWarning(true);
		tof.setRangingMode(RangingMode.Short, 100);
		container = new RobotContainer(pigeon, tof);
		led.set(offColor);
	}
	
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		NTLogger.log();
		// if (tof.isRangeValid()) {
			// System.out.println("Distance: " + String.format("%.3f", tof.getRange() / 25.4) + ", Sigma: " + String.format("%.3f", (tof.getRangeSigma()  / 25.4)));
		// }
		// System.out.println("Pigeon: Yaw: " + pigeon.getYaw() + " Pitch: " + pigeon.getPitch() + " Roll: " + pigeon.getRoll());
	}
		
	@Override
	public void autonomousInit() {
		Constants.Balance.pitchOffset = -pigeon.getPitch();
		container.getBalanceSequence().schedule();
	}
	
	@Override
	public void autonomousPeriodic() {}
	
	@Override
	public void teleopInit() {
		container.stopEverything();
		led.pulseCommand(new Color(255, 0, 0), new Color(0, 255, 0), 0.75).schedule();
	}
	
	@Override
	public void teleopPeriodic() {}
	
	@Override
	public void disabledInit() {
		container.stopEverything();
		led.set(offColor);
	}
	
	@Override
	public void disabledPeriodic() {}
	
	@Override
	public void testInit() {}
	
	@Override
	public void testPeriodic() {}
	
	@Override
	public void simulationInit() {}
	
	@Override
	public void simulationPeriodic() {}

}