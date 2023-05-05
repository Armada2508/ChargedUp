// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LED;
import frc.robot.lib.led.LEDStrip;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
	
	private RobotContainer container;
	private TimeOfFlight tof = new TimeOfFlight(0);
	private final PigeonIMU pigeon = new WPI_PigeonIMU(Constants.pigeonID);
	private final LEDStrip led = new LEDStrip(LED.port, LED.length);
	
	@Override
	public void robotInit() {
		DriverStation.silenceJoystickConnectionWarning(true);
		tof.setRangingMode(RangingMode.Short, 100);
		container = new RobotContainer(pigeon, tof);
		led.set(new Color(255, 0, 0));
	}
	
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		// if (tof.isRangeValid()) {
			// System.out.println("Distance: " + String.format("%.3f", tof.getRange() / 25.4) + ", Sigma: " + String.format("%.3f", (tof.getRangeSigma()  / 25.4)));
			// }
			// System.out.println("Pigeon: Yaw: " + pigeon.getYaw() + " Pitch: " + pigeon.getPitch() + " Roll: " + pigeon.getRoll());
		}
		
	@Override
	public void autonomousInit() {
		Constants.Balance.pitchOffset = -pigeon.getPitch();
		container.getAltAutoCommand().schedule();
	}
	
	@Override
	public void autonomousPeriodic() {}
	
	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
		container.stopEverything();
		final Color red = new Color(255, 0, 0);
		final Color green = new Color(0, 255, 0);
		new RepeatCommand(
			new SequentialCommandGroup(
				runOnce(() -> led.set(green)),
				waitSeconds(0.5),
				runOnce(() -> led.set(red))
			)
		).schedule();
	}
	
	@Override
	public void teleopPeriodic() {}
	
	@Override
	public void disabledInit() {
		container.stopEverything();
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