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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.logging.NTLogger;

public class Robot extends TimedRobot {
	
	private RobotContainer container;
	private TimeOfFlight tof = new TimeOfFlight(0);
	private final PigeonIMU pigeon = new WPI_PigeonIMU(Constants.pigeonID);
	
	@Override
	public void robotInit() {
		NTLogger.initDataLogger();
		DriverStation.silenceJoystickConnectionWarning(true);
		tof.setRangingMode(RangingMode.Short, 100);
		container = new RobotContainer(pigeon, tof);
	}
	
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		NTLogger.log();
	}
		
	@Override
	public void autonomousInit() {
		Constants.Balance.pitchOffset = -pigeon.getPitch();
		// container.getAltAutoCommand().schedule();
	}
	
	@Override
	public void autonomousPeriodic() {}
	
	@Override
	public void teleopInit() {
		container.stopEverything();
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