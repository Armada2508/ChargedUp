package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private DoubleSupplier targetDegrees;
    private ArmSubsystem armSubsystem;

    /**
     * 
     * @param theta degree to go to, 0 is straight down.
     * @param armSubsystem
     */
    public ArmCommand(double theta, ArmSubsystem armSubsystem) {
        this(() -> theta, armSubsystem);
    }

    /**
     * 
     * @param theta degree to go to, 0 is straight down.
     * @param armSubsystem
     */
    public ArmCommand(DoubleSupplier theta, ArmSubsystem armSubsystem) {
        targetDegrees = theta;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.configMotionMagic(degreesDeadband, degreesDeadband);
        armSubsystem.setPosition(targetDegrees.getAsDouble());  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPower(0);
        armSubsystem.holdPosition();
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = armSubsystem.getMotionMagicPosition();
        return (currentDegrees < targetDegrees.getAsDouble()+degreesDeadband && currentDegrees > targetDegrees.getAsDouble()-degreesDeadband);
    }
    
}
