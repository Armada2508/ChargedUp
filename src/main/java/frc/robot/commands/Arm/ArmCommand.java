package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private double targetDegrees;
    private ArmSubsystem armSubsystem;

    /**
     * 
     * @param theta degree to go to, 0 is straight down
     * @param armSubsystem
     */
    public ArmCommand(double theta, ArmSubsystem armSubsystem) {
        targetDegrees = theta;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setPosition(targetDegrees);  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = armSubsystem.getPosition();
        return (Math.abs(currentDegrees) < targetDegrees+degreesDeadband);
    }
    
}
