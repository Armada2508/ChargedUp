package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Lib.util.BetterPair;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class InverseKinematics {
    
    private static double a1 = 0;
    private static double a2 = 0;

    /**
     * @param lengthFirst - length of the first joint to the second joint
     * @param lengthSecond - length of the second joint to the end effector
     */
    public static void config(double lengthFirst, double lengthSecond) {
        if (lengthFirst <= 0 || lengthFirst <= 0) throw new IllegalArgumentException("Length is less than or equal to 0!");
        a1 = lengthFirst;
        a2 = lengthSecond;
    }
    
    /**
     * Converts coordinates to angles for the joints. The origin is the first joint.
     * @param x coordinate
     * @param y coordinate
     * @return angle to apply to first and second joint to get the end effector at position (x, y)
     */
    public static BetterPair<Double, Double> coordinatesToAngles(double x, double y) {
        double q1 = 0, q2 = 0;
        q2 = Math.acos(
            ((squared(x) + squared(y) - squared(a1) - squared(a2)) /
            (2 * a1 * a2))
        );
        q1 = Math.atan(y / x) - Math.atan(
            ((a2 * Math.sin(q2)) /
            (a1 + a2 * Math.cos(q2)))
        );
        return new BetterPair<Double,Double>(Math.toDegrees(q1), Math.toDegrees(q2));
    }

    public static SequentialCommandGroup getIKPositionCommand(double x, double y, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        BetterPair<Double, Double> angles = InverseKinematics.coordinatesToAngles(x, y);
        return new SequentialCommandGroup(
            new ArmCommand(angles.getFirst(), 45, 45, armSubsystem),
            new WristCommand(angles.getFirst() + angles.getSecond(), 10, 10, wristSubsystem)
        );
    }

    private static double squared(double num) {
        return Math.pow(num, 2);
    }

}
