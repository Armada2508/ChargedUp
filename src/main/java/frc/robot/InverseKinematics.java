package frc.robot;

import edu.wpi.first.math.Pair;

public class InverseKinematics {
    
    private static double a1 = 0;
    private static double a2 = 0;

    /**
     * @param lengthFirst - length of the first joint to the second joint
     * @param lengthSecond - length of the second joint to the end effector
     */
    public static void config(double lengthFirst, double lengthSecond) {
        a1 = lengthFirst;
        a2 = lengthSecond;
    }
    
    /**
     * Converts coordinates to angles for the joints. The origin is the first joint.
     * @param x coordinate
     * @param y coordinate
     * @return angle to apply to first and second joint to get the end effector at position (x, y)
     */
    public static Pair<Double, Double> coordinatesToAngles(double x, double y) {
        double q1 = 0, q2 = 0;
        q2 = Math.acos(
            ((squared(x) + squared(y) - squared(a1) - squared(a2)) /
            (2 * a1 * a2))
        );
        q1 = Math.atan(y / x) - Math.atan(
            ((a2 * Math.sin(q2)) /
            (a1 + a2 * Math.cos(q2)))
        );
        return new Pair<Double,Double>(q1, q2);
    }

    private static double squared(double num) {
        return Math.pow(num, 2);
    }

}
