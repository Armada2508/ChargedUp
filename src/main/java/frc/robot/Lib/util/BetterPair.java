package frc.robot.Lib.util;

import edu.wpi.first.math.Pair;

public class BetterPair<A, B> extends Pair<A, B> {
    
    public BetterPair(A a, B b) {
        super(a, b);
    }

    @Override
    public String toString() {
        return "(" + getFirst() + ", " + getSecond() + ")";
    }

}
