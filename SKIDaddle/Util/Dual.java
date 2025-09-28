package Util;

/**
 * Represents a pair of linear and angular values.
 * Useful for storing dual quantities such as PID gains or control outputs.
 */
public class Dual {
    public double linear;
    public double angular;

    /**
     * Constructs a Dual with the given values.
     *
     * @param linear  linear component
     * @param angular angular component
     */
    public Dual(double linear, double angular) {
        this.linear = linear;
        this.angular = angular;
    }

    /**
     * Constructs a Dual initialized to (0, 0).
     */
    public Dual() {
        this(0.0, 0.0);
    }
}
