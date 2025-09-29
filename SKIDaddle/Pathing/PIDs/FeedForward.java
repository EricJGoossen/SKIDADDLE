package Pathing.PIDs;

/**
 * Simple motor feedforward model.
 * Applies static, velocity, and acceleration terms to compute expected output.
 */
public class FeedForward {
    private final double Ks; // Static friction constant
    private final double Kv; // Velocity constant
    private final double Ka; // Acceleration constant

    /**
     * Constructs a feedforward model.
     *
     * @param Ks static friction term
     * @param Kv velocity term
     * @param Ka acceleration term
     */
    protected FeedForward(double Ks, double Kv, double Ka) {
        this.Ks = Ks;
        this.Kv = Kv;
        this.Ka = Ka;
    }

    /**
     * Computes the feedforward output for a motor.
     *
     * @param vel   desired velocity (units match Kv)
     * @param accel desired acceleration (units match Ka)
     * @return expected output
     */
    protected double compute(double vel, double accel) {
        return Math.copySign(Ks, vel) + Kv * vel + Ka * accel;
    }
}
