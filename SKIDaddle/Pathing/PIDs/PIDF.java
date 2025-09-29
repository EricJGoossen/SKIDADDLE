package Pathing.PIDs;

import Util.MotionState;
import Util.Twist;
import Util.Dual;
import Util.Constants;

/**
 * Proportional–Integral–Derivative–Feedforward (PIDF) controller.
 * Computes correction terms based on position error, with optional
 * velocity and acceleration feedforward.
 */
public class PIDF {
    private final Dual Kp; // Proportional gain
    private final Dual Ki; // Integral gain
    private final Dual Kd; // Derivative gain
    private final Dual Kf; // Velocity feedforward gain
    private final Dual Ka; // Acceleration feedforward gain

    private Twist sumError = new Twist(); // Accumulated integral error
    private Twist lastError = new Twist(); // Error from previous step

    /**
     * Constructs a PIDF controller.
     *
     * @param Kp proportional gain
     * @param Ki integral gain
     * @param Kd derivative gain
     * @param Kf velocity feedforward gain
     * @param Ka acceleration feedforward gain
     */
    protected PIDF(Dual Kp, Dual Ki, Dual Kd, Dual Kf, Dual Ka) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Ka = Ka;
    }

    /**
     * Computes the PIDF correction vector for a pose target.
     *
     * @param target desired pose, velocity, acceleration
     * @param actual measured pose, velocity, acceleration
     * @param deltaT time since last update (s)
     * @return correction vector
     */
    protected Twist compute(MotionState target, MotionState actual, double deltaT) {
        if (deltaT < Constants.ZERO_TOLERANCE) {
            return new Twist();
        }

        // Position error
        Twist error = target.pos.minus(actual.pos);

        // Proportional term
        Twist p = error.times(Kp);

        // Integral term (accumulated error over time)
        sumError = sumError.plus(error.times(deltaT));
        Twist i = sumError.times(Ki);

        // Derivative term (rate of change of error)
        Twist d = error.minus(lastError).div(deltaT).times(Kd);
        lastError = error;

        // Feedforward terms
        Twist fV = target.vel.times(Kf); // velocity
        Twist fA = target.accel.times(Ka); // acceleration

        return p.plus(i).plus(d).plus(fV).plus(fA);
    }

    /**
     * Resets the controller’s stored error state.
     * Clears accumulated integral and derivative history.
     */
    protected void reset() {
        sumError = new Twist();
        lastError = new Twist();
    }
}
