package Pathing;

import Pathing.PIDs.FeedForward;
import Util.Constants;
import Util.MotionState;
import Util.Twist;
import Util.Vector;

/**
 * Handles forward and inverse kinematics for a mecanum drive.
 * Converts robot-relative velocity and acceleration into individual wheel motions.
 */
public class Kinematics {
    private final double maxSpeed;
    private final double accel;

    /**
     * Represents individual wheel speeds and accelerations.
     */
    protected static class WheelSpeed {
        public static final int FL = 0, FR = 1, BL = 2, BR = 3;

        public double[] vels = new double[4];
        private double[] accels = new double[4];

        protected WheelSpeed() {}

        protected WheelSpeed(WheelSpeed other) {
            this.vels = other.vels.clone();
            this.accels = other.accels.clone();
        }

        /**
         * Applies feedforward adjustments to wheel velocities.
         *
         * @param ff FeedForward object
         * @return New WheelSpeed with feedforward applied
         */
        protected WheelSpeed feedForward(FeedForward ff) {
            WheelSpeed w = new WheelSpeed(this);

            for (int i = 0; i < 4; i++)
                w.vels[i] = ff.compute(w.vels[i], w.accels[i]);

            return w;
        }
    }

    /**
     * Result of deceleration travel calculation along current and next segments.
     */
    protected static class TravelResult {
        public final double curr;
        public final double next;

        protected TravelResult(double curr, double next) {
            this.curr = curr;
            this.next = next;
        }
    }

    /**
     * Constructs a Kinematics object.
     *
     * @param maxSpeed Maximum allowable speed
     * @param accel    Maximum acceleration
     */
    protected Kinematics(double maxSpeed, double accel) {
        this.maxSpeed = maxSpeed;
        this.accel = accel;
    }

    /**
     * Converts robot-relative velocity and acceleration into wheel velocities and accelerations.
     *
     * @param state Robot motion state (velocity, acceleration)
     * @return WheelSpeed object representing individual wheel commands
     */
    protected static WheelSpeed inverse(MotionState state) {
        double R = Constants.TURNING_RADIUS; // Effective turning radius
        double latMult = Constants.LATERAL_MULTIPLIER;

        WheelSpeed w = new WheelSpeed();
        Twist vel = state.vel;
        Twist accel = state.accel;

        double omega = vel.ang.hypot();
        double alpha = accel.ang.hypot();

        // Compute wheel velocities
        w.vels[WheelSpeed.FL] = vel.lin.y + vel.lin.x * latMult + R * omega;
        w.vels[WheelSpeed.FR] = vel.lin.y - vel.lin.x * latMult - R * omega;
        w.vels[WheelSpeed.BL] = vel.lin.y - vel.lin.x * latMult + R * omega;
        w.vels[WheelSpeed.BR] = vel.lin.y + vel.lin.x * latMult - R * omega;

        // Compute wheel accelerations
        w.accels[WheelSpeed.FL] = accel.lin.y + accel.lin.x * latMult + R * alpha;
        w.accels[WheelSpeed.FR] = accel.lin.y - accel.lin.x * latMult - R * alpha;
        w.accels[WheelSpeed.BL] = accel.lin.y - accel.lin.x * latMult + R * alpha;
        w.accels[WheelSpeed.BR] = accel.lin.y + accel.lin.x * latMult - R * alpha;

        return w;
    }

    /**
     * Computes target speed along a segment considering acceleration, max speed, and braking distance.
     *
     * @param currSpeed Current robot speed
     * @param exitSpeed Target speed at segment end
     * @param dist      Remaining distance
     * @param deltaT    Time step
     * @return Target speed for next step
     */
    protected double speedProfile(double currSpeed, double exitSpeed, double dist, double deltaT) {
        dist = Math.abs(dist);

        // Accelerate
        currSpeed = currSpeed + accel * deltaT;

        // Cap to max speed
        currSpeed = Math.min(currSpeed, maxSpeed);

        // Decelerate if necessary
        currSpeed = Math.min(currSpeed,
            Math.sqrt(exitSpeed * exitSpeed + 2.0 * accel * dist)
        );

        return currSpeed;
    }

    /**
     * Convenience method assuming exit speed is zero.
     */
    protected double speedProfile(double currSpeed, double dist, double deltaT) {
        return speedProfile(currSpeed, 0, dist, deltaT);
    }

    /**
     * Returns true if robot is close enough to goal and moving slowly.
     *
     * @param vel           current velocity
     * @param dist          distance to goal
     * @param posEpsilon    position tolerance
     * @return True if goal reached
     */
    protected boolean atGoal(double vel, double dist, double posEpsilon, double deltaT) {
        double velDelta = accel * deltaT;

        return Math.abs(dist) < posEpsilon && Math.abs(vel) < velDelta;
    }

    /**
     * Computes deceleration travel along current and next segments.
     *
     * @param entryVel   Candidate entry velocity
     * @param exitSpeed  Target speed at end of next segment
     * @param lastSegDir Direction of current segment
     * @param nextSegDir Direction of next segment
     * @return TravelResult with distances along current and next segments
     */
    protected TravelResult decelTravel(Vector entryVel, double exitSpeed,
                                           Vector lastSegDir, Vector nextSegDir) {
        //Split velocity
        Vector trans = entryVel.transverse(nextSegDir);
        double transMag = trans.hypot();

        Vector con = entryVel.convergent(nextSegDir);
        double conMag = con.hypot();

        // Time to decelerate transverse velocity
        double t = transMag / accel;

        // Find transverse and convergent distances traveled while transitioning segments
        double transDist = transMag * t - 0.5 * accel * t * t;
        Vector transTravel = transMag == 0 ? new Vector() : trans.div(transMag).times(transDist);

        double conDist = conMag * t;
        Vector conTravel = nextSegDir.times(conDist);

        Vector totalTravel = transTravel.plus(conTravel);

        // Project onto last segment
        Vector lastSegTravel = totalTravel.project(lastSegDir);
        Vector lastSegTanTravel = totalTravel.minus(lastSegTravel);

        // Adjust for segment angle
        double bCrossA = nextSegDir.cross(lastSegDir);
        double radAdjust = bCrossA == 0 ? 0 : nextSegDir.cross(lastSegTanTravel) / bCrossA;

        double currSegTravel = lastSegTravel.hypot() + radAdjust;
        double nextSegTravel = (conMag * conMag - exitSpeed * exitSpeed) / (2 * accel) + conDist;

        return new TravelResult(currSegTravel, nextSegTravel);
    }


    /**
     * Convenience method returning only current segment travel.
     */
    protected double decelTravel(Vector entryVel, Vector lastSegDir, Vector nextSegDir) {
        return decelTravel(entryVel, 0, lastSegDir, nextSegDir).curr;
    }

    /**
     * Computes maximum feasible speed along a segment given available distance and exit speed.
     *
     * @param nextSegDir Direction of segment
     * @param dist       Available distance
     * @param exitSpeed  Required speed at segment end
     * @return Vector representing maximum feasible velocity along segment
     */
    protected Vector speedOverDist(Vector nextSegDir, double dist, double exitSpeed) {
        double speed = Math.sqrt(2 * accel * dist + exitSpeed * exitSpeed);
        return nextSegDir.times(speed);
    }

    /**
     * Computes stopping distance for a given speed.
     *
     * @param speed Scalar speed
     * @return Distance required to stop
     */
    protected double stopDist(double speed) {
        return speed * speed / (2 * accel);
    }
}