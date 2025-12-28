package Pathing;

import Util.Constants;
import Util.PathSegment;
import Util.Vector;

/**
 * Handles forward and inverse kinematics for a mecanum drive.
 * Converts robot-relative velocity and acceleration into individual wheel motions.
 */
public class Kinematics {
    private final double motorVelCap;
    private final double motorAccel;

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
    protected Kinematics(double motorVelCap, double motorAccel) {
        this.motorVelCap = motorVelCap;
        this.motorAccel = motorAccel;
    }

    private static double findAccel(Vector dir) {
        return Constants.MAX_MOTOR_ACCELERATION / (Math.abs(dir.x) + Math.abs(dir.y));
    }

    public static double speedProfile(double exitSpeed, double dist, Vector dir) {
        dist = Math.abs(dist);

        double speed = Constants.MAX_MOTOR_SPEED;

        speed = Math.min(speed, Math.sqrt(exitSpeed * exitSpeed + 2.0 * findAccel(dir) * dist));

        return speed;
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
    protected TravelResult decelTravel(Vector entryVel, double exitSpeed, Vector lastSegDir, Vector nextSegDir) {
        //Vector lastSegDir = segment.lastSegDir;
        //Vector nextSegDir = segment.nextSegDir;

        //Split velocity
        Vector trans = entryVel.transverse(nextSegDir);
        double transMag = trans.hypot();

        Vector con = entryVel.convergent(nextSegDir);
        double conMag = con.hypot();

        // Time to decelerate transverse velocity
        double accel = findAccel(trans.norm());
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
        double speed = Math.sqrt(2 * findAccel(nextSegDir) * dist + exitSpeed * exitSpeed);
        return nextSegDir.times(speed);
    }

    /**
     * Computes stopping distance for a given speed.
     *
     * @param speed Scalar speed
     * @return Distance required to stop
     */
    protected double stopDist(double speed) {
        return speed * speed / (2 * motorAccel);
    }
}