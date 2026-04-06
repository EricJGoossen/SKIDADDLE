package Pathing;

import Util.Angle;
import Util.Constants;
import Util.Vector;
import Util.Path.Arc;

/**
 * Handles forward and inverse kinematics for a mecanum drive.
 * Converts robot-relative velocity and acceleration into individual wheel motions.
 */
public final class Kinematics {
    private static final double VEL_CAP = Constants.MAX_MOTOR_SPEED;
    private static final double ACCEL = Constants.MAX_MOTOR_ACCELERATION;

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

    private static double findAccel(Vector dir) {
        return ACCEL / (Math.abs(dir.x) + Math.abs(dir.y));
    }

    public static double linearSpeedProfile(double exitSpeed, double dist, Vector dir) {
        dist = Math.abs(dist);

        double speed = VEL_CAP;

        speed = Math.min(speed, Math.sqrt(exitSpeed * exitSpeed + 2.0 * findAccel(dir) * dist));

        return speed;
    }

    public static double angularSpeedProfile(double exitSpeed, Angle angle) {
        double dist = Math.abs(angle.get());
        double R = Constants.TURNING_RADIUS;

        double speed = VEL_CAP / R;

        speed = Math.min(speed, Math.sqrt(exitSpeed * exitSpeed + 2.0 * ACCEL / R * dist));

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
    public static TravelResult decelTravel(Vector entryVel, Arc current, Arc next, double exitSpeed) {
        Vector currentDir = current.current.dir;
        Vector nextDir = next.current.dir;

        //Split velocity
        Vector trans = entryVel.transverse(nextDir);
        double transMag = trans.hypot();

        Vector con = entryVel.convergent(nextDir);
        double conMag = con.hypot();

        // Time to decelerate transverse velocity
        double accel = findAccel(trans.norm());
        double t = transMag / accel;

        // Find transverse and convergent distances traveled while transitioning segments
        double transDist = transMag * t - 0.5 * accel * t * t;
        Vector transTravel = transMag == 0 ? new Vector() : trans.div(transMag).times(transDist);

        double conDist = conMag * t;
        Vector conTravel = nextDir.times(conDist);

        Vector totalTravel = transTravel.plus(conTravel);

        // Project onto last segment
        Vector lastSegTravel = totalTravel.project(currentDir);
        Vector lastSegTanTravel = totalTravel.minus(lastSegTravel);

        // Adjust for segment angle
        double bCrossA = nextDir.cross(currentDir);
        double radAdjust = bCrossA == 0 ? 0 : nextDir.cross(lastSegTanTravel) / bCrossA;

        double currSegTravel = lastSegTravel.hypot() + radAdjust;
        double nextSegTravel = (conMag * conMag - exitSpeed * exitSpeed) / (2 * accel) + conDist;

        return new TravelResult(currSegTravel, nextSegTravel);
    }


    /**
     * Convenience method returning only current segment travel.
     */
    public static double decelTravel(Vector vel, Arc current, Arc next) {
        return decelTravel(vel, current, next, 0.0).curr;
    }

    /**
     * Computes maximum feasible speed along a segment given available distance and exit speed.
     *
     * @param nextSegDir Direction of segment
     * @param dist       Available distance
     * @param exitSpeed  Required speed at segment end
     * @return Vector representing maximum feasible velocity along segment
     */
    public static Vector speedOverDist(Vector nextSegDir, double dist, double exitSpeed) {
        double speed = Math.sqrt(2 * findAccel(nextSegDir) * dist + exitSpeed * exitSpeed);
        return nextSegDir.times(speed);
    }
}