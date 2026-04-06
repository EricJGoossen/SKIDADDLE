package Pathing;

import Util.Constants;
import Util.MotorVector;
import Util.Twist;
import Util.Vector;
import Util.AccelerationCommands.AngularAccelCommand;
import Util.AccelerationCommands.LinearAccelCommand;
import Util.AccelerationCommands.MotorCommand;

public final class SpaceKinematics {
    private static final double LAT = Constants.LATERAL_MULTIPLIER;
    private static final double ROT = Constants.TURNING_RADIUS;

    // Linear

    public static MotorCommand inverse(LinearAccelCommand cmd, double angle) {
        Vector acc = cmd.accel.rotate(angle);

        double[] speeds = {
            acc.y + acc.x * LAT,
            acc.y - acc.x * LAT,
            acc.y - acc.x * LAT,
            acc.y + acc.x * LAT
        };

        return new MotorCommand(cmd.weight, cmd.priority, speeds);
    }

    public static LinearAccelCommand forwardLinear(MotorCommand mc, double angle) {
        Vector acc = new Vector(mc.vector.Lateral(LAT), mc.vector.Longitudinal());

        return new LinearAccelCommand(acc.rotate(-angle), 
            mc.weight, 
            mc.priority
        );
    }

    // Angular

    public static MotorCommand inverse(AngularAccelCommand cmd) {
        double w = cmd.accel * ROT;

        double[] speeds= {
            w,
           -w,
            w,
           -w
        };

        return new MotorCommand(cmd.weight, cmd.priority, speeds);
    }

    public static AngularAccelCommand forwardAngular(MotorCommand mc) {
        double acc = mc.vector.Angular(ROT);

        return new AngularAccelCommand(acc, mc.weight, mc.priority);
    }

    // Twist

    public static MotorVector inverse(Twist t, double angle) {
        Vector acc = t.lin.rotate(angle);
        double w = t.ang * ROT;

        double[] speeds = {
            acc.y + acc.x * LAT + w,
            acc.y - acc.x * LAT - w,
            acc.y - acc.x * LAT + w,
            acc.y + acc.x * LAT - w
        };

        return new MotorVector(speeds);
    }

    public static Twist forwardTwist(MotorVector v, double angle) {
        Vector acc = new Vector(v.Lateral(LAT), v.Longitudinal());
        
        double angAcc = v.Angular(ROT);

        return new Twist(acc.rotate(-angle), angAcc);
    }
}
