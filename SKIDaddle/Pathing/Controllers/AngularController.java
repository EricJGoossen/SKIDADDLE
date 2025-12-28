package Pathing.Controllers;

import Pathing.Kinematics;
import Pathing.Scheduler;
import Pathing.Scheduler.AccelerationController;
import Util.MotionState;
import Util.PathSegment;
import Util.AccelerationCommands.AngularAccelCommand;

public class AngularController implements Scheduler.AccelerationController{
    public interface TargetGenerator {
        double getAngle(MotionState state, PathSegment segment, double deltaT);
    }

    private TargetGenerator generator;

    private final double VEL_EPSILON = Math.toRadians(5.0);
    private final double POS_EPSILON = Math.toRadians(5.0);

    private final double weight;
    private final int priority;

    public AngularController(TargetGenerator generator, double weight, int priority) {
        this.generator = generator;
        this.weight = weight;
        this.priority = priority;
    }

    public void init() {}

    public double wrap(double n) {
        while (n >  Math.PI) n -= 2 * Math.PI;
        while (n < -Math.PI) n += 2 * Math.PI;
        return n;
    }

    public AngularAccelCommand[] update(MotionState state, PathSegment segment, double deltaT) {
        double target = generator.getAngle(state, segment, deltaT);
        target = wrap(target);

        double deltaTheta = wrap(target - state.pos.ang);

        double speed = Kinematics.speedProfile(0, deltaTheta);
        speed *= Math.signum(deltaTheta);

        double accel = (speed - state.vel.ang) / deltaT;

        AngularAccelCommand[] command = {new AngularAccelCommand(accel, weight, priority)};

        return command;
    }

    public boolean running(MotionState state, PathSegment segment, double deltaT) {
        double target = generator.getAngle(state, segment, deltaT);
        target = wrap(target);

        double deltaTheta = wrap(target - state.pos.ang);

        double speed = Kinematics.speedProfile(0, deltaTheta);
        speed *= Math.signum(deltaTheta);

        return Math.abs(target - state.pos.ang) < POS_EPSILON && Math.abs(speed) < VEL_EPSILON;
    }

    public boolean interrupt() {return true;}
    
    public boolean terminate() {return false;}
}
