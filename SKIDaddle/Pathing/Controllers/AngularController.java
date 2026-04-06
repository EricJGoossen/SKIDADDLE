package Pathing.Controllers;

import Pathing.Kinematics;
import Pathing.Scheduler;
import Util.Angle;
import Util.MotionState;
import Util.AccelerationCommands.AngularAccelCommand;
import Util.Path.Arc;

public class AngularController implements Scheduler.AccelerationController{
    public interface TargetGenerator {
        Angle getAngle(MotionState state, Arc arc, double deltaT);
    }

    private TargetGenerator generator;

    private final double VEL_EPSILON = 0.3;
    private final double POS_EPSILON = Math.toRadians(5.0);

    private final double weight;
    private final int priority;

    private Angle lastTarget;
    private double lastSpeed;

    public AngularController(TargetGenerator generator, double weight, int priority) {
        this.generator = generator;

        this.weight = weight;
        this.priority = priority;

        lastTarget = new Angle();
        lastSpeed = Double.POSITIVE_INFINITY;
    }

    public void init() {}

    public AngularAccelCommand[] update(MotionState state, Arc arc, double deltaT) {
        Angle target = generator.getAngle(state, arc, deltaT);

        Angle deltaTheta = target.minus(state.pos.ang);

        double speed = Kinematics.angularSpeedProfile(0, deltaTheta);
        speed *= Math.signum(deltaTheta.get());

        double accel = (speed - state.vel.ang) / deltaT;

        AngularAccelCommand[] command = {new AngularAccelCommand(accel, weight, priority)};

        lastTarget = target;
        lastSpeed = speed;

        return command;
    }

    public boolean running(MotionState state, Arc arc, double deltaT) {
        return lastTarget.minus(state.pos.ang).abs() > POS_EPSILON ||
               Math.abs(lastSpeed) > VEL_EPSILON;
    }

    public boolean interrupt() {return true;}
    
    public boolean terminate() {return true;}
}
