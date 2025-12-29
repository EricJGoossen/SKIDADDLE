package Pathing.Controllers;

import Pathing.Kinematics;
import Pathing.Scheduler;
import Util.MotionState;
import Util.PathSegment;
import Util.Vector;
import Util.AccelerationCommands.LinearAccelCommand;

public class LinearController implements Scheduler.AccelerationController{
    private final double VEL_EPSILON = 0.1;
    private final double POS_EPSILON = 1.0;

    private final double convWeight;
    private final int convPriority;

    private final double transWeight;
    private final int transPriority;

    private double lastTarget;

    public LinearController(double convWeight, int convPriority, double transWeight, int transPriority) {
        this.convWeight = convWeight;
        this.convPriority= convPriority;

        this.transWeight = transWeight;
        this.transPriority = transPriority;

        lastTarget = 0.0;
    }

    public void init() {}

    public LinearAccelCommand[] update(MotionState state, PathSegment segment, double deltaT) {
        double dist = segment.distAlongSegment(state.pos.lin);
        double remainingDist = Math.max(0, dist - segment.transitionDistance);
        Vector dir = segment.last.dir(segment.current);

        double targetSpeed = Kinematics.linearSpeedProfile(segment.maxVel.hypot(), remainingDist, segment.lastSegDir);
        LinearAccelCommand[] commands = new LinearAccelCommand[2];

        Vector convVel = state.vel.lin.convergent(dir);
        double convAccel = (targetSpeed - convVel.hypot()) / deltaT;
        commands[0] = new LinearAccelCommand(dir.times(convAccel), convWeight, convPriority);

        Vector transVel = state.vel.lin.transverse(dir);
        commands[1] = new LinearAccelCommand(transVel.times(-1.0 / deltaT), transWeight, transPriority);

        lastTarget = targetSpeed;

        return commands;
    }

    public boolean running(MotionState state, PathSegment segment, double deltaT) {
        return state.pos.lin.minus(segment.next).hypot() > POS_EPSILON || 
               Math.abs(lastTarget) > VEL_EPSILON;
    }

    public boolean interrupt() {return true;}
    
    public boolean terminate() {return true;}
}
