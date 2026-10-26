package Util.AccelerationCommands;

import Pathing.SpaceKinematics;
import Util.Constants;
import Util.MotionState;

public class AngularAccelCommand extends AccelerationCommand{
    public double accel;

    public AngularAccelCommand(double accel, double weight, int priority) {
        super(weight, priority);
        this.accel = accel;
    }

    @Override
    public MotorCommand toMotorCommand(double angle) {
        return SpaceKinematics.inverse(this);
    }

    @Override
    public AngularAccelCommand fromMotorCommand(MotorCommand command, double angle) {
        return SpaceKinematics.forwardAngular(command);
    }

    @Override
    public boolean isZero() {
        return Math.abs(accel) < Constants.ZERO_TOLERANCE;
    }

    @Override
    public void update(MotionState state) {
        state.accel.ang = state.accel.ang + accel;
    }

    @Override
    public String toString() {
        return "AngularAccelCommand{" +
                "accel=" + accel +
                ", weight=" + weight +
                '}';
    }
}
