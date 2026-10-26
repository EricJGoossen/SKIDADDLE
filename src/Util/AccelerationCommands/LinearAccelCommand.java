package Util.AccelerationCommands;

import Pathing.SpaceKinematics;
import Util.Constants;
import Util.MotionState;
import Util.Vector;

public class LinearAccelCommand extends AccelerationCommand {
    public Vector accel;

    public LinearAccelCommand(Vector accel, double weight, int priority) {
        super(weight, priority);
        this.accel = accel;
    }

    @Override
    public MotorCommand toMotorCommand(double angle) {
        return SpaceKinematics.inverse(this, angle);
    }

    @Override
    public LinearAccelCommand fromMotorCommand(MotorCommand command, double angle) {
        return SpaceKinematics.forwardLinear(command, angle);
    }

    @Override
    public boolean isZero() {
        return accel.hypot() < Constants.ZERO_TOLERANCE;
    }

    @Override
    public void update(MotionState currentState) {
       currentState.accel.lin = currentState.accel.lin.plus(accel);
    }

    @Override
    public String toString() {
        return "LinearAccelCommand{" +
                "accel=" + accel +
                ", weight=" + weight +
                '}';
    }
}
