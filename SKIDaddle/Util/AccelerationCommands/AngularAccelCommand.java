package Util.AccelerationCommands;

import Util.Constants;

public class AngularAccelCommand extends AccelerationCommand{
    public double accel;
    public double weight;

    public AngularAccelCommand(double accel, double weight) {
        this.accel = accel;
        this.weight = weight;
    }

    public AngularAccelCommand(double weight) {
        this.weight = weight;
    }

    public MotorCommand toMotorCommand() {
        double R = Constants.TURNING_RADIUS;

        MotorCommand command = new MotorCommand(weight, 0.0);

        command.speeds[MotorCommand.FL] =  R * accel;
        command.speeds[MotorCommand.FR] = -R * accel;
        command.speeds[MotorCommand.BL] =  R * accel;
        command.speeds[MotorCommand.BR] = -R * accel;

        return command;
    }

    public AngularAccelCommand fromMotorCommand(MotorCommand command) {
        double R = Constants.TURNING_RADIUS;

        double accel = (command.speeds[MotorCommand.FL] - 
                        command.speeds[MotorCommand.FR] + 
                        command.speeds[MotorCommand.BL] - 
                        command.speeds[MotorCommand.BR]) / (4 * R);

        return new AngularAccelCommand(accel, command.weight);
    }

    @Override
    public String toString() {
        return "AngularAccelCommand{" +
                "accel=" + accel +
                ", weight=" + weight +
                '}';
    }
}
