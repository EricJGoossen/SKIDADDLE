package Util.AccelerationCommands;

import Util.Constants;
import Util.Vector;

public class LinearAccelCommand extends AccelerationCommand {
    public Vector accel;
    public double weight;

    public LinearAccelCommand(Vector accel, double weight) {
        this.accel = accel;
        this.weight = weight;
    }

    public LinearAccelCommand(double weight) {
        this.weight = weight;
    }

    @Override
    public MotorCommand toMotorCommand() {
        double latMult = Constants.LATERAL_MULTIPLIER;

        MotorCommand command = new MotorCommand(weight, 0.0);

        command.speeds[MotorCommand.FL] = accel.y + accel.x * latMult;
        command.speeds[MotorCommand.FR] = accel.y - accel.x * latMult;
        command.speeds[MotorCommand.BL] = accel.y - accel.x * latMult;
        command.speeds[MotorCommand.BR] = accel.y + accel.x * latMult;

        return command;
    }

    @Override
    public LinearAccelCommand fromMotorCommand(MotorCommand command) {
        double latMult = Constants.LATERAL_MULTIPLIER;
        
        Vector newAccel = new Vector(
            (command.speeds[MotorCommand.FL] - 
            command.speeds[MotorCommand.FR] -
            command.speeds[MotorCommand.BL] + 
            command.speeds[MotorCommand.BR]) / (4 * latMult),
            (command.speeds[MotorCommand.FL] + 
            command.speeds[MotorCommand.FR] + 
            command.speeds[MotorCommand.BL] +
            command.speeds[MotorCommand.BR]) / 4
        );

        return new LinearAccelCommand(newAccel, command.weight);
    }

    @Override
    public String toString() {
        return "LinearAccelCommand{" +
                "accel=" + accel +
                ", weight=" + weight +
                '}';
    }
}
