package Util.AccelerationCommands;

public abstract class AccelerationCommand {
    public abstract AccelerationCommand fromMotorCommand(MotorCommand motorCommand);
    public abstract MotorCommand toMotorCommand();
}
