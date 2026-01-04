package Pathing;

import Util.AccelerationCommands.AccelerationCommand;
import Util.Path.Arc;

import java.util.ArrayList;

import Pathing.Controllers.DefaultController;
import Pathing.Controllers.DefaultController.CommandGenerator;
import Util.MotionState;

public class Scheduler {

    public interface AccelerationController {
        void init();
        AccelerationCommand[] update(MotionState state, Arc arc, double deltaT);
        boolean running(MotionState state, Arc arc, double deltaT);
        boolean interrupt();
        boolean terminate();
    }

    private ArrayList<AccelerationController> controllers;
    private DefaultController defaultController;

    public Scheduler() {
        controllers = new ArrayList<>();

        defaultController = new DefaultController();
    }

    public void init() {
        for (AccelerationController controller : controllers) {
            controller.init();
        }
    }

    public AccelerationCommand[] update(MotionState state, Arc arc, double deltaT) {
        ArrayList<AccelerationController> deadControllers = new ArrayList<>();
        ArrayList<AccelerationCommand> commands = new ArrayList<>();

        for (AccelerationController controller : controllers) {
            if (controller.running(state, arc, deltaT)) {
                AccelerationCommand[] newCommands = controller.update(state, arc, deltaT);

                for (AccelerationCommand command : newCommands) {
                    if (!command.isZero()) commands.add(command);
                }
            } else {
                if (controller.terminate()) deadControllers.add(controller);
            }
        }

        controllers.removeAll(deadControllers);

        if (controllers.isEmpty()) {
            AccelerationCommand[] newCommands = defaultController.update(state, arc, deltaT);

            for (AccelerationCommand command : newCommands) {
                if (!command.isZero()) commands.add(command);
            }
        }

        return commands.toArray(new AccelerationCommand[0]);
    }

    public void halt() {
        ArrayList<AccelerationController> deadControllers = new ArrayList<>();

        for (AccelerationController controller : controllers) {
            controller.interrupt();
            if (controller.terminate()) deadControllers.add(controller);
        }

        controllers.removeAll(deadControllers);
    }

    public void addController(AccelerationController controller) {
        controllers.add(controller);
    }

    public void setDefaultCOntroller(CommandGenerator defaultCommandGenerator) {
        defaultController.setGenerator(defaultCommandGenerator);
    }

    public boolean isFinished() {
        return controllers.isEmpty();
    }
}
