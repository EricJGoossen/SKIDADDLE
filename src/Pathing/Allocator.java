package Pathing;

import Util.Constants;
import Util.MotorVector;
import Util.Twist;
import Util.AccelerationCommands.AccelerationCommand;
import Util.AccelerationCommands.MotorCommand;

public final class Allocator {
    private static final double ACCEL= Constants.MAX_MOTOR_ACCELERATION;
    private static final double VEL_CAP = Constants.MAX_MOTOR_SPEED;
    private static final double ZERO_TOLERANCE = Constants.ZERO_TOLERANCE;

    private record Allocation(MotorCommand cmd, MotorVector scales, boolean allocated) {}

    private static boolean exhausted(MotorVector h) {
        for (int i = 0; i < h.motorCount(); i++) {
            if (1 - Math.abs(h.speeds[i]) < Constants.ZERO_TOLERANCE) return true;
            if (1 - Math.abs(h.speeds[i]) < Constants.ZERO_TOLERANCE) return true;
        }
        return false;
    }

    private static double maxScaleForCommand(MotorVector limit, MotorVector headroom, double weight) {
        double maxScale = Double.POSITIVE_INFINITY;
        MotorVector norm = limit.normalize();

        for (int i = 0; i < limit.motorCount(); i++) {
            // A wheel this command doesn't use shouldn't limit it
            if (Math.abs(norm.speeds[i]) < ZERO_TOLERANCE) continue;
            int dir = (int) Math.signum(limit.speeds[i]);

            double available = Math.abs(dir - headroom.speeds[i]);
            available *= weight;
            if (available <= 0) return 0.0;

            double motorLimit = Math.abs(limit.speeds[i]) / Constants.MAX_MOTOR_ACCELERATION;
            double motorScale = Math.min(available, motorLimit) / Math.abs(norm.speeds[i]);

            maxScale = Math.min(maxScale, motorScale);
        }

        return Double.isInfinite(maxScale) ? 0.0 : maxScale;
    }

    private static Allocation[] sort(Allocation[] intents) {
        for (int i = 0; i < intents.length - 1; i++) {
            int maxIdx = i;
            for (int j = i + 1; j < intents.length; j++) {
                if (intents[j].cmd.priority > intents[maxIdx].cmd.priority) {
                    maxIdx = j;
                }
            }

            if (maxIdx != i) {
                Allocation temp = intents[i];
                intents[i] = intents[maxIdx];
                intents[maxIdx] = temp;
            }
        }

        return intents;
    }

    private static int lowestPriority(Allocation[] intents, int numActive) {
        int lowest = Integer.MAX_VALUE;
        for (int i = 0; i < numActive; i++) lowest = Math.min(lowest, intents[i].cmd.priority);
        return lowest;
    }

    private static MotorVector findHeadroom(MotorVector vel, double deltaT) {
        MotorVector headroom = new MotorVector();
        for (int i = 0; i < vel.motorCount(); i++) {
            double v = vel.speeds[i];
            double dir = v == 0 ? 1 : Math.signum(v);
            double limit = Math.min((VEL_CAP - Math.abs(v)) / deltaT, ACCEL);
            headroom.speeds[i] = (1 - limit / ACCEL) * dir;
        }

        return headroom;
    }

    protected static final Twist acceleration(
            AccelerationCommand[] commands, 
            Twist velState,
            double angle,
            double deltaT
        ) {
        if (commands.length == 0) return new Twist();

        MotorVector headroom = findHeadroom(velState.toMotorSpace(angle), deltaT);

        Allocation[] intents = new Allocation[commands.length];
        for (int i = 0; i < commands.length; i++) {
            intents[i] = new Allocation(commands[i].toMotorCommand(angle), new MotorVector(), false);
        }

        intents = sort(intents);
        
        int numActive = intents.length;
        double priorityRound = Double.POSITIVE_INFINITY;
        while (numActive > 0) {
            priorityRound = Math.min(priorityRound, intents[0].cmd.priority);
            double weightSum = 0.0;

            for (int i = 0; i < numActive; i++) {
                Allocation intent = intents[i];
                if (intent.cmd.priority < priorityRound) break;
                weightSum += intent.cmd.weight;
            }

            MotorVector deltaHeadroom = new MotorVector();
            for (int i = 0; i < numActive; i++) {
                MotorCommand intent = intents[i].cmd;
                if (intent.priority < priorityRound) break;
                
                double scale = maxScaleForCommand(intent.vector, headroom, intent.weight / weightSum);

                MotorVector scales = intent.vector.normalize().times(scale);
                // A command is done when everything given to it so far, not just this
                // round's share, covers what it asked for.
                MotorVector total = scales.plus(intents[i].scales);
                boolean allocated = Math.abs(total.max() * ACCEL - intent.vector.max()) < ZERO_TOLERANCE;

                deltaHeadroom = deltaHeadroom.plus(scales);
                intents[i] = new Allocation(intent, total, allocated);
            }

            headroom = headroom.plus(deltaHeadroom);
            // Move down a priority when the headroom is used up, or when this round gave
            // nothing more to anyone 
            boolean progressed = false;
            for (int i = 0; i < deltaHeadroom.motorCount(); i++) {
                if (Math.abs(deltaHeadroom.speeds[i]) > ZERO_TOLERANCE) progressed = true;
            }
            if (exhausted(headroom) || !progressed) {
                if (priorityRound <= lowestPriority(intents, numActive) || priorityRound <= 0) break;
                priorityRound--;
            }

            for (int i = numActive - 1; i >= 0; i--) {
                if (intents[i].allocated) {
                    for (int j = i; j < numActive - 1; j++) {
                        Allocation temp = intents[j];
                        intents[j] = intents[j + 1];
                        intents[j + 1] = temp;
                    }
                    numActive--;
                }
            }
        }

        MotorVector totalAccel = new MotorVector();
        for (Allocation intent : intents) {
            totalAccel = totalAccel.plus(intent.scales.times(ACCEL));
        }

        return totalAccel.toTwist(angle);
    }
}
