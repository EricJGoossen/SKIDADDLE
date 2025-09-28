package Pathing;
import java.util.ArrayList;

import Simulator.Draw.Graph;
import Util.Constants;
import Util.Path;
import Util.PoseVelAcc;
import Util.Vector;
import Util.MotionState;

public class UnitTests {
    private final Graph draw;

    private final double deltaT = Constants.SIM_DELTA_TIME;

    public UnitTests(Graph draw) {
        this.draw = draw;
    }

    public void test() {
        int test = 2;

        switch (test) {
            case 0:
                pathGenTest();
                break;
            case 1:
                //lineTest();
                break;
            case 2:
                splineTest();
                break;
            case 3:
                //initVelTest();
                break;
            case 4:
                //linePathCreater();
                break;
            case 5:
                //splinePathCreater();
                break;
        }
    }

    private void updateLog() {
        if (Constants.SIMULATING) {
            draw.updateGraph();
            draw.updateField();
        }
    }

    private void pathGenTest() {
        if (Constants.SIMULATING) draw.setShowRobot(false);
        Vector[] points = new Vector[4];

        points[0] = new Vector(0, 24);
        points[1] = new Vector(0, 60);
        points[2] = new Vector(-48,24);
        points[3] = new Vector(-48, 60);

        Path path = new Path();
        path.create(points, Constants.LINE_APPROX_EPSILON);

        path.draw(0, draw);
        path.printPoints();
    }

    /*public void lineTest() {
        //Create path
        ArrayList<Vector> points = new ArrayList<>();

        points.add(new Vector(0, 0));
        points.add(new Vector(0, 24));
        points.add(new Vector(24, 48));

        //Starting pose
        PoseVelAcc lin = new PoseVelAcc(points.get(0), new Vector(), new Vector());
        PoseVelAcc ang = new PoseVelAcc(new Vector(), new Vector(), new Vector());

        MotionState pose = new MotionState(lin, ang);
        pose.moving = true; // Indicate that the robot is moving

        //Init pathing
        Controller driveTo = new Controller(draw, null, 0, points, null);
        driveTo.setInitState(pose);

        while (pose.moving) {
            pose = driveTo.move(pose, new MotionState(), deltaT);

            updateLog();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {

            }
        }

        System.out.println("done");
    }*/

    public void splineTest() {
        //Create path
        Vector[] points = new Vector[4];

        points[0] = new Vector(0, 24);
        points[1] = new Vector(0, 60);
        points[2] = new Vector(60,24);
        points[3] = new Vector(60, 60);

        Path path = new Path();
        path.create(points, Constants.ANGLE_EPSILON);

        //Starting pose
        PoseVelAcc lin = new PoseVelAcc(points[0], new Vector(), new Vector());
        PoseVelAcc ang = new PoseVelAcc(new Vector(1, 0), new Vector(), new Vector());

        MotionState pose = new MotionState(lin, ang);
        pose.moving = true; // Indicate that the robot is moving

        //Init pathing
        Angular.Controller angController = Angular.turnToAngle(Math.toRadians(-135));
        Controller driveTo = new Controller(draw, null);
        driveTo.setInitState(pose);

        //Setup graphing
        if (Constants.SIMULATING) draw.setShowRobot(true);

        while (pose.moving) {
            pose = driveTo.update(pose, new MotionState(), path, angController, deltaT);

            updateLog();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {

            }
        }

        System.out.println("done");
    }

    /*private void initVelTest() {
        //Create path
        Vector p1 = new Vector(0, 24);
        Vector p2 = new Vector(60, 60);

        Vector initVel = new Vector(0, 24);
        Vector exitVel = new Vector(0, 0);

        Path.Hermite herm = new Path.Hermite(p1, p2, initVel, exitVel);

        //Init pathing
        Controller driveTo = new Controller(draw, null, initVel.hypot(), herm, null);
        driveTo.path.printPoints();

        //Starting pose
        PoseVelAcc lin = new PoseVelAcc(driveTo.path.points()[0], initVel, new Vector());
        PoseVelAcc ang = new PoseVelAcc(new Vector(), new Vector(), new Vector());

        MotionState pose = new MotionState(lin, ang);
        pose.moving = true; // Indicate that the robot is moving

        driveTo.setInitState(pose);

        //Setup graphing
        if (Constants.SIMULATING) draw.setShowRobot(true);

        
        while (pose.moving) {
            pose = driveTo.move(pose, new MotionState(), deltaT);

            updateLog();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {

            }
        }

        System.out.println("done");
    }

    public void linePathCreater() {
        if (Constants.SIMULATING) draw.setShowRobot(false); // Hide the robot during path creation

        while(true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                // Handle interruption
            }

            //Init pathing
            Controller driveTo = new Controller(draw, null);
            driveTo.simLinePath(null);

            if (driveTo.path == null || driveTo.path.points().length < 2) continue;

            //Starting pose
            PoseVelAcc lin = new PoseVelAcc(driveTo.path.points()[0], new Vector(), new Vector());
            PoseVelAcc ang = new PoseVelAcc(new Vector(), new Vector(), new Vector());

            MotionState pose = new MotionState(lin, ang);
            pose.moving = true; // Indicate that the robot is moving
            driveTo.setInitState(pose);

            while (pose.moving) {
                try {
                    pose = driveTo.move(pose, new MotionState(), deltaT);
                } catch (IllegalStateException e) {
                    System.out.println("Error setting path from pose: " + e.getMessage());
                    break;
                }

                updateLog();
            }
        }
    }

    public void splinePathCreater() {
        if (Constants.SIMULATING) draw.setShowRobot(false); // Hide the robot during path creation
        

        while(true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                // Handle interruption
            }

            //Init pathing
            Controller driveTo = new Controller(draw, null);
            driveTo.simSplinePath(null);

            if (driveTo.path == null) continue; // No path created, continue waiting

            //Starting pose
            PoseVelAcc lin = new PoseVelAcc(driveTo.path.points()[0], new Vector(), new Vector());
            PoseVelAcc ang = new PoseVelAcc(new Vector(), new Vector(), new Vector());

            MotionState pose = new MotionState(lin, ang);
            pose.moving = true; // Indicate that the robot is moving
            driveTo.setInitState(pose);

            while (pose.moving) {
                try {
                    pose = driveTo.move(pose, new MotionState(), deltaT);
                } catch (IllegalStateException e) {
                    System.out.println("Error setting path from pose: " + e.getMessage());
                    break;
                }

                updateLog();
            }
        }
    }*/
}