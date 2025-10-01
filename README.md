# SKIDADDLE

## Description
**SKIDADDLE** is a physics-based, real-time dynamic pathing algorithm for FTC robotics written in Java.  
It was designed to address inefficiencies in Road Runner and generate more optimal paths.  

**Key Features:**
- Supports cubic Bézier curves for the autonomous period.  
- Uses Hermite splines to dynamically generate paths for Tele-Op.  
- Provides flexible motor control: pass in lambda functions for direct control, or simply retrieve the target state.  
- Supports lambda functions for orientation specification, while constraining to constants.  
- Uses a physics model for more accurate pathing than other algorithms like Road Runner.  

---

## Setup
SKIDADDLE is entirely library-based. To get started, clone this repository into your project, and import the library and use the provided methods. Then you will have to tune the inch per tick and PID values for SKIDADDLE to work properly.

You can use SKIDADDLE in two ways:
1. **Direct control:** Retrieve the linear/angular position, velocity, and acceleration, then feed them into your own PID system.  
2. **Lambda function control:** Pass SKIDADDLE a lambda function that updates the motors with normalized wheel values.  

### Example: Lambda Function for FTC
```java
Controller.MotorController motorLambda = (WheelSpeed s) -> {
    double v = voltageSensor.getVoltage(); //This normalizes to the motor voltage.
    
    your_front_left_motor.setPower(s.vels[s.FL]  / v);
    your_front_right_motor.setPower(s.vels[s.FR] / v);
    your_back_left_motor.setPower(s.vels[s.BL]   / v);
    your_back_right_motor.setPower(s.vels[s.BR]  / v);
};
```

## Usage Examples
### Autonomous with Bézier Curves  
This is how to write a Bézier-based routine for the Autonomous period.

```java
  Controller driveTo = new Controller(null, motorLambda); // instantiate pathing object. If not using the motor lambda, set it to null
  Angular.Controller angController = your_angular_lambda; // e.g., Angular.turnToAngle(rad)
  Path path = new Path(new Path.Bezier(control_points)); // vector array of four control points (length must be 4)

  MotionState pose = new MotionState(your_initial_linear_PoseVelAcc, your_initial_angular_PoseVelAcc); //Initialize pose to the starting position, velocity and acceleration of the robot.
  
  while (pose.moving) { 
    MotionState sensorState = new MotionState(
      new PoseVelAcc(current_linear_position, current_linear_velocity, current_linear_acceleration),
      new PoseVelAcc(current_angle_vector, current_angular_velocity, current_angular_acceleration)
    ); // get from localization data
  
    pose = driveTo.update(pose, sensorState, path, angController, elapsed_time_from_last_loop_secs);
  
    try {
      Thread.sleep(10); 
    } catch (InterruptedException e) {}
  }
```

### Tele-Op with Hermite Splines
This is how to write a Hermite-based routine for Tele-Op.  

```java
  Controller driveTo = new Controller(null, lambda);
  
  Angular.Controller angController = your_angular_lambda;
  Path path = new Path(new Path.Hermite(
      your_start_point, your_desired_end_point, 
      your_start_velocity, your_desired_exit_velocity
  ));

  MotionState pose = new MotionState(your_initial_linear_PoseVelAcc, your_initial_angular_PoseVelAcc); //Initialize pose to the starting position, velocity and acceleration of the robot.
  
  while (pose.moving) { 
    MotionState sensorState = new MotionState(
      new PoseVelAcc(current_linear_position, current_linear_velocity, current_linear_acceleration),
      new PoseVelAcc(current_angle_vector, current_angular_velocity, current_angular_acceleration)
    ); 
  
    pose = driveTo.update(pose, sensorState, path, angController, elapsed_time_from_last_loop_secs);
  
    try {
      Thread.sleep(10); 
    } catch (InterruptedException e) {}
  }
```

## References

### Path
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `Path(Vector[] controlPoints)` | `controlPoints`: an array of 4 vectors | `Path` | Creates a path directly from a set of Bézier control points. |
| `Path(Bezier b)` | `b`: a Bézier object | `Path` | Creates a path from a Bézier curve. |
| `Path(Hermite h)` | `h`: a Hermite spline | `Path` | Converts a Hermite spline into an optimal Bézier curve for pathing. |

**Bézier**  
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `Bezier(Vector[] controlPoints)` | `controlPoints`: array of 4 vectors | `Bezier` | Defines a Bézier curve from four control points. |

**Hermite**  
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `Hermite(Vector start, Vector end, Vector startVel, Vector endVel)` | `start`: starting point<br>`end`: target point<br>`startVel`: starting velocity vector<br>`endVel`: desired exit velocity vector | `Hermite` | Defines a Hermite spline that smoothly connects a start and end state. |

---

### Angular
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `Angular.turnToAngle(double theta)` | `theta`: desired heading in radians | `Angular.Controller` (lambda) | Returns a default angular controller for turning toward a given target heading. |

---

### Controller
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `Controller(Graph draw, MotorController motor)` | `draw`: optional graphing utility (nullable)<br>`motor`: motor control lambda (nullable) | `Controller` | Creates a controller. If `draw` or `motor` are `null`, those behaviors are skipped. |
| `setInitState(MotionState m)` | `m`: initial linear & angular position, velocity, and acceleration | `void` | Sets the initial state of the controller before motion begins. |
| `update(MotionState motion, MotionState sensorMotion, Path path, Angular.Controller angle, double deltaT)` | `motion`: the last commanded state<br>`sensorMotion`: the current measured state (from localization)<br>`path`: target path<br>`angle`: angular controller lambda<br>`deltaT`: timestep in seconds | `MotionState` (the new commanded state) | Computes the next motion state based on path following and angular control. If a motor lambda was provided, updates the motors automatically. |

---

### MotionState
| Field | Type | Description |
|-------|------|-------------|
| `linear` | `PoseVelAcc` | Linear position, velocity, acceleration. |
| `angular` | `PoseVelAcc` | Angular position, velocity, acceleration. Stored as Vectors |

**PoseVelAcc**  
| Field | Type | Description |
|-------|------|-------------|
| `pos` | `Vector` | Position vector. |
| `vel` | `Vector` | Velocity vector. |
| `acc` | `Vector` | Acceleration vector. |

**Vector**  
| Field | Type | Description |
|-------|------|-------------|
| `x` | `double` | x component. |
| `y` | `double` | y component. |

## Technical Explanation
In short: SKIDADDLE continuously decomposes robot velocity into convergent and transverse components, prioritizing correction toward the path while dynamically planning lookahead transitions.  

### Linear Pathing
The best way to explain the algorithm is to start with how it handles a **single path segment**.  
It first generates a standard motion profile to a target exit velocity. From this, it computes the target velocity.  

The current velocity is then decomposed into two components:  
- **Convergent velocity**: the portion of velocity directed toward the goal (positive tangential).  
- **Transverse velocity**: the portion not directed toward the goal (negative tangential or normal).  

The algorithm prioritizes decelerating transverse velocity to zero (to stay aligned with the path). Any remaining acceleration budget is then applied to increasing the convergent velocity. This guarantees that, regardless of the starting position or velocity, the robot will always converge toward the goal.

---

### Segment Transitions
Next, we extend the logic to handle **transitions between segments**.  

To do this, the algorithm calculates the distance required to decelerate transverse velocity. However, because convergent velocity causes forward progress along the path, the eventual intersection point with the new line shifts during the deceleration. The algorithm accounts for this displacement and executes the transition once the robot reaches the calculated distance.

---

### Generating Path Segments
With segment handling defined, the next question is: **how are the line segments generated?**  

- A Hermite spline is created, defined by the robot’s current position/velocity and the desired end position/velocity.  
- The Hermite spline is then converted into an optimal cubic Bézier curve by scaling the curvature down if the acceleration constant is exceeded.
- For autonomous use, Béziers can also be used directly. The algorithm estimates the required acceleration for a cubic Bézier curve and scales its curvature if the acceleration limit is exceeded.  
- Cubic Bézier curves are flattened into line segments using **recursive De Casteljau subdivision with adaptive error tolerance**, yielding more precise approximations than uniform linear sampling.  

---

### Lookahead and Velocity Planning
Finally, the algorithm estimates the distance required to decelerate to a full stop and performs a **lookahead** over that distance.  

During lookahead:  
1. Each vertex is checked to determine the maximum feasible velocity for making the transition.  
2. This velocity is scaled based on both the maximum speed of the upcoming transition and the available distance.  

This lookahead step is the most computationally intensive part of the algorithm.  

---

### Orientation
Orientation is controlled via a user-provided lambda function, which receives the current `MotionState` and timestep.  
This enables features like point tracking. Orientation changes are capped at the robot’s max angular velocity/acceleration, ensuring constraints are never exceeded. Orientation, angular velocity, and angular acceleration are all treated as vectors.

---

### Disassembly
Converting from field-relative velocity to wheel velocities occurs in four steps:

1. The `PoseVelAcc` is updated using a PIDF controller, which minimizes error and applies the desired velocity/acceleration.  
2. Updated vectors are converted from field-relative to robot-relative coordinates using the current orientation.  
3. Linear/angular velocity and acceleration are passed through an inverse kinematics equation, producing per-wheel velocities.
4. A final feedforward adjustment compensates for motor-control imperfections when setting wheel velocity and acceleration.

Finally, the per-wheel velocities are passed to the user’s lambda, where the user can normalize to motor voltage and apply them.

---
   
### Putting It All Together
The resulting system ensures that:  
- The motion profile reaches the maximum safe velocity for the next segment before the transition.  
- The transition occurs exactly when the remaining distance is less than the required deceleration distance.  

By combining motion profiling, velocity decomposition, adaptive curve flattening, predictive lookahead, and PID control, SKIDADDLE produces smooth, dynamically feasible robot paths in real time.

## License
This project is licensed under the Apache License 2.0 – see the [LICENSE](LICENSE) file for details.
