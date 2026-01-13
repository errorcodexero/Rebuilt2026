package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

public class IntakeConstants {
  // CAN IDs
  public static final int DEPLOY_MOTOR_CAN_ID = 20; // TODO: Set actual CAN ID
  public static final int SPINNER_MOTOR_CAN_ID = 21; // TODO: Set actual CAN ID

  // Gear Ratios
  public static final double DEPLOY_GEAR_RATIO = 13.0; // Motor rotations per intake rotation
  public static final double SPINNER_GEAR_RATIO = 2.0; // Motor rotations per spinner wheel rotation

  // Physical Constraints - Deploy Motor
  public static final Angle DEPLOY_MIN_ANGLE = Degrees.of(0.0);
  public static final Angle DEPLOY_MAX_ANGLE = Degrees.of(120.0);
  public static final Angle DEPLOY_INITIAL_ANGLE = Degrees.of(0.0); // Starting position on robot init
  
  // Deploy Position Setpoints
  public static final Angle DEPLOY_RETRACTED_ANGLE = Degrees.of(0.0);
  public static final Angle DEPLOY_DEPLOYED_ANGLE = Degrees.of(90.0);

  // Deploy Motor PID
  public static final double DEPLOY_KP = 20.0; // TODO: Tune these values
  public static final double DEPLOY_KI = 0.0;
  public static final double DEPLOY_KD = 0.5;
  public static final double DEPLOY_KS = 0.1; // Static friction feedforward
  public static final double DEPLOY_KV = 0.0; // Velocity feedforward
  public static final double DEPLOY_KA = 0.0; // Acceleration feedforward
  public static final double DEPLOY_KG = 0.3; // Gravity feedforward

  // Motion Magic - Deploy Motor
  public static final AngularVelocity DEPLOY_CRUISE_VELOCITY = RotationsPerSecond.of(2.0); // TODO: Tune
  public static final AngularAcceleration DEPLOY_MAX_ACCELERATION = RotationsPerSecond.per(Second).of(4.0); // TODO: Tune
  public static final double DEPLOY_MAX_JERK = 40.0; // Rotations per second^3 - TODO: Tune
  
  // Spinner Motor PID (for velocity control)
  public static final double SPINNER_KP = 0.1; // TODO: Tune these values
  public static final double SPINNER_KI = 0.0;
  public static final double SPINNER_KD = 0.0;
  public static final double SPINNER_KS = 0.05; // Static friction feedforward
  public static final double SPINNER_KV = 0.12; // Velocity feedforward
  public static final double SPINNER_KA = 0.01; // Acceleration feedforward

  // Spinner Motor Constraints
  public static final AngularVelocity SPINNER_MAX_VELOCITY = RotationsPerSecond.of(80.0); // TODO: Tune
  public static final AngularVelocity SPINNER_INTAKE_VELOCITY = RotationsPerSecond.of(40.0); // Intake game piece
  public static final AngularVelocity SPINNER_EJECT_VELOCITY = RotationsPerSecond.of(-30.0); // Eject game piece
  
  // Current Limits
  public static final Current DEPLOY_CURRENT_LIMIT = Amps.of(40.0);
  public static final Current SPINNER_CURRENT_LIMIT = Amps.of(40.0);

  // Tolerance
  public static final Angle DEPLOY_TOLERANCE = Degrees.of(2.0);
  public static final AngularVelocity SPINNER_TOLERANCE = RotationsPerSecond.of(2.0);

  // Simulation
  public static final MomentOfInertia DEPLOY_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.05); // TODO: Calculate actual value
  public static final MomentOfInertia SPINNER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.001); // TODO: Calculate actual value
}
