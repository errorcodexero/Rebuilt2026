package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
  // CAN IDs
  public static final int FLYWHEEL_LEADER_CAN_ID = 30; // TODO: Set actual CAN ID
  public static final int FLYWHEEL_FOLLOWER_CAN_ID = 31; // TODO: Set actual CAN ID
  public static final int HOOD_MOTOR_CAN_ID = 32; // TODO: Set actual CAN ID

  // Gear Ratios
  public static final double FLYWHEEL_GEAR_RATIO = 1.5; // Motor rotations per flywheel rotation - TODO: Set actual ratio
  public static final double HOOD_GEAR_RATIO = 8.0; // Motor rotations per hood rotation (placeholder)

  // Physical Constraints - Hood Motor
  public static final Angle HOOD_MIN_ANGLE = Degrees.of(0.0);
  public static final Angle HOOD_MAX_ANGLE = Degrees.of(60.0);
  public static final Angle HOOD_INITIAL_ANGLE = Degrees.of(30.0); // Starting position on robot init
  
  // Hood Position Setpoints
  public static final Angle HOOD_STOWED_ANGLE = Degrees.of(0.0);
  public static final Angle HOOD_LOW_ANGLE = Degrees.of(20.0);
  public static final Angle HOOD_MID_ANGLE = Degrees.of(35.0);
  public static final Angle HOOD_HIGH_ANGLE = Degrees.of(50.0);

  // Hood Motor PID
  public static final double HOOD_KP = 20.0; // TODO: Tune these values
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.5;
  public static final double HOOD_KS = 0.1; // Static friction feedforward
  public static final double HOOD_KV = 0.0; // Velocity feedforward
  public static final double HOOD_KA = 0.0; // Acceleration feedforward
  public static final double HOOD_KG = 0.25; // Gravity feedforward

  // Motion Magic - Hood Motor
  public static final AngularVelocity HOOD_CRUISE_VELOCITY = RotationsPerSecond.of(2.0); // TODO: Tune
  public static final AngularAcceleration HOOD_MAX_ACCELERATION = RotationsPerSecond.per(Second).of(4.0); // TODO: Tune
  public static final double HOOD_MAX_JERK = 40.0; // Rotations per second^3 - TODO: Tune
  
  // Flywheel Motor PID (for velocity control)
  public static final double FLYWHEEL_KP = 0.0; // TODO: Tune these values
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0;
  public static final double FLYWHEEL_KS = 0.05; // Static friction feedforward
  public static final double FLYWHEEL_KV = 0.12; // Velocity feedforward
  public static final double FLYWHEEL_KA = 0.01; // Acceleration feedforward

  // Flywheel Constraints
  public static final AngularVelocity FLYWHEEL_MAX_VELOCITY = RotationsPerSecond.of(100.0); // TODO: Tune
  public static final AngularVelocity FLYWHEEL_IDLE_VELOCITY = RotationsPerSecond.of(10.0); // Keep spinning slowly
  public static final AngularVelocity FLYWHEEL_LOW_SHOT_VELOCITY = RotationsPerSecond.of(40.0);
  public static final AngularVelocity FLYWHEEL_MID_SHOT_VELOCITY = RotationsPerSecond.of(60.0);
  public static final AngularVelocity FLYWHEEL_HIGH_SHOT_VELOCITY = RotationsPerSecond.of(80.0);
  
  // Current Limits
  public static final Current HOOD_CURRENT_LIMIT = Amps.of(40.0);
  public static final Current FLYWHEEL_CURRENT_LIMIT = Amps.of(60.0);

  // Tolerance
  public static final Angle HOOD_TOLERANCE = Degrees.of(1.0);
  public static final AngularVelocity FLYWHEEL_TOLERANCE = RotationsPerSecond.of(2.0);

  // Simulation
  public static final MomentOfInertia HOOD_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.08); // TODO: Calculate actual value
  public static final MomentOfInertia FLYWHEEL_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.01); // TODO: Calculate actual value
}
