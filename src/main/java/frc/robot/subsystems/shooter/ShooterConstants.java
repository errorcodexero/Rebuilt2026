package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
  // CAN IDs - Array where [0] is leader, others are followers
  public static final int[] FLYWHEEL_MOTOR_CAN_IDS = {1, 4, 5}; // TODO: Set actual CAN IDs

  // Gear Ratios
  public static final double FLYWHEEL_GEAR_RATIO = 1.71;
  
  // Flywheel Motor PID (for velocity control)
  public static final double FLYWHEEL_KP = 0.0; // TODO: Tune these values
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0;
  public static final double FLYWHEEL_KS = 0.05; // Static friction feedforward
  public static final double FLYWHEEL_KV = 0.12; // Velocity feedforward
  public static final double FLYWHEEL_KA = 0.01; // Acceleration feedforward

  // Flywheel Constraints
  public static final AngularVelocity FLYWHEEL_MAX_VELOCITY = RotationsPerSecond.of(100.0);
  public static final AngularVelocity FLYWHEEL_IDLE_VELOCITY = RotationsPerSecond.of(10.0);
  public static final AngularVelocity FLYWHEEL_LOW_SHOT_VELOCITY = RotationsPerSecond.of(40.0);
  public static final AngularVelocity FLYWHEEL_MID_SHOT_VELOCITY = RotationsPerSecond.of(60.0);
  public static final AngularVelocity FLYWHEEL_HIGH_SHOT_VELOCITY = RotationsPerSecond.of(80.0);
  
  // Current Limits
  public static final Current FLYWHEEL_CURRENT_LIMIT = Amps.of(60.0);

  // Tolerance
  public static final AngularVelocity FLYWHEEL_TOLERANCE = RotationsPerSecond.of(2.0);

  // Simulation
  public static final MomentOfInertia FLYWHEEL_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.0001);
}
