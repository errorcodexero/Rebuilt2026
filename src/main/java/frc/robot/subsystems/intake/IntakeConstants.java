package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

public class IntakeConstants {
  // CAN IDs
  public static final int SPINNER_MOTOR_CAN_ID = 1; // TODO: Set actual CAN ID

  // Gear Ratios
  public static final double DEPLOY_GEAR_RATIO = 13.0; // Motor rotations per intake rotation
  public static final double SPINNER_GEAR_RATIO = 2.0; // Motor rotations per spinner wheel rotation

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
  public static final Current SPINNER_CURRENT_LIMIT = Amps.of(40.0);

  // Tolerance
  public static final AngularVelocity SPINNER_TOLERANCE = RotationsPerSecond.of(2.0);

  // Simulation
  public static final MomentOfInertia SPINNER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.001); // TODO: Calculate actual value
}
