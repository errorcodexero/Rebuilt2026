package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

public class HopperConstants {
  // CAN IDs
  public static final int AGITATOR_LEADER_CAN_ID = 40; // TODO: Set actual CAN ID
  public static final int AGITATOR_FOLLOWER_CAN_ID = 41; // TODO: Set actual CAN ID
  public static final int FEEDER_MOTOR_CAN_ID = 42; // TODO: Set actual CAN ID

  // Gear Ratios
  public static final double AGITATOR_GEAR_RATIO = 3.0; // Motor rotations per agitator wheel rotation - TODO: Set actual ratio
  public static final double FEEDER_GEAR_RATIO = 2.0; // Motor rotations per feeder mechanism rotation - TODO: Set actual ratio

  // Agitator Motor PID (for velocity control)
  public static final double AGITATOR_KP = 0.15; // TODO: Tune these values
  public static final double AGITATOR_KI = 0.0;
  public static final double AGITATOR_KD = 0.0;
  public static final double AGITATOR_KS = 0.05; // Static friction feedforward
  public static final double AGITATOR_KV = 0.12; // Velocity feedforward
  public static final double AGITATOR_KA = 0.01; // Acceleration feedforward

  // Feeder Motor PID (for velocity control)
  public static final double FEEDER_KP = 0.2; // TODO: Tune these values
  public static final double FEEDER_KI = 0.0;
  public static final double FEEDER_KD = 0.0;
  public static final double FEEDER_KS = 0.05; // Static friction feedforward
  public static final double FEEDER_KV = 0.12; // Velocity feedforward
  public static final double FEEDER_KA = 0.01; // Acceleration feedforward

  // Agitator Constraints
  public static final AngularVelocity AGITATOR_MAX_VELOCITY = RotationsPerSecond.of(50.0); // TODO: Tune
  public static final AngularVelocity AGITATOR_IDLE_VELOCITY = RotationsPerSecond.of(5.0); // Slow agitation
  public static final AngularVelocity AGITATOR_ACTIVE_VELOCITY = RotationsPerSecond.of(20.0); // Active agitation
  
  // Feeder Constraints
  public static final AngularVelocity FEEDER_MAX_VELOCITY = RotationsPerSecond.of(80.0); // TODO: Tune
  public static final AngularVelocity FEEDER_FEED_VELOCITY = RotationsPerSecond.of(40.0); // Feed to shooter
  public static final AngularVelocity FEEDER_SLOW_FEED_VELOCITY = RotationsPerSecond.of(15.0); // Slow feed
  public static final AngularVelocity FEEDER_REVERSE_VELOCITY = RotationsPerSecond.of(-20.0); // Unjam
  
  // Current Limits
  public static final Current AGITATOR_CURRENT_LIMIT = Amps.of(40.0);
  public static final Current FEEDER_CURRENT_LIMIT = Amps.of(40.0);

  // Tolerance
  public static final AngularVelocity AGITATOR_TOLERANCE = RotationsPerSecond.of(2.0);
  public static final AngularVelocity FEEDER_TOLERANCE = RotationsPerSecond.of(2.0);

  // Simulation
  public static final MomentOfInertia AGITATOR_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.005); // TODO: Calculate actual value
  public static final MomentOfInertia FEEDER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.003); // TODO: Calculate actual value
}
