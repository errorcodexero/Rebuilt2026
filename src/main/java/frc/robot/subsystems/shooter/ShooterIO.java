package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Flywheel Motors (Leader + Follower)
    public AngularVelocity flywheelVelocity = ShooterConstants.FLYWHEEL_MAX_VELOCITY.times(0.0);
    public Voltage flywheelLeaderAppliedVoltage = Volts.of(0.0);
    public Voltage flywheelFollowerAppliedVoltage = Volts.of(0.0);
    public Current flywheelLeaderCurrent = Amps.of(0.0);
    public Current flywheelFollowerCurrent = Amps.of(0.0);
    public double flywheelLeaderTemperature = 0.0; // Celsius
    public double flywheelFollowerTemperature = 0.0; // Celsius

    // Hood Motor
    public Angle hoodPosition = ShooterConstants.HOOD_INITIAL_ANGLE;
    public AngularVelocity hoodVelocity = ShooterConstants.HOOD_CRUISE_VELOCITY.times(0.0);
    public Voltage hoodAppliedVoltage = Volts.of(0.0);
    public Current hoodCurrent = Amps.of(0.0);
    public double hoodTemperature = 0.0; // Celsius
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the flywheel motors to a target velocity */
  public default void setFlywheelVelocity(AngularVelocity velocity) {}

  /** Set the hood motor to a target position using Motion Magic */
  public default void setHoodPosition(Angle position) {}

  /** Stop the flywheel motors */
  public default void stopFlywheels() {}

  /** Stop the hood motor */
  public default void stopHood() {}

  /** Set the flywheel motors to brake mode */
  public default void setFlywheelBrake(boolean brake) {}

  /** Set the hood motor to brake mode */
  public default void setHoodBrake(boolean brake) {}

  /** Reset the hood position to the current angle (call on robot init) */
  public default void resetHoodPosition(Angle currentAngle) {}
}
