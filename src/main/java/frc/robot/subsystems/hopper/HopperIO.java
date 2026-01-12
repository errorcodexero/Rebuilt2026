package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    // Agitator Motors (Leader + Follower)
    public AngularVelocity agitatorVelocity = HopperConstants.AGITATOR_MAX_VELOCITY.times(0.0);
    public Voltage agitatorLeaderAppliedVoltage = Volts.of(0.0);
    public Voltage agitatorFollowerAppliedVoltage = Volts.of(0.0);
    public Current agitatorLeaderCurrent = Amps.of(0.0);
    public Current agitatorFollowerCurrent = Amps.of(0.0);
    public double agitatorLeaderTemperature = 0.0; // Celsius
    public double agitatorFollowerTemperature = 0.0; // Celsius

    // Feeder Motor
    public AngularVelocity feederVelocity = HopperConstants.FEEDER_MAX_VELOCITY.times(0.0);
    public Voltage feederAppliedVoltage = Volts.of(0.0);
    public Current feederCurrent = Amps.of(0.0);
    public double feederTemperature = 0.0; // Celsius
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Set the agitator motors to a target velocity */
  public default void setAgitatorVelocity(AngularVelocity velocity) {}

  /** Set the feeder motor to a target velocity */
  public default void setFeederVelocity(AngularVelocity velocity) {}

  /** Stop the agitator motors */
  public default void stopAgitators() {}

  /** Stop the feeder motor */
  public default void stopFeeder() {}

  /** Set the agitator motors to brake mode */
  public default void setAgitatorBrake(boolean brake) {}

  /** Set the feeder motor to brake mode */
  public default void setFeederBrake(boolean brake) {}
}
