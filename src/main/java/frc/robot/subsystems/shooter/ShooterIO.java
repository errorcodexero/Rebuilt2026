package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Flywheel Motors
    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0);
    public Voltage flywheelAppliedVoltage = Volts.of(0.0); // Leader voltage only
    public Current flywheelTotalCurrent = Amps.of(0.0); // Sum of all motor currents
    public double flywheelMaxTemperature = 0.0; // Maximum temperature across all motors (Celsius)
    public Current flywheelCurrent0 = Amps.of(0.0);
    public Current flywheelCurrent1 = Amps.of(0.0);
    public Current flywheelCurrent2 = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the flywheel motors to a target velocity */
  public default void setFlywheelVelocity(AngularVelocity velocity) {}

  /** Set the flywheel motors to a specific voltage */
  public default void setFlywheelVoltage(Voltage voltage) {}

  /** Stop the flywheel motors */
  public default void stopFlywheels() {}

  /** Set the flywheel motors to brake mode */
  public default void setFlywheelBrake(boolean brake) {}
}
