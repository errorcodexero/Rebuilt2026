package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Deploy Motor
    public Angle deployPosition = IntakeConstants.DEPLOY_INITIAL_ANGLE;
    public AngularVelocity deployVelocity = IntakeConstants.DEPLOY_CRUISE_VELOCITY.times(0.0);
    public Voltage deployAppliedVoltage = Volts.of(0.0);
    public Current deployCurrent = Amps.of(0.0);
    public double deployTemperature = 0.0; // Celsius

    // Spinner Motor
    public AngularVelocity spinnerVelocity = IntakeConstants.SPINNER_MAX_VELOCITY.times(0.0);
    public Voltage spinnerAppliedVoltage = Volts.of(0.0);
    public Current spinnerCurrent = Amps.of(0.0);
    public double spinnerTemperature = 0.0; // Celsius
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the deploy motor to a target position using Motion Magic */
  public default void setDeployPosition(Angle position) {}

  /** Set the spinner motor to a target velocity */
  public default void setSpinnerVelocity(AngularVelocity velocity) {}

  /** Stop the deploy motor */
  public default void stopDeploy() {}

  /** Stop the spinner motor */
  public default void stopSpinner() {}

  /** Set the deploy motor to brake mode */
  public default void setDeployBrake(boolean brake) {}

  /** Set the spinner motor to brake mode */
  public default void setSpinnerBrake(boolean brake) {}

  /** Reset the deploy position to the current angle (call on robot init) */
  public default void resetDeployPosition(Angle currentAngle) {}
}
