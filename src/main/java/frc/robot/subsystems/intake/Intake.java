package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private AngularVelocity spinnerGoal = RotationsPerSecond.of(0.0);

  /** Creates a new Intake subsystem. */
  public Intake(IntakeIO io) {
    this.io = io;
    
    // Set brake modes
    io.setDeployBrake(true);
    io.setSpinnerBrake(false);
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Intake/SpinnerGoalRPS", spinnerGoal.in(RotationsPerSecond));
    Logger.recordOutput("Intake/SpinnerAtGoal", isSpinnerAtGoal());
    Logger.processInputs("Intake", inputs);
  }


  /**
   * Set the spinner velocity.
   * @param velocity The target velocity for the spinner (positive = intake)
   */
  public void setSpinnerVelocity(AngularVelocity velocity) {
    spinnerGoal = velocity;
    io.setSpinnerVelocity(velocity);
  }

  /**
   * Set the spinner motor voltage directly (for SysId characterization).
   * @param voltage The voltage to apply to the spinner motor
   */
  public void setSpinnerVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    spinnerGoal = RotationsPerSecond.of(0.0); // Clear velocity goal when using voltage control
    io.setSpinnerVoltage(voltage);
  }

  /**
   * Stop the spinner motor.
   */
  public void stopSpinner() {
    spinnerGoal = RotationsPerSecond.of(0.0);
    io.stopSpinner();
  }

  /**
   * Run the intake at the default intake velocity.
   */
  public void intake() {
    setSpinnerVelocity(IntakeConstants.SPINNER_INTAKE_VELOCITY);
  }

  /**
   * Run the intake at the default eject velocity.
   */
  public void eject() {
    setSpinnerVelocity(IntakeConstants.SPINNER_EJECT_VELOCITY);
  }

  /**
   * Get the current spinner velocity.
   * @return Current spinner velocity
   */
  public AngularVelocity getSpinnerVelocity() {
    return inputs.spinnerVelocity;
  }

  /**
   * Check if the spinner is at the goal velocity.
   * @return True if at goal within tolerance
   */
  public boolean isSpinnerAtGoal() {
    return Math.abs(
        inputs.spinnerVelocity.in(RotationsPerSecond) - spinnerGoal.in(RotationsPerSecond)
    ) < IntakeConstants.SPINNER_TOLERANCE.in(RotationsPerSecond);
  }

  /**
   * Get the spinner motor current.
   * @return Spinner motor current in amps
   */
  public double getSpinnerCurrent() {
    return inputs.spinnerCurrent.in(Amps);
  }

  /**
   * Reset the deploy position to a known angle.
   * Call this when the robot is at a known position (e.g., after homing).
   * @param currentAngle The current known angle
   */
  public void resetDeployPosition(Angle currentAngle) {
    io.resetDeployPosition(currentAngle);
  }
}
