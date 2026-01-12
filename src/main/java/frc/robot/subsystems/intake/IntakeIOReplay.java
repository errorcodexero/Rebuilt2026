package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Replay implementation of IntakeIO for log replay.
 * This class reads logged data and provides it to the subsystem.
 */
public class IntakeIOReplay implements IntakeIO {
  
  public IntakeIOReplay() {
    // No initialization needed for replay
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Inputs are automatically updated from the log by AdvantageKit
  }

  @Override
  public void setDeployPosition(Angle position) {
    // No-op for replay
  }

  @Override
  public void setSpinnerVelocity(AngularVelocity velocity) {
    // No-op for replay
  }

  @Override
  public void stopDeploy() {
    // No-op for replay
  }

  @Override
  public void stopSpinner() {
    // No-op for replay
  }

  @Override
  public void setDeployBrake(boolean brake) {
    // No-op for replay
  }

  @Override
  public void setSpinnerBrake(boolean brake) {
    // No-op for replay
  }

  @Override
  public void resetDeployPosition(Angle currentAngle) {
    // No-op for replay
  }
}
