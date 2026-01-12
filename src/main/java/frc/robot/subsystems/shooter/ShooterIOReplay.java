package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Replay implementation of ShooterIO for log replay.
 * This class reads logged data and provides it to the subsystem.
 */
public class ShooterIOReplay implements ShooterIO {
  
  public ShooterIOReplay() {
    // No initialization needed for replay
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Inputs are automatically updated from the log by AdvantageKit
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    // No-op for replay
  }

  @Override
  public void setHoodPosition(Angle position) {
    // No-op for replay
  }

  @Override
  public void stopFlywheels() {
    // No-op for replay
  }

  @Override
  public void stopHood() {
    // No-op for replay
  }

  @Override
  public void setFlywheelBrake(boolean brake) {
    // No-op for replay
  }

  @Override
  public void setHoodBrake(boolean brake) {
    // No-op for replay
  }

  @Override
  public void resetHoodPosition(Angle currentAngle) {
    // No-op for replay
  }
}
