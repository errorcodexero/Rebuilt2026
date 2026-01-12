package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Replay implementation of HopperIO for log replay.
 * This class reads logged data and provides it to the subsystem.
 */
public class HopperIOReplay implements HopperIO {
  
  public HopperIOReplay() {
    // No initialization needed for replay
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Inputs are automatically updated from the log by AdvantageKit
  }

  @Override
  public void setAgitatorVelocity(AngularVelocity velocity) {
    // No-op for replay
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    // No-op for replay
  }

  @Override
  public void stopAgitators() {
    // No-op for replay
  }

  @Override
  public void stopFeeder() {
    // No-op for replay
  }

  @Override
  public void setAgitatorBrake(boolean brake) {
    // No-op for replay
  }

  @Override
  public void setFeederBrake(boolean brake) {
    // No-op for replay
  }
}
