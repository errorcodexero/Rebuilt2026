package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private AngularVelocity agitatorGoal = RotationsPerSecond.of(0.0);
  private AngularVelocity feederGoal = RotationsPerSecond.of(0.0);

  /** Creates a new Hopper subsystem. */
  public Hopper(HopperIO io) {
    this.io = io;
    
    // Set brake modes
    io.setAgitatorBrake(false); // Coast for agitators
    io.setFeederBrake(true); // Brake for feeder
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Hopper/AgitatorGoalRPS", agitatorGoal.in(RotationsPerSecond));
    Logger.recordOutput("Hopper/FeederGoalRPS", feederGoal.in(RotationsPerSecond));
    Logger.recordOutput("Hopper/AgitatorAtGoal", isAgitatorAtGoal());
    Logger.recordOutput("Hopper/FeederAtGoal", isFeederAtGoal());
    Logger.processInputs("Hopper", inputs);
  }

  /**
   * Set the agitator velocity.
   * @param velocity The target velocity for the agitators
   */
  public void setAgitatorVelocity(AngularVelocity velocity) {
    agitatorGoal = velocity;
    io.setAgitatorVelocity(velocity);
  }

  /**
   * Set the agitator motors voltage directly (for SysId characterization).
   * @param voltage The voltage to apply to the agitator motors
   */
  public void setAgitatorVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    agitatorGoal = RotationsPerSecond.of(0.0); // Clear velocity goal when using voltage control
    io.setAgitatorVoltage(voltage);
  }

  /**
   * Set the feeder velocity.
   * @param velocity The target velocity for the feeder
   */
  public void setFeederVelocity(AngularVelocity velocity) {
    feederGoal = velocity;
    io.setFeederVelocity(velocity);
  }

  /**
   * Set the feeder motor voltage directly (for SysId characterization).
   * @param voltage The voltage to apply to the feeder motor
   */
  public void setFeederVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    feederGoal = RotationsPerSecond.of(0.0); // Clear velocity goal when using voltage control
    io.setFeederVoltage(voltage);
  }

  /**
   * Run agitators at idle speed.
   */
  public void idleAgitators() {
    setAgitatorVelocity(HopperConstants.AGITATOR_IDLE_VELOCITY);
  }

  /**
   * Run agitators at active speed.
   */
  public void activeAgitators() {
    setAgitatorVelocity(HopperConstants.AGITATOR_ACTIVE_VELOCITY);
  }

  /**
   * Stop the agitator motors.
   */
  public void stopAgitators() {
    agitatorGoal = RotationsPerSecond.of(0.0);
    io.stopAgitators();
  }

  /**
   * Stop the feeder motor.
   */
  public void stopFeeder() {
    feederGoal = RotationsPerSecond.of(0.0);
    io.stopFeeder();
  }

  /**
   * Stop all hopper motors.
   */
  public void stopAll() {
    stopAgitators();
    stopFeeder();
  }

  /**
   * Feed balls to shooter at normal speed.
   */
  public void feed() {
    setFeederVelocity(HopperConstants.FEEDER_FEED_VELOCITY);
  }

  /**
   * Feed balls to shooter at slow speed.
   */
  public void feedSlow() {
    setFeederVelocity(HopperConstants.FEEDER_SLOW_FEED_VELOCITY);
  }

  /**
   * Reverse feeder to unjam.
   */
  public void reverseFeeder() {
    setFeederVelocity(HopperConstants.FEEDER_REVERSE_VELOCITY);
  }

  /**
   * Agitate and feed balls simultaneously.
   */
  public void agitateAndFeed() {
    activeAgitators();
    feed();
  }

  /**
   * Get the current agitator velocity.
   * @return Current agitator velocity
   */
  public AngularVelocity getAgitatorVelocity() {
    return inputs.agitatorVelocity;
  }

  /**
   * Get the current feeder velocity.
   * @return Current feeder velocity
   */
  public AngularVelocity getFeederVelocity() {
    return inputs.feederVelocity;
  }

  /**
   * Check if the agitators are at the goal velocity.
   * @return True if at goal within tolerance
   */
  public boolean isAgitatorAtGoal() {
    return Math.abs(
        inputs.agitatorVelocity.in(RotationsPerSecond) - agitatorGoal.in(RotationsPerSecond)
    ) < HopperConstants.AGITATOR_TOLERANCE.in(RotationsPerSecond);
  }

  /**
   * Check if the feeder is at the goal velocity.
   * @return True if at goal within tolerance
   */
  public boolean isFeederAtGoal() {
    return Math.abs(
        inputs.feederVelocity.in(RotationsPerSecond) - feederGoal.in(RotationsPerSecond)
    ) < HopperConstants.FEEDER_TOLERANCE.in(RotationsPerSecond);
  }

  /**
   * Check if agitators are running.
   * @return True if agitators are spinning
   */
  public boolean isAgitating() {
    return Math.abs(inputs.agitatorVelocity.in(RotationsPerSecond)) > 1.0;
  }

  /**
   * Check if feeder is running.
   * @return True if feeder is spinning
   */
  public boolean isFeeding() {
    return inputs.feederVelocity.in(RotationsPerSecond) > 1.0;
  }

  /**
   * Get the agitator leader motor current.
   * @return Agitator leader motor current in amps
   */
  public double getAgitatorLeaderCurrent() {
    return inputs.agitatorLeaderCurrent.in(Amps);
  }

  /**
   * Get the agitator follower motor current.
   * @return Agitator follower motor current in amps
   */
  public double getAgitatorFollowerCurrent() {
    return inputs.agitatorFollowerCurrent.in(Amps);
  }

  /**
   * Get the feeder motor current.
   * @return Feeder motor current in amps
   */
  public double getFeederCurrent() {
    return inputs.feederCurrent.in(Amps);
  }

  /**
   * Get the total agitator current (leader + follower).
   * @return Total agitator current in amps
   */
  public double getTotalAgitatorCurrent() {
    return getAgitatorLeaderCurrent() + getAgitatorFollowerCurrent();
  }
}
