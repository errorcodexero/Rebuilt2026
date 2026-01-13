package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private AngularVelocity flywheelGoal = RotationsPerSecond.of(0.0);
  private Angle hoodGoal = ShooterConstants.HOOD_INITIAL_ANGLE;

  /** Creates a new Shooter subsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
    
    // Reset hood position to initial angle on construction
    io.resetHoodPosition(ShooterConstants.HOOD_INITIAL_ANGLE);
    
    // Set brake modes
    io.setFlywheelBrake(false); // Coast for flywheels
    io.setHoodBrake(true); // Brake for hood
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Shooter/FlywheelGoalRPS", flywheelGoal.in(RotationsPerSecond));
    Logger.recordOutput("Shooter/HoodGoalDegrees", hoodGoal.in(Degrees));
    Logger.recordOutput("Shooter/FlywheelAtGoal", isFlywheelAtGoal());
    Logger.recordOutput("Shooter/HoodAtGoal", isHoodAtGoal());
    Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * Set the flywheel velocity.
   * @param velocity The target velocity for the flywheels
   */
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelGoal = velocity;
    io.setFlywheelVelocity(velocity);
  }

  /**
   * Set the flywheel voltage directly (open-loop control).
   * @param voltage The voltage to apply to the flywheel motors
   */
  public void setFlywheelVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    flywheelGoal = RotationsPerSecond.of(0.0); // Clear velocity goal when using voltage control
    io.setFlywheelVoltage(voltage);
  }

  /**
   * Set the hood position.
   * @param angle The target angle for the hood
   */
  public void setHoodPosition(Angle angle) {
    hoodGoal = angle;
    io.setHoodPosition(angle);
  }

  /**
   * Set the hood motor voltage directly (for SysId characterization).
   * @param voltage The voltage to apply to the hood motor
   */
  public void setHoodVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    hoodGoal = Degrees.of(0.0); // Clear position goal when using voltage control
    io.setHoodVoltage(voltage);
  }

  /**
   * Spin up flywheels to idle speed.
   */
  public void spinToIdle() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_IDLE_VELOCITY);
  }

  /**
   * Stop the flywheel motors.
   */
  public void stopFlywheels() {
    flywheelGoal = RotationsPerSecond.of(0.0);
    io.stopFlywheels();
  }

  /**
   * Stop the hood motor.
   */
  public void stopHood() {
    io.stopHood();
  }

  /**
   * Set shooter to low shot configuration.
   */
  public void setLowShot() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_LOW_SHOT_VELOCITY);
    setHoodPosition(ShooterConstants.HOOD_LOW_ANGLE);
  }

  /**
   * Set shooter to mid shot configuration.
   */
  public void setMidShot() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_MID_SHOT_VELOCITY);
    setHoodPosition(ShooterConstants.HOOD_MID_ANGLE);
  }

  /**
   * Set shooter to high shot configuration.
   */
  public void setHighShot() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_HIGH_SHOT_VELOCITY);
    setHoodPosition(ShooterConstants.HOOD_HIGH_ANGLE);
  }

  /**
   * Stow the hood to the stowed position.
   */
  public void stowHood() {
    setHoodPosition(ShooterConstants.HOOD_STOWED_ANGLE);
  }

  /**
   * Get the current flywheel velocity.
   * @return Current flywheel velocity
   */
  public AngularVelocity getFlywheelVelocity() {
    return inputs.flywheelVelocity;
  }

  /**
   * Get the current hood position.
   * @return Current hood position
   */
  public Angle getHoodPosition() {
    return inputs.hoodPosition;
  }

  /**
   * Get the current hood velocity.
   * @return Current hood velocity
   */
  public AngularVelocity getHoodVelocity() {
    return inputs.hoodVelocity;
  }

  /**
   * Check if the flywheels are at the goal velocity.
   * @return True if at goal within tolerance
   */
  public boolean isFlywheelAtGoal() {
    return Math.abs(
        inputs.flywheelVelocity.in(RotationsPerSecond) - flywheelGoal.in(RotationsPerSecond)
    ) < ShooterConstants.FLYWHEEL_TOLERANCE.in(RotationsPerSecond);
  }

  /**
   * Check if the hood is at the goal position.
   * @return True if at goal within tolerance
   */
  public boolean isHoodAtGoal() {
    return Math.abs(
        inputs.hoodPosition.in(Degrees) - hoodGoal.in(Degrees)
    ) < ShooterConstants.HOOD_TOLERANCE.in(Degrees);
  }

  /**
   * Check if the shooter is ready to shoot (both flywheel and hood at goals).
   * @return True if ready to shoot
   */
  public boolean isReadyToShoot() {
    return isFlywheelAtGoal() && isHoodAtGoal() && 
           flywheelGoal.in(RotationsPerSecond) > ShooterConstants.FLYWHEEL_IDLE_VELOCITY.in(RotationsPerSecond);
  }

  /**
   * Check if the hood is stowed.
   * @return True if stowed
   */
  public boolean isHoodStowed() {
    return inputs.hoodPosition.in(Degrees) < 
        (ShooterConstants.HOOD_STOWED_ANGLE.in(Degrees) + 
         ShooterConstants.HOOD_TOLERANCE.in(Degrees));
  }

  /**
   * Get the flywheel leader motor current.
   * @return Flywheel leader motor current in amps
   */
  public double getFlywheelLeaderCurrent() {
    return inputs.flywheelLeaderCurrent.in(Amps);
  }

  /**
   * Get the flywheel follower motor current.
   * @return Flywheel follower motor current in amps
   */
  public double getFlywheelFollowerCurrent() {
    return inputs.flywheelFollowerCurrent.in(Amps);
  }

  /**
   * Get the hood motor current.
   * @return Hood motor current in amps
   */
  public double getHoodCurrent() {
    return inputs.hoodCurrent.in(Amps);
  }

  /**
   * Get the total flywheel current (leader + follower).
   * @return Total flywheel current in amps
   */
  public double getTotalFlywheelCurrent() {
    return getFlywheelLeaderCurrent() + getFlywheelFollowerCurrent();
  }

  /**
   * Reset the hood position to a known angle.
   * Call this when the robot is at a known position (e.g., after homing).
   * @param currentAngle The current known angle
   */
  public void resetHoodPosition(Angle currentAngle) {
    io.resetHoodPosition(currentAngle);
  }
}
