package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private AngularVelocity flywheelGoal = RotationsPerSecond.of(0.0);

  /** Creates a new Shooter subsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
    
    // Set brake modes
    io.setFlywheelBrake(false); // Coast for flywheels
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Shooter/FlywheelGoalRPS", flywheelGoal.in(RotationsPerSecond));
    Logger.recordOutput("Shooter/FlywheelAtGoal", isFlywheelAtGoal());
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
   * Set shooter to low shot configuration.
   */
  public void setLowShot() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_LOW_SHOT_VELOCITY);
  }

  /**
   * Set shooter to mid shot configuration.
   */
  public void setMidShot() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_MID_SHOT_VELOCITY);
  }

  /**
   * Set shooter to high shot configuration.
   */
  public void setHighShot() {
    setFlywheelVelocity(ShooterConstants.FLYWHEEL_HIGH_SHOT_VELOCITY);
  }

  /**
   * Get the current flywheel velocity.
   * @return Current flywheel velocity
   */
  public AngularVelocity getFlywheelVelocity() {
    return inputs.flywheelVelocity;
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
   * Check if the shooter is ready to shoot (flywheel at goal).
   * @return True if ready to shoot
   */
  public boolean isReadyToShoot() {
    return isFlywheelAtGoal() && 
           flywheelGoal.in(RotationsPerSecond) > ShooterConstants.FLYWHEEL_IDLE_VELOCITY.in(RotationsPerSecond);
  }

  public Voltage getFlywheelVoltage() {
    return inputs.flywheelAppliedVoltage ;
  }

  /**
   * Get the total flywheel current (leader + follower).
   * @return Total flywheel current in amps
   */
  public Current getTotalFlywheelCurrent() {
    return inputs.flywheelTotalCurrent ;
  }
}
