package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Factory class for creating shooter commands.
 */
public class ShooterCommands {
  
  /**
   * Command to set the hood to a specific angle and wait until it reaches the position.
   * @param shooter The shooter subsystem
   * @param angle The target angle
   * @return A command that moves the hood to the specified angle
   */
  public static Command setHoodAngle(Shooter shooter, Angle angle) {
    return Commands.runOnce(() -> shooter.setHoodPosition(angle), shooter)
        .andThen(Commands.waitUntil(shooter::isHoodAtGoal))
        .withName("SetHoodAngle");
  }

  /**
   * Command to set the hood to a specific angle without waiting.
   * @param shooter The shooter subsystem
   * @param angle The target angle
   * @return A command that starts moving the hood to the specified angle
   */
  public static Command setHoodAngleNoWait(Shooter shooter, Angle angle) {
    return Commands.runOnce(() -> shooter.setHoodPosition(angle), shooter)
        .withName("SetHoodAngleNoWait");
  }

  /**
   * Command to set the flywheel velocity.
   * @param shooter The shooter subsystem
   * @param velocity The target velocity
   * @return A command that runs the flywheels at the specified velocity
   */
  public static Command setFlywheelVelocity(Shooter shooter, AngularVelocity velocity) {
    return Commands.run(() -> shooter.setFlywheelVelocity(velocity), shooter)
        .withName("SetFlywheelVelocity");
  }

  /**
   * Command to apply a specific voltage to the flywheel motors (open-loop control).
   * @param shooter The shooter subsystem
   * @param voltage The voltage to apply
   * @return A command that applies the specified voltage to the flywheels
   */
  public static Command setFlywheelVoltage(Shooter shooter, Voltage voltage) {
    return Commands.run(() -> shooter.setFlywheelVoltage(voltage), shooter)
        .withName("SetFlywheelVoltage");
  }

  /**
   * Command to spin up flywheels to a specific velocity and wait until ready.
   * @param shooter The shooter subsystem
   * @param velocity The target velocity
   * @return A command that spins up the flywheels and waits
   */
  public static Command spinUpFlywheels(Shooter shooter, AngularVelocity velocity) {
    return Commands.runOnce(() -> shooter.setFlywheelVelocity(velocity), shooter)
        .andThen(Commands.waitUntil(shooter::isFlywheelAtGoal))
        .withName("SpinUpFlywheels");
  }

  /**
   * Command to prepare for a low shot (spin up flywheels and position hood).
   * @param shooter The shooter subsystem
   * @return A command that prepares for a low shot
   */
  public static Command prepareLowShot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setLowShot(), shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareLowShot");
  }

  /**
   * Command to prepare for a mid shot (spin up flywheels and position hood).
   * @param shooter The shooter subsystem
   * @return A command that prepares for a mid shot
   */
  public static Command prepareMidShot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setMidShot(), shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareMidShot");
  }

  /**
   * Command to prepare for a high shot (spin up flywheels and position hood).
   * @param shooter The shooter subsystem
   * @return A command that prepares for a high shot
   */
  public static Command prepareHighShot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setHighShot(), shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareHighShot");
  }

  /**
   * Command to prepare for shooting with custom flywheel velocity and hood angle.
   * @param shooter The shooter subsystem
   * @param velocity The target flywheel velocity
   * @param angle The target hood angle
   * @return A command that prepares the shooter
   */
  public static Command prepareShot(Shooter shooter, AngularVelocity velocity, Angle angle) {
    return Commands.runOnce(() -> {
      shooter.setFlywheelVelocity(velocity);
      shooter.setHoodPosition(angle);
    }, shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareShot");
  }

  /**
   * Command to spin flywheels at idle speed.
   * @param shooter The shooter subsystem
   * @return A command that idles the flywheels
   */
  public static Command idleFlywheels(Shooter shooter) {
    return Commands.run(() -> shooter.spinToIdle(), shooter)
        .withName("IdleFlywheels");
  }

  /**
   * Command to stop the flywheels.
   * @param shooter The shooter subsystem
   * @return A command that stops the flywheels
   */
  public static Command stopFlywheels(Shooter shooter) {
    return Commands.runOnce(() -> shooter.stopFlywheels(), shooter)
        .withName("StopFlywheels");
  }

  /**
   * Command to stow the hood.
   * @param shooter The shooter subsystem
   * @return A command that stows the hood
   */
  public static Command stowHood(Shooter shooter) {
    return Commands.runOnce(() -> shooter.stowHood(), shooter)
        .andThen(Commands.waitUntil(shooter::isHoodStowed))
        .withName("StowHood");
  }

  /**
   * Command to stop flywheels and stow the hood (safe shutdown).
   * @param shooter The shooter subsystem
   * @return A command that safely shuts down the shooter
   */
  public static Command stopAndStow(Shooter shooter) {
    return stopFlywheels(shooter)
        .andThen(stowHood(shooter))
        .withName("StopAndStow");
  }

  /**
   * Command to hold the current hood position.
   * Useful for maintaining position without further movement.
   * @param shooter The shooter subsystem
   * @return A command that holds the current hood position
   */
  public static Command holdHoodPosition(Shooter shooter) {
    return Commands.runOnce(
        () -> shooter.setHoodPosition(shooter.getHoodPosition()), 
        shooter
    ).withName("HoldHoodPosition");
  }

  /**
   * Command to reset the hood encoder to a known angle.
   * @param shooter The shooter subsystem
   * @param currentAngle The current known angle of the hood
   * @return A command that resets the encoder
   */
  public static Command resetHoodPosition(Shooter shooter, Angle currentAngle) {
    return Commands.runOnce(
        () -> shooter.resetHoodPosition(currentAngle), 
        shooter
    ).withName("ResetHoodPosition");
  }

  /**
   * Command that keeps the shooter ready at a specific configuration.
   * This is a perpetual command that should be set as the default command.
   * @param shooter The shooter subsystem
   * @param velocity The flywheel velocity to maintain
   * @param angle The hood angle to maintain
   * @return A command that maintains shooter readiness
   */
  public static Command maintainReadiness(Shooter shooter, AngularVelocity velocity, Angle angle) {
    return Commands.run(() -> {
      shooter.setFlywheelVelocity(velocity);
      shooter.setHoodPosition(angle);
    }, shooter).withName("MaintainReadiness");
  }
}
