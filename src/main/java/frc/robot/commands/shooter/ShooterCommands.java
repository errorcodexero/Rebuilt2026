package frc.robot.commands.shooter;

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
   * Command to prepare for a low shot (spin up flywheels).
   * @param shooter The shooter subsystem
   * @return A command that prepares for a low shot
   */
  public static Command prepareLowShot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setLowShot(), shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareLowShot");
  }

  /**
   * Command to prepare for a mid shot (spin up flywheels).
   * @param shooter The shooter subsystem
   * @return A command that prepares for a mid shot
   */
  public static Command prepareMidShot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setMidShot(), shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareMidShot");
  }

  /**
   * Command to prepare for a high shot (spin up flywheels).
   * @param shooter The shooter subsystem
   * @return A command that prepares for a high shot
   */
  public static Command prepareHighShot(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setHighShot(), shooter)
        .andThen(Commands.waitUntil(shooter::isReadyToShoot))
        .withName("PrepareHighShot");
  }

  /**
   * Command to prepare for shooting with custom flywheel velocity.
   * @param shooter The shooter subsystem
   * @param velocity The target flywheel velocity
   * @return A command that prepares the shooter
   */
  public static Command prepareShot(Shooter shooter, AngularVelocity velocity) {
    return Commands.runOnce(() -> shooter.setFlywheelVelocity(velocity), shooter)
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
   * Command that keeps the shooter ready at a specific velocity.
   * This is a perpetual command that should be set as the default command.
   * @param shooter The shooter subsystem
   * @param velocity The flywheel velocity to maintain
   * @return A command that maintains shooter readiness
   */
  public static Command maintainReadiness(Shooter shooter, AngularVelocity velocity) {
    return Commands.run(() -> shooter.setFlywheelVelocity(velocity), shooter)
        .withName("MaintainReadiness");
  }
}
