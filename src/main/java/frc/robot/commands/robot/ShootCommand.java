package frc.robot.commands.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Robot-level command to shoot game pieces.
 * Sets shooter flywheel velocity and feeds balls from hopper.
 * When interrupted, safely stops all motors.
 */
public class ShootCommand {
  
  /**
   * Creates a command to shoot game pieces at a specific velocity.
   * This command will:
   * - Spin up shooter flywheels to target velocity
   * - Wait until shooter is ready
   * - Start feeding balls from hopper
   * 
   * When interrupted, it will:
   * - Stop the shooter flywheels
   * - Stop the hopper feeder
   * 
   * @param shooter The shooter subsystem
   * @param hopper The hopper subsystem
   * @param flywheelVelocity The target flywheel velocity
   * @return A command that shoots game pieces
   */
  public static Command shoot(Shooter shooter, Hopper hopper, AngularVelocity flywheelVelocity) {
    return Commands.sequence(
        // Prepare shooter (spin up flywheels)
        Commands.runOnce(() -> shooter.setFlywheelVelocity(flywheelVelocity), shooter),
        // Wait until shooter is ready
        Commands.waitUntil(shooter::isReadyToShoot),
        // Start feeding balls
        Commands.run(() -> hopper.feed(), hopper)
    )
    .finallyDo((interrupted) -> {
      // Clean up when command ends or is interrupted
      shooter.stopFlywheels();
      hopper.stopFeeder();
    })
    .withName("Shoot");
  }
  
  /**
   * Creates a command to shoot game pieces with preset low shot configuration.
   * 
   * @param shooter The shooter subsystem
   * @param hopper The hopper subsystem
   * @return A command that shoots at low configuration
   */
  public static Command shootLow(Shooter shooter, Hopper hopper) {
    return Commands.sequence(
        // Prepare shooter with low shot preset
        Commands.runOnce(() -> shooter.setLowShot(), shooter),
        // Wait until shooter is ready
        Commands.waitUntil(shooter::isReadyToShoot),
        // Start feeding balls
        Commands.run(() -> hopper.feed(), hopper)
    )
    .finallyDo((interrupted) -> {
      // Clean up when command ends or is interrupted
      shooter.stopFlywheels();
      hopper.stopFeeder();
    })
    .withName("ShootLow");
  }
  
  /**
   * Creates a command to shoot game pieces with preset mid shot configuration.
   * 
   * @param shooter The shooter subsystem
   * @param hopper The hopper subsystem
   * @return A command that shoots at mid configuration
   */
  public static Command shootMid(Shooter shooter, Hopper hopper) {
    return Commands.sequence(
        // Prepare shooter with mid shot preset
        Commands.runOnce(() -> shooter.setMidShot(), shooter),
        // Wait until shooter is ready
        Commands.waitUntil(shooter::isReadyToShoot),
        // Start feeding balls
        Commands.run(() -> hopper.feed(), hopper)
    )
    .finallyDo((interrupted) -> {
      // Clean up when command ends or is interrupted
      shooter.stopFlywheels();
      hopper.stopFeeder();
    })
    .withName("ShootMid");
  }
  
  /**
   * Creates a command to shoot game pieces with preset high shot configuration.
   * 
   * @param shooter The shooter subsystem
   * @param hopper The hopper subsystem
   * @return A command that shoots at high configuration
   */
  public static Command shootHigh(Shooter shooter, Hopper hopper) {
    return Commands.sequence(
        // Prepare shooter with high shot preset
        Commands.runOnce(() -> shooter.setHighShot(), shooter),
        // Wait until shooter is ready
        Commands.waitUntil(shooter::isReadyToShoot),
        // Start feeding balls
        Commands.run(() -> hopper.feed(), hopper)
    )
    .finallyDo((interrupted) -> {
      // Clean up when command ends or is interrupted
      shooter.stopFlywheels();
      hopper.stopFeeder();
    })
    .withName("ShootHigh");
  }
  
  /**
   * Creates a command to prepare the shooter without feeding.
   * Useful for pre-spinning before shooting.
   * 
   * @param shooter The shooter subsystem
   * @param flywheelVelocity The target flywheel velocity
   * @return A command that prepares the shooter
   */
  public static Command prepareShooter(Shooter shooter, AngularVelocity flywheelVelocity) {
    return Commands.runOnce(() -> shooter.setFlywheelVelocity(flywheelVelocity), shooter)
    .andThen(Commands.waitUntil(shooter::isReadyToShoot))
    .withName("PrepareShooter");
  }
  
  /**
   * Creates a command to shoot for a specific duration.
   * Automatically stops after the specified time.
   * 
   * @param shooter The shooter subsystem
   * @param hopper The hopper subsystem
   * @param flywheelVelocity The target flywheel velocity
   * @param seconds The duration to shoot in seconds
   * @return A command that shoots for the specified duration
   */
  public static Command shootForDuration(Shooter shooter, Hopper hopper, 
                                         AngularVelocity flywheelVelocity, 
                                         double seconds) {
    return shoot(shooter, hopper, flywheelVelocity)
        .withTimeout(seconds)
        .withName("ShootForDuration");
  }
}
