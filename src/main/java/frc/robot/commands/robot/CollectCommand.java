package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

/**
 * Robot-level command to collect game pieces.
 * Deploys the intake, spins the intake roller, and activates hopper agitators.
 * When interrupted, safely stops all motors and retracts the intake.
 */
public class CollectCommand {
  
  /**
   * Creates a command to collect game pieces.
   * This command will:
   * - Deploy the intake
   * - Start the intake spinner motor
   * - Start the hopper agitator motors
   * 
   * When interrupted, it will:
   * - Stop the intake spinner
   * - Stop the hopper agitators
   * - Retract the intake
   * 
   * @param intake The intake subsystem
   * @param hopper The hopper subsystem
   * @return A command that collects game pieces
   */
  public static Command collect(Intake intake, Hopper hopper) {
    return Commands.parallel(
        // Deploy intake and start spinner
        Commands.sequence(
            Commands.runOnce(() -> intake.deploy(), intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.run(() -> intake.intake(), intake)
        ),
        // Start hopper agitators
        Commands.run(() -> hopper.activeAgitators(), hopper)
    )
    .finallyDo((interrupted) -> {
      // Clean up when command ends or is interrupted
      intake.stopSpinner();
      hopper.stopAgitators();
      intake.retract();
    })
    .withName("Collect");
  }
  
  /**
   * Creates a command to collect game pieces without waiting for deploy.
   * Starts all motors immediately while deploying.
   * 
   * @param intake The intake subsystem
   * @param hopper The hopper subsystem
   * @return A command that collects game pieces quickly
   */
  public static Command collectFast(Intake intake, Hopper hopper) {
    return Commands.parallel(
        // Deploy intake
        Commands.runOnce(() -> intake.deploy(), intake),
        // Start intake spinner immediately
        Commands.run(() -> intake.intake(), intake),
        // Start hopper agitators
        Commands.run(() -> hopper.activeAgitators(), hopper)
    )
    .finallyDo((interrupted) -> {
      // Clean up when command ends or is interrupted
      intake.stopSpinner();
      hopper.stopAgitators();
      intake.retract();
    })
    .withName("CollectFast");
  }
}
