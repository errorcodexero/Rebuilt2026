package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

/**
 * Factory class for creating intake commands.
 */
public class IntakeCommands {
  
  /**
   * Command to deploy the intake and wait until it reaches the deployed position.
   * @param intake The intake subsystem
   * @return A command that deploys the intake
   */
  public static Command deploy(Intake intake) {
    return Commands.runOnce(() -> intake.deploy(), intake)
        .andThen(Commands.waitUntil(intake::isDeployed))
        .withName("DeployIntake");
  }

  /**
   * Command to retract the intake and wait until it reaches the retracted position.
   * @param intake The intake subsystem
   * @return A command that retracts the intake
   */
  public static Command retract(Intake intake) {
    return Commands.runOnce(() -> intake.retract(), intake)
        .andThen(Commands.waitUntil(intake::isRetracted))
        .withName("RetractIntake");
  }

  /**
   * Command to move the intake to a specific angle.
   * @param intake The intake subsystem
   * @param angle The target angle
   * @return A command that moves the intake to the specified angle
   */
  public static Command moveToAngle(Intake intake, Angle angle) {
    return Commands.runOnce(() -> intake.setDeployPosition(angle), intake)
        .andThen(Commands.waitUntil(intake::isDeployAtGoal))
        .withName("MoveIntakeToAngle");
  }

  /**
   * Command to move the intake to a specific angle without waiting.
   * @param intake The intake subsystem
   * @param angle The target angle
   * @return A command that starts moving the intake to the specified angle
   */
  public static Command setDeployAngle(Intake intake, Angle angle) {
    return Commands.runOnce(() -> intake.setDeployPosition(angle), intake)
        .withName("SetDeployAngle");
  }

  /**
   * Command to run the intake at the default intake velocity.
   * @param intake The intake subsystem
   * @return A command that runs the intake
   */
  public static Command intake(Intake intake) {
    return Commands.run(() -> intake.intake(), intake)
        .withName("RunIntake");
  }

  /**
   * Command to run the intake at the default eject velocity.
   * @param intake The intake subsystem
   * @return A command that ejects from the intake
   */
  public static Command eject(Intake intake) {
    return Commands.run(() -> intake.eject(), intake)
        .withName("EjectIntake");
  }

  /**
   * Command to set the spinner to a specific velocity.
   * @param intake The intake subsystem
   * @param velocity The target velocity
   * @return A command that sets the spinner velocity
   */
  public static Command setSpinnerVelocity(Intake intake, AngularVelocity velocity) {
    return Commands.run(() -> intake.setSpinnerVelocity(velocity), intake)
        .withName("SetSpinnerVelocity");
  }

  /**
   * Command to stop the spinner motor.
   * @param intake The intake subsystem
   * @return A command that stops the spinner
   */
  public static Command stopSpinner(Intake intake) {
    return Commands.runOnce(() -> intake.stopSpinner(), intake)
        .withName("StopSpinner");
  }

  /**
   * Command to deploy and start intaking.
   * @param intake The intake subsystem
   * @return A command that deploys and runs the intake
   */
  public static Command deployAndIntake(Intake intake) {
    return Commands.parallel(
        deploy(intake),
        intake(intake)
    ).withName("DeployAndIntake");
  }

  /**
   * Command to stop the spinner and retract the intake.
   * @param intake The intake subsystem
   * @return A command that stops and retracts the intake
   */
  public static Command stopAndRetract(Intake intake) {
    return stopSpinner(intake)
        .andThen(retract(intake))
        .withName("StopAndRetract");
  }

  /**
   * Command to hold the deploy position at the current angle.
   * Useful for maintaining position without further movement.
   * @param intake The intake subsystem
   * @return A command that holds the current deploy position
   */
  public static Command holdPosition(Intake intake) {
    return Commands.runOnce(
        () -> intake.setDeployPosition(intake.getDeployPosition()), 
        intake
    ).withName("HoldDeployPosition");
  }

  /**
   * Command to reset the deploy encoder to a known angle.
   * @param intake The intake subsystem
   * @param currentAngle The current known angle of the intake
   * @return A command that resets the encoder
   */
  public static Command resetDeployPosition(Intake intake, Angle currentAngle) {
    return Commands.runOnce(
        () -> intake.resetDeployPosition(currentAngle), 
        intake
    ).withName("ResetDeployPosition");
  }
}
