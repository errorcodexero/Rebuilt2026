package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;

public class IntakeCommands {

    //////////////////////////////////////////////////////
    /// First stow the intake, and then wait until it is stowed
    //////////////////////////////////////////////////////
    public static Command stowIntakeCommand(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.stowIntake(), intake)
        .andThen(Commands.waitUntil(() -> intake.isIntakeStowed())
        .withTimeout(1)).withName("Stow Intake");
    }

    //////////////////////////////////////////////////////
    /// First deploy the intake, and then wait until it is deployed
    //////////////////////////////////////////////////////
    public static Command deployIntakeCommand(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.deployIntake(), intake)
        .andThen(Commands.waitUntil(() -> intake.isIntakeDeployed())
        .withTimeout(1)).withName("Deploy Intake");
    }

    /////////////////////////
    /// Stop the rollers once
    /////////////////////////
    public static Command stopRollerCommand(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.stopRoller(), intake)
        .withName("Stop Roller");
    }

    ////////////////////////////////////////////////////////
    /// Continously set the roller voltage until interrupted
    ////////////////////////////////////////////////////////
    public static Command setRollerVoltageCommand(IntakeSubsystem intake, Voltage volts) {
        return Commands.run(() -> intake.setRollerVoltage(volts), intake)
        .withName("Set Roller Voltage");
    }

    //////////////////////////////////////////////////////////////////
    /// Continously move the rollers at set velocity until interrupted
    //////////////////////////////////////////////////////////////////
    public static Command setRollerVelocityCommand(IntakeSubsystem intake, AngularVelocity velocity) {
        return Commands.run(() -> intake.setRollerVelocity(velocity), intake)
        .withName("Set Roller Velocity");
    }

    ////////////////////////////////////////////////////////////////
    /// Continously move the pivot at set velocity until interrupted
    ////////////////////////////////////////////////////////////////
    public static Command setPivotVoltageCommand(IntakeSubsystem intake, Voltage voltage) {
        return Commands.run(() -> intake.setPivotVoltage(voltage), intake)
        .withName("Set Pivot Voltage");
    }

    /////////////////////////////////////////////////////////////
    /// First set the angle target, then wait until it is reached
    /////////////////////////////////////////////////////////////
    public static Command setPivotAngleCommand(IntakeSubsystem intake, Angle angle) {
        return Commands.runOnce(() -> intake.setPivotAngle(angle), intake)
        .andThen(Commands.waitUntil(() -> intake.isPivotAtAngle(angle))
        .withTimeout(1)).withName("Set Pivot Angle");
    }

    ////////////////////////////////
    ///Deploy and intake in parallel
    ////////////////////////////////
    public static Command intakeDeployCommand(IntakeSubsystem intake){
        return Commands.parallel(setRollerVoltageCommand(intake, IntakeConstants.rollerCollectVoltage), deployIntakeCommand(intake));
    }

    ////////////////////////////////////////
    ///Stow and stop the rollers in parallel
    ////////////////////////////////////////
    public static Command stopStowCommand(IntakeSubsystem intake){
        return Commands.parallel(stopRollerCommand(intake), stowIntakeCommand(intake));
    }
}
