package frc.robot.commands.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotCommands {
    /**
     * Shoots into the hub.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    private static Command shootHub(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Trigger shakeTrigger) {
        BooleanSupplier aimedAtHub =
            () -> drive.rotationIsNear(RobotState.rotationToVirtualHub(), ShooterConstants.aimingTolerance);

        return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(RobotState::rotationToVirtualHub),
            Commands.repeatingSequence(
                Commands.waitUntil(aimedAtHub).deadlineFor(shooter.spinUpForDistance(RobotState::hubDistance)),
                shooter.shootAtDistance(RobotState::virtualHubDistance, hopper, intake, shakeTrigger)
                    .until(() -> !aimedAtHub.getAsBoolean())
            )
        );
    }

    /**
     * Ferrys into a target in your alliance zone. Allows movement.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    private static Command ferry(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Trigger shakeTrigger) {
        return DriveCommands.joystickDriveAtAngle(RobotState::rotationToVirtualFerry)
            .alongWith(
                shooter.shootAtDistance(RobotState::virtualFerryDistance, hopper, intake, shakeTrigger),
                Commands.runOnce(() -> Logger.recordOutput("Ferry/IsFerrying", true))
            )
            .finallyDo(i -> Logger.recordOutput("Ferry/IsFerrying", false));
    }

    /**
     * Shoots at either the hub, or ferrying targets based on the current robot position.
     * @param shooter
     * @param hopper
     * @param drive
     * @param shakeTrigger
     * @return
     */
    public static Command shoot(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Trigger shakeTrigger) {
        return Commands.either(
            shootHub(shooter, hopper, intake, drive, shakeTrigger),
            ferry(shooter, hopper, intake, drive, shakeTrigger),
            RobotState::inAllianceZone
        );
    }

    /**
     * Shoots at either the hub, or ferrying targets based on the current robot position.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    public static Command shoot(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive) {
        return shoot(shooter, hopper, intake, drive, new Trigger(() -> false));
    }

    /**
     * Ejects balls from the shooter at a low velocity to get them out of the shooter without shooting them towards the target.
     * @param shooter
     * @return
     */
    public static Command ejectUp(Shooter shooter, Hopper hopper) {
        return shooter.ejectUp().alongWith(hopper.ejectUp());
    }

    /**
     * The full sequence for intaking balls, with scrambler movement. For intake while shooting, or
     * any situation where we want to intake while the hopper is already required, just use intake.intakeSequence().
     * 
     * For most cases, this command is preferred.
     * @param intake
     * @param hopper
     * @return
     */
    public static Command intake(IntakeSubsystem intake, Hopper hopper) {
        return intake.intakeSequence().alongWith(hopper.collectScrambler());
    }
}
