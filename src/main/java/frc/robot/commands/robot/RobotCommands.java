package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotCommands {
    /**
     * Shoots at either the hub, or ferrying targets based on the current robot position.
     * @param shooter
     * @param hopper
     * @param drive
     * @param intake
     * @param gamepad
     * @param shootOnMove
     * 
     * @return
     */
    public static Command shoot(Shooter shooter, Hopper hopper, Drive drive, IntakeSubsystem intake, CommandXboxController gamepad, boolean shootOnMove, Trigger shakeTrigger) {
        return Commands.parallel(
                DriveCommands.pointAtShootingTarget(drive, shooter, gamepad, shootOnMove),
                Commands.repeatingSequence(
                    Commands.waitUntil(() -> drive.rotationIsNear(drive.getVirtualTarget(shooter).minus(drive.getPose().getTranslation()).getAngle(), ShooterConstants.aimingTolerance)),
                    shooter.shoot(drive, hopper, intake, shakeTrigger)
                )
            );
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
