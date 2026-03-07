package frc.robot.commands.drive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.thriftyclimb.ThriftyClimb;

public class AutoCommands {
    public static Command DepotShootClimbAuto(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, ThriftyClimb climber, boolean fieldside){
        return Commands.sequence(
            Commands.runOnce(() -> drive.resetGyroCmd()),

            DriveCommands.initialFollowPathCommand(drive, "GoToCollectDepot",fieldside).deadlineFor(intake.intakeSequence()),

            shooter.shootCmd(hopper).withTimeout(4),

            shooter.stopCmd(),

            /*DriveCommands.followPathCommand("DepotToNZ", fieldside).deadlineFor(intake.intakeSequence()),

            shooter.shootCmd(hopper).withTimeout(5)*/
        );
    }
}
