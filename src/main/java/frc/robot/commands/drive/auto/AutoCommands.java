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
            Commands.parallel(
                DriveCommands.initialFollowPathCommand(drive,"GoToDepot"),
                intake.deployCmd()
            ),
            Commands.runOnce(()-> System.err.println("DRIVE POSE AFTER RESET:" + drive.getPose())),

            DriveCommands.followPathCommand("Collect").deadlineFor(intake.runIntakeCmd()),

            DriveCommands.followPathCommand("GoToShoot"),

            shooter.shootCmd(hopper).withTimeout(2.5),

            shooter.stopCmd(),

            DriveCommands.followPathCommand("GoToOutpost"), 

            Commands.waitSeconds(2),

            DriveCommands.followPathCommand("OutpostToShoot"),

            shooter.shootCmd(hopper).withTimeout(2),

            shooter.stopCmd(),

            DriveCommands.followPathCommand("ShootToClimb")
        );
    }
}
