package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.RobotCommands;
import frc.robot.commands.robot.StartupCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class AutoCommands {
    public static Command trenchToTrench(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, boolean mirroredX){
        return Commands.sequence(
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive,"TopToBottom Trench", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.4)))
            ),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),
            DriveCommands.followPathCommand("BottomToTop", mirroredX).deadlineFor(intake.intakeSequence()),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0))
        );
    }
}