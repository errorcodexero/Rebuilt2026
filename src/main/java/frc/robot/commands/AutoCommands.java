package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.RobotCommands;

public class AutoCommands {


    public static Command NeutralZoneTrenchToTrench(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter){
        return Commands.sequence(
            DriveCommands.initialFollowPathCommand(drive,"TopToBottom Trench").deadlineFor(intake.intakeSequence()),

            RobotCommands.shoot(shooter, hopper, drive, null, false),
            DriveCommands.followPathCommand("BottomToTop").deadlineFor(intake.intakeSequence()),
            RobotCommands.shoot(shooter, hopper, drive, null, false)
        );
    }
    
}