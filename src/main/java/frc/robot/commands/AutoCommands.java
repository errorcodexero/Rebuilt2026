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
    public static Command a1TrenchToTrench(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, boolean mirroredX){
        return Commands.sequence(
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive,"A1_TopToBottomTrench", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.4)))
            ),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),
            DriveCommands.followPathCommand("A1_BottomToTopTrench", mirroredX).deadlineFor(intake.intakeSequence()),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0))
        );
    }

    public static Command a2NZCollectAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper, boolean mirroredX){
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive, "a2TrenchToNZ", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.4)))
            ),
            //Drive robot to the shoot position and shoot balls into hub
            DriveCommands.followPathCommand("a2NZtoShoot1", mirroredX),
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(3.8)),

            //Drive back into the neutral zone, collecting more balls along the way
            DriveCommands.followPathCommand("a2Shoot1toHub").deadlineFor(intake.intakeSequence()),
            
            //Drive back to alliance zone and shoot the rest of the balls into the hub
            DriveCommands.followPathCommand("a2HubToShoot2", mirroredX),
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(4.0))
            
        ); 
    }

    public static Command a3DepotShootNZ(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, boolean fieldside){
        return Commands.sequence(
            //Drive to the depot and collect the balls from it, then drive to the starting position trench to shoot collected balls
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive, "GoToCollectDepot",fieldside),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.4)))
            ),
            
            //Shoot the balls from collecting the depot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(3)),

            //Drive to the Neutral Zone and Collect, then drive to the opposite trench
            DriveCommands.followPathCommand("TopTrenchToNZ", fieldside).deadlineFor(intake.intakeSequence()),

            //Shoot the remainging balls
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0))
        );
    }
}