package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
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
    public static Command a1TrenchToTrench(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, boolean mirroredX) {
        return Commands.sequence(
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive,"A1_TopToBottomTrench", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.8)))
            ),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),
            DriveCommands.followPathCommand("A1_BottomToTopTrench", mirroredX)
                .deadlineFor(RobotCommands.intake(intake, hopper)),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0))
        );
    }

    public static Command a2NZCollectAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper, boolean mirroredX) {
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive, "a2TrenchToNZ", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.6)))
            ),
            //Drive robot to the shoot position and shoot balls into hub
            DriveCommands.followPathCommand("a2NZtoShoot1", mirroredX).deadlineFor(
                shooter.spinUpForDistanceHoodParked(() -> Meters.of(3.7)),
                hopper.preShoot()
            ),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),

            //Drive back into the neutral zone, collecting more balls along the way
            DriveCommands.followPathCommand("a2Shoot1toHub", mirroredX)
                .deadlineFor(RobotCommands.intake(intake, hopper)),
            
            //Drive back to alliance zone and shoot the rest of the balls into the hub
            DriveCommands.followPathCommand("a2HubToShoot2", mirroredX).deadlineFor(
                shooter.spinUpForDistanceHoodParked(() -> Meters.of(3.7)),
                hopper.preShoot()
            ),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(10.0))
            
        ); 
    }

    public static Command a3DepotAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper) {
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive, "a3BumpToDepot"),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.6)))
            ),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),

            DriveCommands.followPathCommand("a3DepotToOutpost")
                .deadlineFor(RobotCommands.intake(intake, hopper)),

            DriveCommands.followPathCommand("a3OutpostToShoot"),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(8.0))
        ); 
    }
    
    public static Command a4OnlyPreload(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper) {
        return Commands.sequence(
            DriveCommands.initialFollowPathCommand(drive, "a4OnlyPreload")
                .deadlineFor(new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.3)))),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(4))
        );
    }

    public static Command aTest1DriveCollect(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper) {
        return new StartupCmd(intake, hopper, shooter).alongWith(
            DriveCommands.initialFollowPathCommand(drive, "aTest1DriveCollect")
                .beforeStarting(Commands.waitTime(Seconds.one()))
        );
    }

    private static final Time shootingStartTime1 = Seconds.of(18);
    private static final Time shootingLength1 = Seconds.of(5);

    private static final Time shootingLength2 = Seconds.of(20);
    private static final Time shootingOverlap2 = Milliseconds.of(500);

    public static Command a5NZCollectAutoAlt(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper, boolean mirroredX) {
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            // Commands.parallel(
            //     DriveCommands.initialFollowPathCommand(drive, "a5CollectLoop1", mirroredX)
            //         .deadlineFor(new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.6))))
                
            //     // RobotCommands.shootHubNoAim(shooter, hopper, intake, drive).beforeStarting(Commands.waitTime(shootingStartTime1))
            // ),

            Commands.parallel(
                DriveCommands.initialFollowPathCommand(drive, "a5CollectLoop1", mirroredX),
                Commands.sequence(
                    new StartupCmd(intake, hopper, shooter).withTimeout(shootingStartTime1),
                    RobotCommands.shootHubNoAim(shooter, hopper, intake, drive).withTimeout(shootingLength1)
                )
            ),

            // Commands.parallel(
            //     // RobotCommands.shootHubNoAim(shooter, hopper, intake, drive).withTimeout(shootingLength2),
                
            //     DriveCommands.followPathCommand("a5CollectLoop2", mirroredX)
            //         .deadlineFor(RobotCommands.intake(intake, hopper))
            //         .beforeStarting(Commands.waitTime(shootingLength2.minus(shootingOverlap2)))
            // ),

            RobotCommands.shoot(shooter, hopper, intake, drive)
        );
    }
}