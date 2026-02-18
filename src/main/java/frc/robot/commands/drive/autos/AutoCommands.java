package frc.robot.commands.drive.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.thriftyclimb.ThriftyClimb;


import edu.wpi.first.units.measure.Angle;

public class AutoCommands {
    public static Command NZClimbAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper, ThriftyClimb climb, Angle angle ){
        return Commands.sequence(
            //Drive to the first shooting position and shoot preloaded balls   
            DriveCommands.initialFollowPathCommand(drive, "Start to Shoot1", false),
            shooter.shootCmd(hopper).withTimeout(1),

            //Drive to the collect position in the NZ and collect the ball s while moving, bringing down the hood to clear the trench
            
            Commands.sequence(
                Commands.parallel(
                    DriveCommands.followPathCommand("Shoot1 to Collect", false),
                    shooter.hoodToPosCmd(angle)
                ),

                //Drive to the second shooting position and shooting
                DriveCommands.followPathCommand("Collect to Shoot2", false)
            ).deadlineFor(intake.intakeSequence()),

            shooter.shootCmd(hopper).withTimeout(4),
            
            //Drive to the climb position, then climb
            DriveCommands.followPathCommand("Shoot2 to Climb", false),
            climb.toggle(),
            Commands.waitSeconds(1),
            climb.toggle()
        ); 
    }
}
