package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {

    public static Command goToShootReadyCommand(Shooter shooter, AngularVelocity vel, Angle pos) {
        return Commands.parallel(shooter.setVelocityCmd(vel), shooter.hoodToPosCmd(pos)).withName("Ready To Shoot");
    }
}
