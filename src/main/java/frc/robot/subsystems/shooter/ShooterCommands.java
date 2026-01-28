package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {

    public static Command setShooterVelocityCommand(Shooter shooter, AngularVelocity vel) {
        return Commands.runOnce(() -> shooter.setShooterVelocity(vel), shooter)
        .andThen(Commands.waitUntil(() -> shooter.isShooterReady())).withName("Set Shooter Velocity");
    }

    public static Command hoodToPosCommand(Shooter shooter, Angle pos) {
        return Commands.runOnce(() -> shooter.setHoodAngle(pos), shooter).withName("Set Hood Position");
    }

    public static Command goToShootReadyCommand(Shooter shooter, AngularVelocity vel, Angle pos) {
        return Commands.parallel(setShooterVelocityCommand(shooter, vel), hoodToPosCommand(shooter, pos)).withName("Ready To Shoot");
    }

    public static Command stopShooterCommand(Shooter shooter) {
        return Commands.runOnce(() -> shooter.stopShooter(), shooter)
        .andThen(Commands.waitUntil(() -> shooter.isShooterReady())).withName("Stop Shooter");
    }

    public static Command setShooterVoltageCommand(Shooter shooter, Voltage vol) {
        return Commands.runOnce(() -> shooter.setShooterVoltage(vol), shooter).withName("Set Shooter Voltage");
    }

    public static Command setHoodVoltageCommand(Shooter shooter, Voltage vol) {
        return Commands.runOnce(() -> shooter.setHoodVoltage(vol), shooter).withName("Set Hood Voltage");
    }
}
