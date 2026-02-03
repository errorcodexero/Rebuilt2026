package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterTuningCommand extends Command {
    private final Shooter shooter_;

    public ShooterTuningCommand(Shooter shooter) {
        shooter_ = shooter;
        addRequirements(shooter_);
        SmartDashboard.putNumber("Shooter/Tuning/FlywheelRPS", 0);
        SmartDashboard.putNumber("Shooter/Tuning/HoodAngle", ShooterConstants.SoftwareLimits.hoodMinAngle);
    }

    
    public void intialize() {}

    @Override
    public void execute() {

        LoggedNetworkNumber shooterVelocity = new LoggedNetworkNumber("Shooter/Tuning/TargetShooterRPS", 0);
        LoggedNetworkNumber HoodAngle = new LoggedNetworkNumber("Shooter/Tuning/TargetHoodAngle", ShooterConstants.SoftwareLimits.hoodMinAngle);

        shooter_.goToShootReadyCommand(RotationsPerSecond.of(shooterVelocity.getAsDouble()), Degrees.of(HoodAngle.getAsDouble()));

        SmartDashboard.putNumber("Shooter/Tuning/ActualShooterRPS", shooter_.getShooterVelocity().in(RotationsPerSecond));
        SmartDashboard.putBoolean("Shooter/Tuning/ShooterReady", shooter_.isShooterReady());
    }

    public void end() {
        shooter_.stopCommand();
    }

    @Override
    public boolean isFinished() {return false;}
}
