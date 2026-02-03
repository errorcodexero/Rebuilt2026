package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
        double shooterVelocity = SmartDashboard.getNumber("Shooter/Tuning/ShooterRPS", 0);

        
        double HoodAngle = SmartDashboard.getNumber("Shooter/Tuning/HoodAngle", ShooterConstants.SoftwareLimits.hoodMinAngle);

        shooter_.goToShootReadyCommand(RotationsPerSecond.of(shooterVelocity), Degrees.of(HoodAngle));

        SmartDashboard.putNumber("Shooter/Tuning/ActualShooterRPS", shooter_.getShooterVelocity().in(RotationsPerSecond));
        SmartDashboard.putBoolean("Shooter/Tuning/ShooterReady", shooter_.isShooterReady());
    }

    public void end() {
        shooter_.stopCommand();
    }

    @Override
    public boolean isFinished() {return false;}
}
