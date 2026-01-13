package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.units.Units.*;

/**
 * Command that reads a voltage value from SmartDashboard and applies it to the flywheel motors.
 * Useful for open-loop characterization and testing.
 */
public class FlywheelVoltageCommand extends Command {
  private final Shooter shooter_;
  
  private static final String VOLTAGE_KEY = "Flywheel/VoltageSetpoint";
  private static final double DEFAULT_VOLTAGE = 0.0;

  public FlywheelVoltageCommand(Shooter shooter) {
    shooter_ = shooter;
    addRequirements(shooter_);
    
    // Initialize SmartDashboard value
    SmartDashboard.putNumber(VOLTAGE_KEY, DEFAULT_VOLTAGE);
  }

  @Override
  public void initialize() {
    // Ensure the key exists on dashboard
    if (!SmartDashboard.containsKey(VOLTAGE_KEY)) {
      SmartDashboard.putNumber(VOLTAGE_KEY, DEFAULT_VOLTAGE);
    }
  }

  @Override
  public void execute() {
    // Read voltage from SmartDashboard
    double voltageSetpoint = SmartDashboard.getNumber(VOLTAGE_KEY, DEFAULT_VOLTAGE);
    
    // Apply voltage to flywheel motors
    shooter_.setFlywheelVoltage(Volts.of(voltageSetpoint));
    
    // Log actual flywheel velocity for feedback
    SmartDashboard.putNumber("Flywheel/ActualVelocityRPS", 
        shooter_.getFlywheelVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("Flywheel/LeaderCurrentAmps", 
        shooter_.getFlywheelLeaderCurrent());
    SmartDashboard.putNumber("Flywheel/FollowerCurrentAmps", 
        shooter_.getFlywheelFollowerCurrent());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop flywheels when command ends
    shooter_.stopFlywheels();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
