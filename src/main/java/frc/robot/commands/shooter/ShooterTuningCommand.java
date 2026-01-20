package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Command for tuning shooter parameters via SmartDashboard.
 * Allows manual control of flywheel velocity.
 * Updates continuously based on SmartDashboard inputs.
 */
public class ShooterTuningCommand extends Command {
  private final Shooter shooter;
  
  // SmartDashboard keys
  private static final String FLYWHEEL_VELOCITY_KEY = "Shooter/Tuning/FlywheelVelocityRPS";
  
  /**
   * Creates a new ShooterTuningCommand.
   * 
   * @param shooter The shooter subsystem
   */
  public ShooterTuningCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    
    // Initialize SmartDashboard with default values
    SmartDashboard.putNumber(FLYWHEEL_VELOCITY_KEY, ShooterConstants.FLYWHEEL_MID_SHOT_VELOCITY.in(RotationsPerSecond));
  }

  @Override
  public void initialize() {
    // Log that tuning mode is active
    System.out.println("Shooter Tuning Command Started");
  }

  @Override
  public void execute() {
    // Read values from SmartDashboard
    double flywheelVelocityRPS = SmartDashboard.getNumber(FLYWHEEL_VELOCITY_KEY, 
        ShooterConstants.FLYWHEEL_MID_SHOT_VELOCITY.in(RotationsPerSecond));
    
    // Apply values to shooter
    shooter.setFlywheelVelocity(RotationsPerSecond.of(flywheelVelocityRPS));
    
    // Log current state to SmartDashboard
    SmartDashboard.putNumber("Shooter/Tuning/ActualFlywheelRPS", 
        shooter.getFlywheelVelocity().in(RotationsPerSecond));
    SmartDashboard.putBoolean("Shooter/Tuning/ReadyToShoot", 
        shooter.isReadyToShoot());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors when command ends
    shooter.stopFlywheels();
    
    if (interrupted) {
      System.out.println("Shooter Tuning Command Interrupted");
    } else {
      System.out.println("Shooter Tuning Command Ended");
    }
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}
