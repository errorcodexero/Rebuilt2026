package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class AdjustHoodAngleCmd extends Command {
    
    private final Shooter shooter;
    private final Angle delta;
    
    public AdjustHoodAngleCmd(Shooter shooter, Angle delta) {
        super();
        this.shooter = shooter;
        this.delta = delta;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Angle current = shooter.getHoodAngle();
        if (current == null) {
            current = Degrees.zero();
        }

        double targetDeg = current.plus(delta).in(Degrees);
        double clampedDeg = Math.max(ShooterConstants.SoftwareLimits.hoodMinAngle, 
        Math.min(ShooterConstants.SoftwareLimits.hoodMaxAngle, targetDeg));

        shooter.hoodToPosCmd(Degrees.of(clampedDeg)).schedule();
    }
}
