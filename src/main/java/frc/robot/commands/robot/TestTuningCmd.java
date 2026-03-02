package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.tunings.FlatShooterTuning;
import frc.robot.subsystems.shooter.tunings.ShooterTuning.ShooterParams;

public class TestTuningCmd extends Command {
    private double dist_ ;
    private double incr_ ;
    private FlatShooterTuning tuning_ ;

    public TestTuningCmd() {
        tuning_ = new FlatShooterTuning();
    }

    @Override
    public void initialize() {
         dist_ = 0 ;
         incr_ = 0.05 ;
     }

    @Override
    public void execute() {
        ShooterParams p = tuning_.getShooterParams(dist_);
        System.out.println("Distance: " + dist_ + " Shooter Params: hood = " + p.hood + "  wheel = " + p.velocity) ;

        if (dist_ < 0.0 && incr_ < 0.0) {
            incr_ = -incr_ ;
        } else if (dist_ > 6.0 && incr_ > 0.0) {
            incr_ = -incr_ ;
        }

        dist_ += incr_ ;
    }

    @Override
    public boolean isFinished() {
       return false;
    }    
}
