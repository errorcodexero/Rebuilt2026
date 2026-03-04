package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterTuning;

public class VirtualTarget {
    public static Translation2d getVirtualTargetFromTarget(Drive drive, Translation2d target, ShooterTuning tuning, int loopPasses){
        Distance distToHub;
        if(loopPasses == 0){
            distToHub = Meters.of(target.getDistance(drive.getPose().getTranslation()));
        }else{
            distToHub = Meters.of(getVirtualTargetFromTarget(drive, target, tuning, loopPasses - 1)
                .getDistance(drive.getPose().getTranslation())
            );
        }
        Time hangTime = Seconds.of(tuning.getShooterParams(distToHub.in(Meters)).hangtime);
        return target.minus(
                new Translation2d(
                    MetersPerSecond.of(drive.getFieldChassisSpeeds().vxMetersPerSecond).times(hangTime),
                    MetersPerSecond.of(drive.getFieldChassisSpeeds().vyMetersPerSecond).times(hangTime)
                )
            );
    }
}
