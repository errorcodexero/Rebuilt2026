package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.ShooterTuning;

public class VirtualTarget {
    /**
     * Gets the translation of the virtual target based on a target, and the current drivebase velocity.
     * 
     * In order to shoot, we need to find the tuning constants for a shot at the distance from the virtual target,
     * which includes the hangtime of a shot of that distance.
     * 
     * In order to calculate the virtual target, you need this hangtime which can only be found after the virtual target
     * is calculated, in order to fix this, this function is recursive, and based on the number of loop passes, it will
     * push the virtual target closer and closer to where it should be.
     * 
     * This has been tested to work at slow velocities.
     * 
     * @param drive
     * @param target
     * @param tuning
     * @param loopPasses
     * @return
     */
    public static Translation2d getVirtualTargetFromTarget(Pose2d pose, ChassisSpeeds speeds, Translation2d target, ShooterTuning tuning, int loopPasses){
        Distance distToHub;

        if(loopPasses == 0) {
            distToHub = Meters.of(target.getDistance(pose.getTranslation()));
        } else{
            distToHub = Meters.of(
                getVirtualTargetFromTarget(pose, speeds, target, tuning, loopPasses - 1).getDistance(pose.getTranslation())
            );
        }

        Time hangTime = Seconds.of(tuning.getShooterParams(distToHub.in(Meters)).hangtime);
        
        return target.minus(
            new Translation2d(
                MetersPerSecond.of(speeds.vxMetersPerSecond).times(hangTime),
                MetersPerSecond.of(speeds.vyMetersPerSecond).times(hangTime)
            )
        );
    }
}
