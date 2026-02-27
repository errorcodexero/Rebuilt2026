package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.drive.Drive;

public class VirtualTarget {
    public static Translation2d getVirtualTargetFromTarget(Drive drive, Translation2d target, Time hangTime){
        return target.minus(
                new Translation2d(
                    MetersPerSecond.of(drive.getFieldChassisSpeeds().vxMetersPerSecond).times(hangTime),
                    MetersPerSecond.of(drive.getFieldChassisSpeeds().vyMetersPerSecond).times(hangTime)
                )
            );
    }
}
