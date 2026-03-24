package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTracer;

/**
 * Object that calculates robot-wide values that change periodically,
 * for example, distance to a field element.
 */
public class RobotState {
    private static Supplier<Pose2d> pose = () -> Pose2d.kZero;

    @AutoLogOutput
    private static Distance hubDistance = Meters.zero();

    private static Rotation2d rotationToHub = Rotation2d.kZero;
    private static boolean inAllianceZone = false;

    /**
     * This method supplies the object with the information it needs for its calculations.
     * @param poseSupplier
     */
    public static void initialize(Supplier<Pose2d> poseSupplier) {
        pose = poseSupplier;
    }

    /**
     * This method gets called periodically so that the variables can update.
     */
    public static void periodic() {
        LoggedTracer.reset();
        
        Pose2d currentPose = pose.get();

        Translation2d hubTranslation =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? ShooterConstants.Positions.blueHubPose
            : ShooterConstants.Positions.redHubPose;
             
        hubDistance = Meters.of(pose.get().getTranslation().getDistance(hubTranslation));
        Logger.recordOutput("RobotState/hubDistance", hubDistance) ;

        Translation2d translationToHub = hubTranslation.minus(currentPose.getTranslation());

        rotationToHub = new Rotation2d(translationToHub.getX(), translationToHub.getY());
        Logger.recordOutput("RobotState/HubRotation", rotationToHub);
        Logger.recordOutput("RobotState/AngleDelta", rotationToHub.minus(currentPose.getRotation()));

        Distance allianceWall =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? ShooterConstants.Positions.blueAllianceWall
                : ShooterConstants.Positions.redAllianceWall;
                
        inAllianceZone = currentPose.getMeasureX().minus(allianceWall).abs(Meters) < ShooterConstants.Positions.spinUpZone.in(Meters);

        LoggedTracer.record("RobotState");
    }


    public static Distance hubDistance() {
        return hubDistance;
    }

    @AutoLogOutput
    public static Rotation2d rotationToHub() {
        return rotationToHub;
    }

    @AutoLogOutput
    public static boolean inAllianceZone() {
        return inAllianceZone;
    }
}
