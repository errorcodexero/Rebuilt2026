package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterTuning;
import frc.robot.util.LoggedTracer;
import frc.robot.util.VirtualTarget;

/**
 * Object that calculates robot-wide values that change periodically,
 * for example, distance to a field element.
 */
public class RobotState {
    private static Supplier<Pose2d> pose = null;
    private static Supplier<ChassisSpeeds> speed = null;
    private static Supplier<ShooterTuning> tuning = null;

    private static boolean updatedRealHub = false;
    private static boolean updatedVirtualHub = false;
    private static boolean updatedVirtualFerry = false;

    private static Pose2d currentPose = Pose2d.kZero;
    private static Translation2d hubTranslation = Translation2d.kZero;
    private static Distance allianceWall = Meters.zero();

    private static boolean inAllianceZone = false;

    private static Distance hubDistance = Meters.zero();
    private static Rotation2d rotationToHub = Rotation2d.kZero;

    private static Distance virtualHubDistance = Meters.zero();
    private static Rotation2d rotationToVirtualHub = Rotation2d.kZero;

    private static Distance virtualFerryDistance = Meters.zero();
    private static Rotation2d rotationToVirtualFerry = Rotation2d.kZero;

    /**
     * This method supplies the object with the information it needs for its calculations.
     * @param poseSupplier
     */
    public static void initialize(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Supplier<ShooterTuning> tuningSupplier
    ) {
        pose = poseSupplier;
        speed = speedSupplier;
        tuning = tuningSupplier;
    }

    /**
     * This method gets called periodically so that the variables can update.
     */
    public static void periodic() {
        LoggedTracer.reset();

        // If not initialized, do nothing.
        if (
            pose == null ||
            speed == null ||
            tuning == null
        ) return;

        updatedRealHub = false;
        updatedVirtualHub = false;
        updatedVirtualFerry = false;

        currentPose = pose.get();

        hubTranslation =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? ShooterConstants.Positions.blueHubPose
            : ShooterConstants.Positions.redHubPose;

        allianceWall =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? ShooterConstants.Positions.blueAllianceWall
                : ShooterConstants.Positions.redAllianceWall;
                
        inAllianceZone = currentPose.getMeasureX().minus(allianceWall).abs(Meters) < ShooterConstants.Positions.spinUpZone.in(Meters);

        LoggedTracer.record("RobotState/InAllianceZone");

        // Log Outputs
        Logger.recordOutput("RobotState/HubDistance", hubDistance);
        Logger.recordOutput("RobotState/VirtualHubDistance", virtualHubDistance);
    }

    private static void updateRealHub() {
        LoggedTracer.reset();

        hubDistance = Meters.of(currentPose.getTranslation().getDistance(hubTranslation));

        Translation2d translationToHub = hubTranslation.minus(currentPose.getTranslation());

        rotationToHub = new Rotation2d(translationToHub.getX(), translationToHub.getY());

        updatedRealHub = true;

        LoggedTracer.record("RobotState/UpdateRealHub");
    }

    private static void updateVirtualHub() {
        LoggedTracer.reset();

        Pose2d currentPose = pose.get();

        Translation2d virtualHub =
            VirtualTarget.getVirtualTargetFromTarget(currentPose, speed.get(), hubTranslation, tuning.get(), ShooterConstants.hangtimeLoopPasses);

        virtualHubDistance = Meters.of(currentPose.getTranslation().getDistance(virtualHub));

        Translation2d translationToTarget = virtualHub.minus(currentPose.getTranslation());
        rotationToVirtualHub = new Rotation2d(translationToTarget.getX(), translationToTarget.getY());

        Logger.recordOutput("RobotState/VirtualHub", virtualHub);

        updatedVirtualHub = true;

        LoggedTracer.record("RobotState/UpdateVirtualHub");
    }

    private static void updateVirtualFerry() {
        LoggedTracer.reset();

        Translation2d rightTarget =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? ShooterConstants.Positions.blueTargetRight
                : ShooterConstants.Positions.redTargetRight;

        Translation2d leftTarget =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? ShooterConstants.Positions.blueTargetLeft
                : ShooterConstants.Positions.redTargetLeft;

        Translation2d realTarget = currentPose.getY() < ShooterConstants.Positions.centerLineY
            ? rightTarget
            : leftTarget;

        Translation2d virtualTarget =
            VirtualTarget.getVirtualTargetFromTarget(currentPose, speed.get(), realTarget, tuning.get(), ShooterConstants.hangtimeLoopPasses);
            
        Translation2d botToTarget = virtualTarget.minus(currentPose.getTranslation());

        virtualFerryDistance = Meters.of(virtualTarget.getDistance(currentPose.getTranslation()));
        rotationToVirtualFerry =  new Rotation2d(botToTarget.getX(), botToTarget.getY());

        updatedVirtualFerry = true;

        LoggedTracer.record("RobotState/UpdateVirtualFerry");
    }


    public static Distance hubDistance() {
        if (!updatedRealHub) updateRealHub();
        return hubDistance;
    }

    public static Rotation2d rotationToHub() {
        if (!updatedRealHub) updateRealHub();
        return rotationToHub;
    }

    public static Distance virtualHubDistance() {
        if (!updatedVirtualHub) updateVirtualHub();
        return virtualHubDistance;
    }

    public static Rotation2d rotationToVirtualHub() {
        if (!updatedVirtualHub) updateVirtualHub();
        return rotationToVirtualHub;
    }

    public static Distance virtualFerryDistance() {
        if (!updatedVirtualFerry) updateVirtualFerry();
        return virtualFerryDistance;
    }

    public static Rotation2d rotationToVirtualFerry() {
        if (!updatedVirtualFerry) updateVirtualFerry();
        return rotationToVirtualFerry;
    }

    public static boolean inAllianceZone() {
        return inAllianceZone;
    }
}
