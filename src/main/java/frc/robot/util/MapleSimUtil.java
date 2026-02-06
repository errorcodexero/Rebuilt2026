package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class MapleSimUtil {
    private static SwerveDriveSimulation drivebaseSimulation;

    private static IntakeSimulation intakeSimulation;

    private static Angle hoodAngle = Degrees.zero();
    private static boolean shooterRunning = false;
    private static AngularVelocity shooterVelocity = RadiansPerSecond.zero();

    private static final Distance shooterRadius = Centimeters.of(5);

    private static final Command run =
        Commands.run(MapleSimUtil::periodic)
            .ignoringDisable(true)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    private static final Command runShooter =
        Commands.runOnce(() -> {
            if (getRemainingGamepieces() == 0) return;

            var fuelLeft = createProjectile(Inches.of(3.7));
            loseGamepiece();
            SimulatedArena.getInstance().addGamePieceProjectile(fuelLeft);

            if (getRemainingGamepieces() == 0) return;
            
            var fuelRight = createProjectile(Inches.of(-3.7));
            loseGamepiece();
            SimulatedArena.getInstance().addGamePieceProjectile(fuelRight);            
        })
        .andThen(Commands.waitTime(Milliseconds.of(1000 / 10)))
        .repeatedly();

    /** Starts the MapleSim simulation, kicks off a periodic method. */
    public static void start() {
        var rebuilt = (Arena2026Rebuilt) SimulatedArena.getInstance();
        rebuilt.setEfficiencyMode(Constants.spawnLessFuelInSim);
        rebuilt.resetFieldForAuto();

        new Trigger(() -> shooterRunning).whileTrue(runShooter);

        CommandScheduler.getInstance().schedule(run);
    }

    private static void periodic() {
        var arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
        
        arena.simulationPeriodic();

        var fuel = arena.getGamePiecesArrayByType("Fuel");

        Logger.recordOutput("MapleSim/Gamepieces", fuel);
        Logger.recordOutput("MapleSim/Pose", getPosition());
        Logger.recordOutput("MapleSim/Score/Red", arena.getScore(Alliance.Red));
        Logger.recordOutput("MapleSim/Score/Blue", arena.getScore(Alliance.Blue));
        Logger.recordOutput("MapleSim/FuelRemaining", getRemainingGamepieces());
        Logger.recordOutput("MapleSim/Intaking", intakeSimulation.isRunning());
        Logger.recordOutput("MapleSim/ActiveHub", arena.isActive(true) ? Alliance.Blue : Alliance.Red);
    }

    public static SwerveDriveSimulation createSwerve(DriveTrainSimulationConfig config, Pose2d initialPose) {
        drivebaseSimulation = new SwerveDriveSimulation(config, initialPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(drivebaseSimulation);
        return drivebaseSimulation;
    }

    public static IntakeSimulation createIntake() {
        if (drivebaseSimulation == null) throw new IllegalStateException("Intake cannot be created before swerve is!");

        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            drivebaseSimulation,
            Inches.of(29.5),
            Inches.of(8.125),
            IntakeSide.FRONT,
            80
        );

        return intakeSimulation;
    }

    public static GamePieceProjectile createProjectile(Distance yOffset) {
        return new RebuiltFuelOnFly(
            getPosition().getTranslation(),
            new Translation2d(Inches.of(-8), Inches.of(3.7)), // Initial Shooter Position
            getFieldChassisSpeeds(),
            getPosition().getRotation(),
            Inches.of(18.5), // Initial Height
            MetersPerSecond.of(shooterVelocity.in(RadiansPerSecond) * shooterRadius.in(Meters)),
            hoodAngle
        )
        .withHitTargetCallBack(() -> System.out.println("Fuel Scored!"))
        .withProjectileTrajectoryDisplayCallBack(
            poses -> Logger.recordOutput("MapleSim/Trajectory", poses.toArray(Pose3d[]::new))
        );
    }

    public static Pose2d getPosition() {
        return drivebaseSimulation.getSimulatedDriveTrainPose();
    }

    public static ChassisSpeeds getFieldChassisSpeeds() {
        return drivebaseSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    public static int getRemainingGamepieces() {
        return intakeSimulation.getGamePiecesAmount();
    }

    public static void setIntakeRunning(boolean running) {
        if (running) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    public static void loseGamepiece() {
        intakeSimulation.obtainGamePieceFromIntake();
    }

    public static void setShooterRunning(boolean running) {
        shooterRunning = running;
    }

    public static void setShooterVelocity(AngularVelocity velocity) {
        shooterVelocity = velocity;
    }

    public static void setHoodAngle(Angle angle) {
        hoodAngle = angle;
    }

    public static GyroSimulation getGyroSimulation() {
        return drivebaseSimulation.getGyroSimulation();
    }

    public static SwerveModuleSimulation[] getModuleSimulations() {
        return drivebaseSimulation.getModules();
    }
 }
