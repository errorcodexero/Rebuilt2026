package frc.robot.util;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class MapleSimUtil {
    private static SwerveDriveSimulation drivebaseSimulation;

    private static final Command run =
        Commands.run(MapleSimUtil::periodic)
            .ignoringDisable(true)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    /** Starts the MapleSim simulation, kicks off a periodic method. */
    public static void start() {
        var rebuilt = (Arena2026Rebuilt) SimulatedArena.getInstance();
        rebuilt.setEfficiencyMode(false);
        rebuilt.resetFieldForAuto();

        CommandScheduler.getInstance().schedule(run);
    }

    private static void periodic() {
        var arena = SimulatedArena.getInstance();
        
        arena.simulationPeriodic();

        var fuel = arena.getGamePiecesArrayByType("Fuel");

        Logger.recordOutput("MapleSim/Gamepieces", fuel);
        Logger.recordOutput("MapleSim/Pose", getPosition());
        Logger.recordOutput("MapleSim/Score/Red", arena.getScore(Alliance.Red));
        Logger.recordOutput("MapleSim/Score/Blue", arena.getScore(Alliance.Blue));
    }

    public static void createSwerve(DriveTrainSimulationConfig config, Pose2d initialPose) {
        drivebaseSimulation = new SwerveDriveSimulation(config, initialPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(drivebaseSimulation);
    }

    public static GyroSimulation getGyroSimulation() {
        if (drivebaseSimulation == null) throw new IllegalStateException("Drivebase sim is not setup yet!");
        return drivebaseSimulation.getGyroSimulation();
    }

    public static SwerveModuleSimulation[] getModuleSimulations() {
        if (drivebaseSimulation == null) throw new IllegalStateException("Drivebase sim is not setup yet!");
        return drivebaseSimulation.getModules();
    }

    public static Pose2d getPosition() {
        if (drivebaseSimulation == null) throw new IllegalStateException("Drivebase sim is not setup yet!");
        return drivebaseSimulation.getSimulatedDriveTrainPose();
    }

    public static ChassisSpeeds getFieldChassisSpeeds() {
        if (drivebaseSimulation == null) throw new IllegalStateException("Drivebase sim is not setup yet!");
        return drivebaseSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }
 }
