// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.BetaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOMaple;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOMaple;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.MapleSimUtil;
import frc.robot.util.Mechanism3d;

public class RobotContainer {

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
    private IntakeSubsystem intake_;
    private Shooter shooter_;

    // Choosers
    private final LoggedDashboardChooser<Command> autoChooser_;

    // Trigger Devices
    private final CommandXboxController gamepad_ = new CommandXboxController(0);

    public RobotContainer() {
        /**
         * Subsystem setup
         */
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ALPHA:

                    drivebase_ = new Drive(
                        new GyroIOPigeon2(AlphaTunerConstants.DrivetrainConstants.Pigeon2Id, AlphaTunerConstants.kCANBus),
                        ModuleIOTalonFX::new,
                        AlphaTunerConstants.FrontLeft,
                        AlphaTunerConstants.FrontRight,
                        AlphaTunerConstants.BackLeft,
                        AlphaTunerConstants.BackRight,
                        AlphaTunerConstants.kCANBus,
                        AlphaTunerConstants.kSpeedAt12Volts
                    );

                    break;

                case BETA:

                    drivebase_ = new Drive(
                        new GyroIOPigeon2(BetaTunerConstants.DrivetrainConstants.Pigeon2Id, BetaTunerConstants.kCANBus),
                        ModuleIOTalonFX::new,
                        BetaTunerConstants.FrontLeft,
                        BetaTunerConstants.FrontRight,
                        BetaTunerConstants.BackLeft,
                        BetaTunerConstants.BackRight,
                        BetaTunerConstants.kCANBus,
                        BetaTunerConstants.kSpeedAt12Volts
                    );

                    break;

                case SIMBOT:
                    // Sim robot, instantiate physics sim IO implementations
                    // Create and configure a drivetrain simulation configuration
                    DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(COTS.ofMark4(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getKrakenX60(1),
                            COTS.WHEELS.COLSONS.cof,
                            2
                        ))
                        .withTrackLengthTrackWidth(
                            Meters.of(Math.abs(
                                CompTunerConstants.FrontLeft.LocationX -
                                CompTunerConstants.BackLeft.LocationX
                            )),
                            Meters.of(Math.abs(
                                CompTunerConstants.FrontLeft.LocationX -
                                CompTunerConstants.FrontRight.LocationX
                            ))
                        )
                        .withBumperSize(Inches.of(30.75), Inches.of(37.25));

                    // Add sim drivebase to simulation and where modules can get it.
                    // CALL THIS BEFORE CREATING THE DRIVEBASE!
                    MapleSimUtil.createSwerve(config, new Pose2d(2.0, 2.0, Rotation2d.kZero));

                    drivebase_ = new Drive(
                        new GyroIOMaple(),
                        ModuleIOMaple::new,
                        CompTunerConstants.FrontLeft,
                        CompTunerConstants.FrontRight,
                        CompTunerConstants.BackLeft,
                        CompTunerConstants.BackRight,
                        CompTunerConstants.kCANBus,
                        CompTunerConstants.kSpeedAt12Volts
                    );

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOPhotonSim("front", VisionConstants.frontTransform, MapleSimUtil::getPosition, true)
                    );

                    intake_= new IntakeSubsystem(new IntakeIOSim());

                    break;
            }
        }

        /**
         * Empty subsystem setup (required in replay)
         */
        if (drivebase_ == null) { // This will be null in replay, or whenever a case above leaves a subsystem uninstantiated.
            switch (Constants.getRobot()) {
                case ALPHA:
                    drivebase_ = new Drive(
                        new GyroIO() {},
                        ModuleIOReplay::new,
                        AlphaTunerConstants.FrontLeft,
                        AlphaTunerConstants.FrontRight,
                        AlphaTunerConstants.BackLeft,
                        AlphaTunerConstants.BackRight,
                        AlphaTunerConstants.kCANBus,
                        AlphaTunerConstants.kSpeedAt12Volts
                    );

                    break;

                case BETA:
                    drivebase_ = new Drive(
                        new GyroIO() {},
                        ModuleIOReplay::new,
                        BetaTunerConstants.FrontLeft,
                        BetaTunerConstants.FrontRight,
                        BetaTunerConstants.BackLeft,
                        BetaTunerConstants.BackRight,
                        BetaTunerConstants.kCANBus,
                        BetaTunerConstants.kSpeedAt12Volts
                    );

                    break;
                    
                default: // SimBot or Comp Bot
                    drivebase_ = new Drive(
                        new GyroIO() {},
                        ModuleIOReplay::new,
                        CompTunerConstants.FrontLeft,
                        CompTunerConstants.FrontRight,
                        CompTunerConstants.BackLeft,
                        CompTunerConstants.BackRight,
                        CompTunerConstants.kCANBus,
                        CompTunerConstants.kSpeedAt12Volts
                    );

                    break;
            }
        }

        if (vision_ == null) {
            int numCams = switch (Constants.getRobot()) {
                default -> 1;
            };

            CameraIO[] cams = new CameraIO[numCams];
            Arrays.fill(cams, new CameraIO() {});

            vision_ = new AprilTagVision(
                drivebase_::addVisionMeasurement,
                cams
            );
        }

        if (intake_ == null) {
            intake_ = new IntakeSubsystem(new IntakeIO() {});
        }
        
        if (shooter_ == null) {
            shooter_ = new Shooter(new ShooterIO() {});
        }

        DriveCommands.configure(
            drivebase_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );

        // Initialize the visualizers.
        Mechanism3d.measured.zero();
        Mechanism3d.setpoints.zero();

        // Maple Sim
        if (Constants.getRobot() == RobotType.SIMBOT) {
            MapleSimUtil.start();

            gamepad_.a().onTrue(Commands.runOnce(() -> {
                var fuel = new RebuiltFuelOnFly(
                    MapleSimUtil.getPosition().getTranslation(),
                    new Translation2d(), // Initial Robot Position
                    MapleSimUtil.getFieldChassisSpeeds(),
                    MapleSimUtil.getPosition().getRotation(),
                    Meters.of(0.5), // Initial Height
                    MetersPerSecond.of(8),
                    Degrees.of(50)
                )
                .withHitTargetCallBack(() -> System.out.println("Fuel Scored!"))
                .withProjectileTrajectoryDisplayCallBack(
                    poses -> Logger.recordOutput("MapleSim/Trajectory", poses.toArray(Pose3d[]::new))
                );

                SimulatedArena.getInstance().addGamePieceProjectile(fuel);
            }));
        }

        // Choosers
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser_.onChange(auto -> {
            System.err.println("Auto \"" + auto.getName() + "\" selected!");
            // This should be used to set up robot position setting, initialization, etc.
        });

        // Publish Deploy Directory (for layout/asset downloading)
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        configureBindings();
        configureDriveBindings();
        configureTestModeBindings();
    }

    // Bind robot actions to commands here.
    private void configureBindings() {
        //Testing out each of the commands in the simulator
        gamepad_.x().onTrue(Commands.either(
            intake_.deployIntakeCommand(),
            intake_.stowIntakeCommand(),
            intake_::isIntakeStowed
        ));

    }

    private void configureDriveBindings() {
        // Default command, normal field-relative drive
        drivebase_.setDefaultCommand(DriveCommands.joystickDrive());

        // Slow Mode, during left bumper
        gamepad_.leftBumper().whileTrue(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier));

        // Switch to X pattern / brake while X button is pressed
        gamepad_.x().whileTrue(drivebase_.stopWithXCmd());

        // Robot Relative
        gamepad_.povUp().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

        gamepad_.povDown().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0),
                        RadiansPerSecond.zero()));

        gamepad_.povLeft().whileTrue(
                drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero()));

        gamepad_.povRight().whileTrue(
                drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(),
                        RadiansPerSecond.zero()));

        // Robot relative diagonal
        gamepad_.povUpLeft().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero()));

        gamepad_.povUpRight().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero()));

        gamepad_.povDownLeft().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero()));

        gamepad_.povDownRight().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero()));

        // Reset gyro to 0° when Y & B button is pressed
        gamepad_.y().and(gamepad_.b()).onTrue(drivebase_.resetGyroCmd());
    }

    private void configureTestModeBindings() {
        gamepad_.back().and(RobotModeTriggers.test()).toggleOnTrue(
            DriveCommands.wheelRadiusCharacterization(drivebase_)
        );

        LoggedNetworkNumber shooterVelocity = new LoggedNetworkNumber("Tuning/Shooter/TargetShooterRPS", 0);
        LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("Tuning/Shooter/TargetHoodAngle", ShooterConstants.SoftwareLimits.hoodMinAngle);

        gamepad_.a().and(RobotModeTriggers.test()).toggleOnTrue(
            shooter_.runDynamicSetpoint(() -> RotationsPerSecond.of(shooterVelocity.get()), () -> Degrees.of(hoodAngle.get()))
        );
    }
    
    public Command getAutonomousCommand() {
        return DriveCommands.feedforwardCharacterization(drivebase_);
    }
}
