// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.BetaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterTuningCommand;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;

public class RobotContainer {

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
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
                    drivebase_ = new Drive(
                        new GyroIO() {},
                        ModuleIOSim::new,
                        CompTunerConstants.FrontLeft,
                        CompTunerConstants.FrontRight,
                        CompTunerConstants.BackLeft,
                        CompTunerConstants.BackRight,
                        CompTunerConstants.kCANBus,
                        CompTunerConstants.kSpeedAt12Volts
                    );

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOPhotonSim("front", VisionConstants.frontTransform, drivebase_::getPose, true)
                    );

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

        if (shooter_ == null) {
            shooter_ = new Shooter(new ShooterIO() {});
        }

        DriveCommands.configure(
            drivebase_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );

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

        gamepad_.a().and(RobotModeTriggers.test()).toggleOnTrue(
            new ShooterTuningCommand(shooter_)
        );
    }
    
    public Command getAutonomousCommand() {
        return DriveCommands.feedforwardCharacterization(drivebase_);
    }
}
