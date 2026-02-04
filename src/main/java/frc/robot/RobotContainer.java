// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight4;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeConstants;
import static edu.wpi.first.units.Units.Volts;

public class RobotContainer {

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
    private IntakeSubsystem intake_;

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
                case COMPETITION:

                    drivebase_ = new Drive(
                        new GyroIOPigeon2(CompTunerConstants.DrivetrainConstants.Pigeon2Id, CompTunerConstants.kCANBus),
                        ModuleIOTalonFX::new,
                        CompTunerConstants.FrontLeft,
                        CompTunerConstants.FrontRight,
                        CompTunerConstants.BackLeft,
                        CompTunerConstants.BackRight,
                        CompTunerConstants.kSpeedAt12Volts
                    );

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOLimelight4(VisionConstants.frontLimelightName, drivebase_::getRotation)
                    );

                    break;

                case PRACTICE:

                    drivebase_ = new Drive(
                        new GyroIOPigeon2(PracticeTunerConstants.DrivetrainConstants.Pigeon2Id, PracticeTunerConstants.kCANBus),
                        ModuleIOTalonFX::new,
                        PracticeTunerConstants.FrontLeft,
                        PracticeTunerConstants.FrontRight,
                        PracticeTunerConstants.BackLeft,
                        PracticeTunerConstants.BackRight,
                        PracticeTunerConstants.kSpeedAt12Volts
                    );

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOLimelight4(VisionConstants.frontLimelightName, drivebase_::getRotation)
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
                        CompTunerConstants.kSpeedAt12Volts
                    );

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOPhotonSim("front", VisionConstants.frontTransform, drivebase_::getPose, true)
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
                case PRACTICE:
                    drivebase_ = new Drive(
                        new GyroIO() {},
                        ModuleIOReplay::new,
                        PracticeTunerConstants.FrontLeft,
                        PracticeTunerConstants.FrontRight,
                        PracticeTunerConstants.BackLeft,
                        PracticeTunerConstants.BackRight,
                        PracticeTunerConstants.kSpeedAt12Volts
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

        DriveCommands.configure(
            drivebase_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );

        // Choosers
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices");

        // Publish Deploy Directory (for layout/asset downloading)
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        configureBindings();
        configureDriveBindings();
        configureTestModeBindings();
    }

    // Bind robot actions to commands here.
    private void configureBindings() {
        //Testing out each of the commands in the simulator
        gamepad_.a().whileTrue(
            intake_.setRollerVoltageCommand(IntakeConstants.rollerCollectVoltage)
        );

        gamepad_.b().whileTrue(
            intake_.setPivotAngleCommand(IntakeConstants.pivotTargetAngle)
        );

        gamepad_.x().whileTrue(
            intake_.deployIntakeCommand()
        );

        gamepad_.y().whileTrue(
            intake_.stowIntakeCommand()
        );

        gamepad_.leftBumper().whileTrue(
            intake_.stopRollerCommand()
        );

        gamepad_.rightBumper().whileTrue(
            intake_.setRollerVelocityCommand(IntakeConstants.rollerMaxVelocity)
        );

        gamepad_.back().whileTrue(
            intake_.setPivotVoltageCommand(IntakeConstants.pivotVoltage)
        );

        gamepad_.rightTrigger().whileTrue(
            intake_.intakeDeployCommand()
        );

        gamepad_.leftTrigger().whileTrue(
            intake_.stopStowCommand()
        );

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
    }
    
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
