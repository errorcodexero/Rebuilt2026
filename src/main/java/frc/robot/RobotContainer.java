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
import frc.robot.autos.FlywheelVoltageAuto;
import frc.robot.commands.auto.ShooterTuningAuto;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeSysIdCommands;
import frc.robot.commands.shooter.ShooterSysIdCommands;
import frc.robot.commands.hopper.HopperSysIdCommands;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOKraken;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight4;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;

public class RobotContainer {

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
    private Shooter shooter_;
    private Intake intake_;
    private Hopper hopper_;

    // Choosers
    private final LoggedDashboardChooser<Command> autoChooser_;
    private final LoggedDashboardChooser<SysIdMechanism> sysIdMechanismChooser_;

    // Trigger Devices
    private final CommandXboxController gamepad_ = new CommandXboxController(0);

    public RobotContainer() {

        /**
         * Subsystem setup
         */
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case COMPETITION:
                    // drivebase_ = new Drive(
                    //     new GyroIOPigeon2(CompTunerConstants.DrivetrainConstants.Pigeon2Id, CompTunerConstants.kCANBus),
                    //     ModuleIOTalonFX::new,
                    //     CompTunerConstants.FrontLeft,
                    //     CompTunerConstants.FrontRight,
                    //     CompTunerConstants.BackLeft,
                    //     CompTunerConstants.BackRight,
                    //     CompTunerConstants.kSpeedAt12Volts
                    // );

                    // vision_ = new AprilTagVision(
                    //     drivebase_::addVisionMeasurement,
                    //     new CameraIOLimelight4(VisionConstants.frontLimelightName, drivebase_::getRotation)
                    // );

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

                    shooter_ = new Shooter(new ShooterIOKraken());
                    // intake_ = new Intake(new IntakeIOKraken());
                    // hopper_ = new Hopper(new HopperIOKraken());

                    break;

                case SIMREAL:
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

                    shooter_ = new Shooter(new ShooterIOKraken());                
                    break ;

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

                    shooter_ = new Shooter(new ShooterIOKraken());
                    intake_ = new Intake(new IntakeIOKraken());
                    hopper_ = new Hopper(new HopperIOKraken());

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

                    shooter_ = new Shooter(new ShooterIOSim());
                    intake_ = new Intake(new IntakeIOSim());
                    hopper_ = new Hopper(new HopperIOSim());

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

        if (shooter_ == null) {
            shooter_ = new Shooter(new ShooterIO() {});
        }

        if (intake_ == null) {
            intake_ = new Intake(new IntakeIO() {});
        }

        if (hopper_ == null) {
            hopper_ = new Hopper(new HopperIO() {});
        }

        DriveCommands.configure(
            drivebase_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );

        // Choosers
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser_.addDefaultOption("None", Commands.print("No autonomous command configured"));
        autoChooser_.addOption("Flywheel Veloticy", ShooterTuningAuto.getCommand(shooter_));
        autoChooser_.addOption("Flywheel Voltage", FlywheelVoltageAuto.getCommand(shooter_));

        // SysId mechanism chooser for test mode
        sysIdMechanismChooser_ = new LoggedDashboardChooser<>("SysId Mechanism");
        sysIdMechanismChooser_.addDefaultOption(
            SysIdMechanism.INTAKE_DEPLOY.toString(), SysIdMechanism.INTAKE_DEPLOY);
        sysIdMechanismChooser_.addOption(
            SysIdMechanism.INTAKE_SPINNER.toString(), SysIdMechanism.INTAKE_SPINNER);
        sysIdMechanismChooser_.addOption(
            SysIdMechanism.SHOOTER_FLYWHEEL.toString(), SysIdMechanism.SHOOTER_FLYWHEEL);
        sysIdMechanismChooser_.addOption(
            SysIdMechanism.HOPPER_AGITATOR.toString(), SysIdMechanism.HOPPER_AGITATOR);
        sysIdMechanismChooser_.addOption(
            SysIdMechanism.HOPPER_FEEDER.toString(), SysIdMechanism.HOPPER_FEEDER);

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
        // Drive characterization (existing)
        gamepad_.back().and(RobotModeTriggers.test()).toggleOnTrue(
            DriveCommands.wheelRadiusCharacterization(drivebase_)
        );

        // SysId characterization commands - only available in test mode
        // Quasistatic tests (slow ramp)
        gamepad_.a().and(RobotModeTriggers.test()).whileTrue(
            Commands.either(
                getSysIdQuasistaticForward(),
                Commands.none(),
                () -> sysIdMechanismChooser_.get() != null
            )
        );

        gamepad_.b().and(RobotModeTriggers.test()).whileTrue(
            Commands.either(
                getSysIdQuasistaticReverse(),
                Commands.none(),
                () -> sysIdMechanismChooser_.get() != null
            )
        );

        // Dynamic tests (fast step)
        gamepad_.x().and(RobotModeTriggers.test()).whileTrue(
            Commands.either(
                getSysIdDynamicForward(),
                Commands.none(),
                () -> sysIdMechanismChooser_.get() != null
            )
        );

        gamepad_.y().and(RobotModeTriggers.test()).whileTrue(
            Commands.either(
                getSysIdDynamicReverse(),
                Commands.none(),
                () -> sysIdMechanismChooser_.get() != null
            )
        );
    }

    /**
     * Get the quasistatic forward SysId command for the selected mechanism.
     */
    private Command getSysIdQuasistaticForward() {
        SysIdMechanism mechanism = sysIdMechanismChooser_.get();
        if (mechanism == null) return Commands.none();

        switch (mechanism) {
            case INTAKE_DEPLOY:
                return IntakeSysIdCommands.deployQuasistaticForward(intake_);
            case INTAKE_SPINNER:
                return IntakeSysIdCommands.spinnerQuasistaticForward(intake_);
            case SHOOTER_FLYWHEEL:
                return ShooterSysIdCommands.flywheelQuasistaticForward(shooter_);
            case HOPPER_AGITATOR:
                return HopperSysIdCommands.agitatorQuasistaticForward(hopper_);
            case HOPPER_FEEDER:
                return HopperSysIdCommands.feederQuasistaticForward(hopper_);
            default:
                return Commands.none();
        }
    }

    /**
     * Get the quasistatic reverse SysId command for the selected mechanism.
     */
    private Command getSysIdQuasistaticReverse() {
        SysIdMechanism mechanism = sysIdMechanismChooser_.get();
        if (mechanism == null) return Commands.none();

        switch (mechanism) {
            case INTAKE_DEPLOY:
                return IntakeSysIdCommands.deployQuasistaticReverse(intake_);
            case INTAKE_SPINNER:
                return IntakeSysIdCommands.spinnerQuasistaticReverse(intake_);
            case SHOOTER_FLYWHEEL:
                return ShooterSysIdCommands.flywheelQuasistaticReverse(shooter_);
            case HOPPER_AGITATOR:
                return HopperSysIdCommands.agitatorQuasistaticReverse(hopper_);
            case HOPPER_FEEDER:
                return HopperSysIdCommands.feederQuasistaticReverse(hopper_);
            default:
                return Commands.none();
        }
    }

    /**
     * Get the dynamic forward SysId command for the selected mechanism.
     */
    private Command getSysIdDynamicForward() {
        SysIdMechanism mechanism = sysIdMechanismChooser_.get();
        if (mechanism == null) return Commands.none();

        switch (mechanism) {
            case INTAKE_DEPLOY:
                return IntakeSysIdCommands.deployDynamicForward(intake_);
            case INTAKE_SPINNER:
                return IntakeSysIdCommands.spinnerDynamicForward(intake_);
            case SHOOTER_FLYWHEEL:
                return ShooterSysIdCommands.flywheelDynamicForward(shooter_);
            case HOPPER_AGITATOR:
                return HopperSysIdCommands.agitatorDynamicForward(hopper_);
            case HOPPER_FEEDER:
                return HopperSysIdCommands.feederDynamicForward(hopper_);
            default:
                return Commands.none();
        }
    }

    /**
     * Get the dynamic reverse SysId command for the selected mechanism.
     */
    private Command getSysIdDynamicReverse() {
        SysIdMechanism mechanism = sysIdMechanismChooser_.get();
        if (mechanism == null) return Commands.none();

        switch (mechanism) {
            case INTAKE_DEPLOY:
                return IntakeSysIdCommands.deployDynamicReverse(intake_);
            case INTAKE_SPINNER:
                return IntakeSysIdCommands.spinnerDynamicReverse(intake_);
            case SHOOTER_FLYWHEEL:
                return ShooterSysIdCommands.flywheelDynamicReverse(shooter_);
            case HOPPER_AGITATOR:
                return HopperSysIdCommands.agitatorDynamicReverse(hopper_);
            case HOPPER_FEEDER:
                return HopperSysIdCommands.feederDynamicReverse(hopper_);
            default:
                return Commands.none();
        }
    }
    
    public Command getAutonomousCommand() {
        return autoChooser_.get();
    }
}
