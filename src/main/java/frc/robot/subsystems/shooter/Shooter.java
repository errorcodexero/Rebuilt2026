package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterTuning.ShooterParams;
import frc.robot.util.LoggedTracer;
import frc.robot.util.MapleSimUtil;
import frc.robot.util.Mechanism3d;


public class Shooter extends SubsystemBase {
    
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final HoodIO hoodIO;
    private final HoodInputsAutoLogged hoodInputs = new HoodInputsAutoLogged();

    private final Debouncer statusDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private final Alert disconnectionAlert =
        new Alert("One or more shooter motors are disconnected!", AlertType.kError);

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    private AngularVelocity shooterTarget = RadiansPerSecond.zero();
    private Angle hoodTarget = Radians.zero();

    @AutoLogOutput
    private int tuningIndex_ = 0 ;
    private List<ShooterTuning> tunings_ = new ArrayList<ShooterTuning>() ;

    @AutoLogOutput
    private boolean hoodParked = false;

    public Shooter(ShooterIO ioShooter, HoodIO ioHood, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.shooterIO = ioShooter;
        this.hoodIO = ioHood;

        loadTunings();
        if (tunings_.isEmpty()) {
            throw new RuntimeException("No shooter tunings found! Please add a json file to the tuning folder with shooter values.");
        }

        tuningIndex_ = 0 ;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
    }    
    
    @Override
    public void periodic() {
        LoggedTracer.reset();

        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Shooter/Hood", hoodInputs);

        disconnectionAlert.set(!statusDebounce.calculate(shooterInputs.allConnected));

        Mechanism3d.measured.setHood(hoodInputs.position);
        Mechanism3d.setpoints.setHood(hoodTarget);

        if (Constants.getMode() == Mode.SIM) {
            MapleSimUtil.setShooterVelocity(shooterInputs.wheelVelocity);
            MapleSimUtil.setHoodAngle(hoodInputs.position);
        }

        // Hood Protection
        Pose2d pose = poseSupplier.get();
        ChassisSpeeds speed = speedsSupplier.get();
        Pose2d trench = pose.nearest(FieldConstants.trenches);
        Distance nearestDistance = Meters.of(pose.getTranslation().getDistance(trench.getTranslation()));

        if (
            nearestDistance.lte(ShooterConstants.allowedTrenchDistance) && // Too Close
            trench.getMeasureX().minus(pose.getMeasureX()).in(Meters) * speed.vxMetersPerSecond > 0 // And moving towards it
        ) {
            hoodIO.goToAngle(ShooterConstants.hoodParkedAngle);
            hoodParked = true;
        } else {
            hoodParked = false;
        }

        Logger.recordOutput("Shooter/VelocitySetPoint", shooterTarget);
        Logger.recordOutput("Shooter/HoodSetPoint", hoodTarget);

        LoggedTracer.record("ShooterPeriodic");
    }

    // Shooter Methods

    private void loadTunings() {
        File tuningDir = new File(Filesystem.getDeployDirectory(), "tuning") ;
        if (tuningDir.isDirectory()) {
            File[] jsonFiles = tuningDir.listFiles((dir, name) -> name.endsWith(".json")) ;
            if (jsonFiles != null) {
                for (File f : jsonFiles) {
                    String name = f.getName().replace(".json", "") ;
                    tunings_.add(new ShooterTuning(name)) ;
                }
            }
        }
    }

    private void setShooterVelocity(AngularVelocity vel) {
        shooterTarget = vel;
        shooterIO.setVelocity(vel);
    }

    private void setShooterVoltage(Voltage vol) {
        shooterTarget = RotationsPerSecond.zero();
        shooterIO.setVoltage(vol);
    }

    private void stopShooter() {
        shooterTarget = RotationsPerSecond.zero();
        shooterIO.stop();
    }

    public AngularVelocity getShooterVelocity() {
        return shooterInputs.wheelVelocity;
    }

    public Voltage getShooterVoltage() {
        return shooterInputs.shooter1Voltage;
    }
    
    public boolean isShooterReady() {
        return shooterInputs.wheelVelocity.isNear(shooterTarget, ShooterConstants.shooterTolerance); 
    }

    public Current getShooterCurrent() {
        return (shooterInputs.shooter1Current)
            .plus(shooterInputs.shooter2Current)
            .plus(shooterInputs.shooter3Current);
    }

    private void setHoodAngle(Angle pos) {
        hoodTarget = pos;
        if (hoodParked) return;
        hoodIO.goToAngle(pos);
    }

    private void setSetpoints(AngularVelocity vel, Angle pos) {
        shooterTarget = vel;
        setShooterVelocity(shooterTarget);
        hoodTarget = pos;
        setHoodAngle(hoodTarget);
    }

    private ShooterTuning getTuning() {
        return tunings_.get(tuningIndex_);
    }

    // Commands
    public Command cycleTuning() {
        return runOnce(() -> {
            tuningIndex_ = (tuningIndex_ + 1) % tunings_.size();
        }).ignoringDisable(true);
    }

    public Command reloadTunings() {
        return runOnce(() -> {
            tunings_.clear();
            loadTunings();
            if (tunings_.isEmpty()) {
                throw new RuntimeException("No shooter tunings found! Please add a json file to the tuning folder with shooter values.");
            }
            tuningIndex_ = 0;
        }).ignoringDisable(true);
    }

    /**
     * The command that the shooter can run whenever its not shooting to manage
     * things like going to different hood angles to get ready to shoot,
     * or lowering the hood under the trench.
     * @return A command that does so.
     */
    public Command awaitShooting(Supplier<Pose2d> robotPose) {
        return runDynamicSetpoints(
            // () -> RotationsPerSecond.of(
            //     RobotState.inAllianceZone()
            //         ? getTuning().getShooterParams(RobotState.hubDistance().in(Meters)).velocity
            //         : 0.0
            // ),      
            () -> RotationsPerSecond.of(0.0) ,      
            () -> {
                Pose2d pose = robotPose.get();
                Pose2d nearestTrench = pose.nearest(FieldConstants.trenches);
                Distance nearestDistance = Meters.of(pose.getTranslation().getDistance(nearestTrench.getTranslation()));

                if (nearestDistance.lte(ShooterConstants.allowedTrenchDistance)) {
                    return Degrees.zero();
                }

                var params = getTuning().getShooterParams(RobotState.hubDistance().in(Meters));
                return Degrees.of(params.hood);
            }
        );
    }

    public Command runToSetpointsCmd(AngularVelocity vel, Angle pos) {
        return startEnd(() -> setSetpoints(vel, pos), this::stopShooter).withName("Set Shooter Setpoints");
    }

    public Command runToVelocityCmd(AngularVelocity vel) {
        return startEnd(() -> setShooterVelocity(vel), this::stopShooter).withName("Set Shooter Velocity");
    }

    public Command requestToVelocityCmd(AngularVelocity vel) {
        return startEnd(() -> setShooterVelocity(vel), this::stopShooter);
    }

    public Command stopCmd() {
        return runOnce(() -> stopShooter())
            .andThen(Commands.waitUntil(this::isShooterReady)).withName("Stop Shooter");
    }

    public Command runVoltageCmd(Voltage vol) {
        return runOnce(() -> setShooterVoltage(vol)).withName("Set Shooter Voltage");
    }

    public Command hoodToPosCmd(Angle pos) {
        return runOnce(() -> setHoodAngle(pos)).withName("Set Hood Position");
    }

    /**
     * Calculates Velocity and Hood Angle based on distance and Shoots
     * 
     * 
     */
    public Command shootAtDistance(Supplier<Distance> distance, Hopper hopper, IntakeSubsystem intake) {
        Supplier<ShooterParams> shooterParams =
            () -> getTuning().getShooterParams(distance.get().in(Meters));

        return Commands.parallel(
            run(() -> setSetpoints(
                RotationsPerSecond.of(shooterParams.get().velocity).times(ShooterConstants.shooterVelocityMultiplierWhileFeederSlow),
                Degrees.of(shooterParams.get().hood)
            ))
            .alongWith(Commands.runOnce(() -> Logger.recordOutput("Shooting/Boost", true)))
            .until(() -> hopper.getTargetPercent() > 0.9)
            .andThen(runDynamicSetpoints(
                () -> RotationsPerSecond.of(shooterParams.get().velocity),
                () -> Degrees.of(shooterParams.get().hood)
            )
            .alongWith(Commands.runOnce(() -> Logger.recordOutput("Shooting/Boost", false)))),

            hopper.preShoot().until(this::isShooterReady).andThen(hopper.forwardFeed()),

            intake.enableShootMode()
        );
    }

    /**
     * Ejects balls from the shooter at a low velocity to get them out of the shooter without shooting them towards the target.
     * @return
     */
    public Command ejectUp() {
        return startEnd(() -> setShooterVelocity(ShooterConstants.ejectVelocity), this::stopShooter);
    }

    /**
     * Idles the shooter at 0 velocity and the hood at the parked position.
     * @return
     */
    public Command idleCommand() {
        return runToSetpointsCmd(
            RotationsPerSecond.of(0.0),
            ShooterConstants.hoodParkedAngle
        );
    }

    /**
     * Runs supplied setpoints until the command ends, then stops.
     * @param vel
     * @param pos
     * @return
     */
    public Command runDynamicSetpoints(Supplier<AngularVelocity> vel, Supplier<Angle> pos ) {
        return runEnd(() -> setSetpoints(vel.get(), pos.get()), this::stopShooter);
    }

    /**
     * Runs supplied voltage until the command ends, then stops.
     * @param voltage
     * @return
     */
    public Command runDynamicVoltage(Supplier<Voltage> voltage) {
        return runEnd(() -> setShooterVoltage(voltage.get()), this::stopShooter);
    }   

    // Sys ID
    public Command shooterSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return shooterIdRoutine().quasistatic(dir);
    }

    public Command shooterSysIdDynamic(SysIdRoutine.Direction dir) {
        return shooterIdRoutine().dynamic(dir);
    }

    private SysIdRoutine shooterIdRoutine() {
        Voltage step = Volts.of(7);
        Time to = Seconds.of(10.0);
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, (state) -> Logger.recordOutput("SysIdTestState", state.toString()));

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                (volts) -> shooterIO.setVoltage(volts),
                null,
                this);

        return new SysIdRoutine(cfg, mfg);
    }

    public Command testCommand(Hopper hopper) {
        LoggedNetworkNumber shooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/TargetShooterRPS", 0);
        LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("/Tuning/Shooter/TargetHoodAngle", ShooterConstants.hoodMinAngle.in(Degrees));
        LoggedNetworkNumber feederVelocity = new LoggedNetworkNumber("/Tuning/Shooter/FeederRPS", 40.0);
        LoggedNetworkNumber scramblerVelocity = new LoggedNetworkNumber("/Tuning/Shooter/ScramblerRPS", 10.0);
        
        return Commands.parallel(
            runDynamicSetpoints(() -> RotationsPerSecond.of(shooterVelocity.get()), () -> Degrees.of(hoodAngle.get())),
            hopper.dynamicFeed(
                () -> RotationsPerSecond.of(feederVelocity.get()),
                () -> RotationsPerSecond.of(scramblerVelocity.get())
            )
        );
    }

    /**
     * Command that allows you to tune the hood calibration values. These will persist throughout the
     * robot run, but will need to be set in the constants to persist between robot reboots.
     * @return
     */
    public Command hoodCalibration() {
        return defer(() -> {
            LoggedNetworkNumber leftOffset = new LoggedNetworkNumber("Tuning/Hood/LeftOffset", 0.0);
            LoggedNetworkNumber rightOffset = new LoggedNetworkNumber("Tuning/Hood/RightOffset", 0.0);

            return run(() -> {
                hoodIO.applyCalibration(leftOffset.get(), rightOffset.get());
                hoodIO.goToAngle(Degrees.zero());
            });
        });
    }


    /**
     * Shoot balls from the shooter until the command ends.
     * @return
     */
    public Command shootCmd(Hopper hopper) {
        return Commands.parallel(
            runDynamicSetpoints(() -> RPM.of(5000), () -> Degrees.of(30)),
            hopper.forwardFeed()
        );
    }

    public Command ferryToOutpost(Drive drive, Hopper hopper, IntakeSubsystem intake, DoubleSupplier xSupplier, DoubleSupplier ySupplier){
        return new ConditionalCommand(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    ()-> 0,
                    () -> 0,
                    () -> {
                        Translation2d ferryTarget = ShooterConstants.FerryPositions.blueOutpostTarget;

                        var targetTranslation= ferryTarget.minus(drive.getPose().getTranslation());
                        var targetRotation= new Rotation2d(targetTranslation.getX(), targetTranslation.getY());
                        return targetRotation;
                    }
                ),
                shootCmd(hopper)
            ),
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    ()-> 0,
                    () -> 0,
                    () -> {
                        Translation2d ferryTarget= ShooterConstants.FerryPositions.redOutpostTarget;

                        var targetTranslation= ferryTarget.minus(drive.getPose().getTranslation());
                        var targetRotation= new Rotation2d(targetTranslation.getX(), targetTranslation.getY());
                        return targetRotation;
                    }
                ),
                shootCmd(hopper)
            ),

            () -> {
                if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue){
                    return true;
                }
                return false;
            }
        );
    }

    public Command ferryOnMove(Drive drive, Hopper hopper, DoubleSupplier xSupplier, DoubleSupplier ySupplier, IntakeSubsystem intake){
        return new ConditionalCommand(

            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    xSupplier,
                    ySupplier,
                    () -> {
                        Translation2d ferryTarget= 
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? ShooterConstants.FerryPositions.blueOutpostTarget
                        : ShooterConstants.FerryPositions.redOutpostTarget;

                        var targetTranslation= ferryTarget.minus(drive.getPose().getTranslation());
                        var targetRotation= new Rotation2d(targetTranslation.getX(), targetTranslation.getY());
                        return targetRotation;
                }),
                intake.intakeSequence(),
                shootCmd(hopper)
            ),
            
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    xSupplier,
                    ySupplier,
                    () -> {
                        Translation2d ferryTarget= 
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? ShooterConstants.FerryPositions.blueOutpostTarget
                        : ShooterConstants.FerryPositions.redOutpostTarget;

                        var targetTranslation= ferryTarget.minus(drive.getPose().getTranslation());
                        var targetRotation= new Rotation2d(targetTranslation.getX(), targetTranslation.getY());
                        return targetRotation;
                }),
                intake.intakeSequence(),
                shootCmd(hopper)
            ),

            () -> {
                var robotPose= drive.getPose().getTranslation();

                if(robotPose.getY()<3){
                    return true;
                } else if (robotPose.getY()>5){
                    return false;
                } 
                return false;
            }
        );
    }
}
