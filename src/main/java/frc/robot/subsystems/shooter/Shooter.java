package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.util.MapleSimUtil;
import frc.robot.util.Mechanism3d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.tunings.ArcShooterTuning;
import frc.robot.subsystems.shooter.tunings.FlatShooterTuning;
import frc.robot.subsystems.shooter.tunings.ShooterTuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;


public class Shooter extends SubsystemBase {
    
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final HoodIO hoodIO;
    private final HoodInputsAutoLogged hoodInputs = new HoodInputsAutoLogged();

    private final Debouncer statusDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private final Alert disconnectionAlert =
        new Alert("One or more shooter motors are disconnected!", AlertType.kError);

    private AngularVelocity shooterTarget = RadiansPerSecond.zero();
    private Angle hoodTarget = Radians.zero();

    private int tuningIndex_ = 0 ;
    private List<ShooterTuning> tunings_ = new ArrayList<ShooterTuning>() ;

    public Shooter(ShooterIO ioShooter, HoodIO ioHood) {
        this.shooterIO = ioShooter;
        this.hoodIO = ioHood;

        tunings_.add(new ArcShooterTuning()) ;
        tunings_.add(new FlatShooterTuning()) ;
        tuningIndex_ = 0 ;
    }

    @Override
    public void periodic() {
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
        
        Logger.recordOutput("Shooter/VelocitySetPoint", shooterTarget);
        Logger.recordOutput("Shooter/HoodSetPoint", hoodTarget);
    }

    // Shooter Methods

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
        });
    }

    /**
     * The command that the shooter can run whenever its not shooting to manage
     * things like going to different hood angles to get ready to shoot,
     * or lowering the hood under the trench.
     * @return A command that does so.
     */
    public Command awaitShooting(Supplier<Pose2d> robotPose) {
        return runDynamicSetpoints(
            () -> {
                Distance zone =
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterConstants.Positions.blueAllianceWall
                    : ShooterConstants.Positions.redAllianceWall;

                Pose2d pose = robotPose.get();

                Translation2d hubTranslation =
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterConstants.Positions.blueHubPose
                    : ShooterConstants.Positions.redHubPose;
            
                Distance distanceToHub = Meters.of(pose.getTranslation().getDistance(hubTranslation));
                var vel = 0.0;

                if ((Math.abs(pose.getX() - zone.magnitude()) < ShooterConstants.Positions.spinUpZone.magnitude())) {
                    var params = getTuning().getShooterParams(distanceToHub.in(Meters));
                    vel = params.velocity ;
                }
                return RotationsPerSecond.of(vel);
            },
                                
            () -> {
                Pose2d pose = robotPose.get();
                Pose2d nearestTrench = pose.nearest(FieldConstants.trenches);
                Distance nearestDistance = Meters.of(pose.getTranslation().getDistance(nearestTrench.getTranslation()));

                if (nearestDistance.lte(ShooterConstants.allowedTrenchDistance)) {
                    return Degrees.zero();
                }

                Translation2d hubTranslation =
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterConstants.Positions.blueHubPose
                    : ShooterConstants.Positions.redHubPose;

                Distance distanceToHub = Meters.of(pose.getTranslation().getDistance(hubTranslation));
                var params = getTuning().getShooterParams(distanceToHub.in(Meters));
                return Degrees.of(params.hood) ;
        });
    }

    public Command runToSetpointsCmd(AngularVelocity vel, Angle pos) {
        return runOnce(() -> setSetpoints(vel, pos)).andThen(Commands.waitUntil(this::isShooterReady));
    }

    public Command runToVelocityCmd(AngularVelocity vel) {
        return runOnce(() -> setShooterVelocity(vel))
            .andThen(Commands.waitUntil(this::isShooterReady)).withName("Set Shooter Velocity");
    }

    public Command requestToVelocityCmd(AngularVelocity vel) {
        return runOnce(() -> setShooterVelocity(vel)) ;
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
    public Command shoot(Supplier<Pose2d> pose, Hopper hopper, IntakeSubsystem intake) {
        
        return Commands.parallel(
                defer(() -> {

                    // constructing
                    Translation2d hubTranslation =
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? ShooterConstants.Positions.blueHubPose
                        : ShooterConstants.Positions.redHubPose;

                    return runDynamicSetpoints(
                        () -> {
                            Distance distanceToHub = Meters.of(pose.get().getTranslation().getDistance(hubTranslation));
                            var params = getTuning().getShooterParams(distanceToHub.in(Meters));
                            var vel = params.velocity;
                            return RotationsPerSecond.of(vel);
                        },
                    
                        () -> {
                            // periodic
                            Distance distanceToHub = Meters.of(pose.get().getTranslation().getDistance(hubTranslation));
                            var params = getTuning().getShooterParams(distanceToHub.in(Meters));
                            return Degrees.of(params.hood) ;
                    });
            }),
            hopper.forwardFeed(),
            intake.enableShootMode()
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
        
        return Commands.defer(() -> Commands.parallel(
            runDynamicSetpoints(() -> RotationsPerSecond.of(shooterVelocity.get()), () -> Degrees.of(hoodAngle.get())),
            hopper.feed(
                RotationsPerSecond.of(feederVelocity.get()),
                RotationsPerSecond.of(scramblerVelocity.get())
            )
        ), Set.of(this, hopper));
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
}
