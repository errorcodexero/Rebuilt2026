package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.MapleSimUtil;
import frc.robot.util.Mechanism3d;

public class Shooter extends SubsystemBase {
    
    private final ShooterIO io_;
    private final ShooterIOInputsAutoLogged inputs_ = new ShooterIOInputsAutoLogged();

    private AngularVelocity shooterTarget = RadiansPerSecond.zero();
    private Angle hoodTarget = Radians.zero();

    public Shooter(ShooterIO io) {
        io_ = io;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Shooter", inputs_);

        Mechanism3d.measured.setHood(hoodTarget);
        Mechanism3d.setpoints.setHood(hoodTarget);

        if (Constants.getMode() == Mode.SIM) {
            MapleSimUtil.setShooterVelocity(inputs_.wheelVelocity);
            MapleSimUtil.setHoodAngle(hoodTarget);
        }
        
        Logger.recordOutput("Shooter/VelocitySetPoint", shooterTarget);
        Logger.recordOutput("Shooter/HoodSetPoint", hoodTarget);
    }

    // Shooter Methods

    private void setShooterVelocity(AngularVelocity vel) {
        shooterTarget = vel;
        io_.setShooterVelocity(vel);
    }

    private void setShooterVoltage(Voltage vol) {
        shooterTarget = RotationsPerSecond.zero();
        io_.setShooterVoltage(vol);
    }

    private void stopShooter() {
        shooterTarget = RotationsPerSecond.zero();
        io_.stopShooter();
    }

    public AngularVelocity getShooterVelocity() {
        return inputs_.wheelVelocity;
    }

    public Voltage getShooterVoltage() {
        return inputs_.shooter1Voltage;
    }
    
    public boolean isShooterReady() {
        return inputs_.wheelVelocity.isNear(shooterTarget, ShooterConstants.shooterTolerance); 
    }

    public Current getShooterCurrent() {
        return (inputs_.shooter1Current)
            .plus(inputs_.shooter2Current)
            .plus(inputs_.shooter3Current)
            .plus(inputs_.shooter4Current);
    }

    public Command runToVelocityCmd(AngularVelocity vel) {
        return runOnce(() -> setShooterVelocity(vel))
            .andThen(Commands.waitUntil(this::isShooterReady)).withName("Set Shooter Velocity");
    }

    public Command stopCmd() {
        return runOnce(() -> stopShooter())
            .andThen(Commands.waitUntil(this::isShooterReady)).withName("Stop Shooter");
    }

    public Command runVoltageCmd(Voltage vol) {
        return runOnce(() -> setShooterVoltage(vol)).withName("Set Shooter Voltage");
    }

    // Hood Methods
    private void setHoodAngle(Angle pos) {
        hoodTarget = pos;
        io_.setHoodPosition(pos);
    }

    public Command hoodToPosCmd(Angle pos) {
        return runOnce(() -> setHoodAngle(pos)).withName("Set Hood Position");
    }

    // Both
    private void setSetpoints(AngularVelocity vel, Angle pos) {
        shooterTarget = vel;
        setShooterVelocity(shooterTarget);
        hoodTarget = pos;
        setHoodAngle(hoodTarget);
    }

    /**
     * Shoot balls into the hub until the command ends.
     * @return
     */
    public Command shootCmd() {
        return startEnd(() -> {
            io_.setShooterVelocity(RPM.of(1000));
            io_.setHoodPosition(Degrees.of(45));
        }, () -> {
            io_.stopShooter();
        });
    }

    public Command runToSetpointsCmd(AngularVelocity vel, Angle pos) {
        return runOnce(() -> setSetpoints(vel, pos)).andThen(Commands.waitUntil(this::isShooterReady));
    }

    public Command runDynamicSetpoints(Supplier<AngularVelocity> vel, Supplier<Angle> pos ) {
        return run(() -> setSetpoints(vel.get(), pos.get()));
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
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, (state) -> Logger.recordOutput("HoodSysIdTestState", state.toString()));

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                (volts) -> io_.setShooterVoltage(volts),
                null,
                this);

        return new SysIdRoutine(cfg, mfg);
    }
}
