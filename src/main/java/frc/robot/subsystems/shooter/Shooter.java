package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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

public class Shooter extends SubsystemBase {
    
    private final ShooterIO io_;
    private final ShooterIOInputsAutoLogged inputs_;
    private AngularVelocity shooterTarget;
    private Angle hoodTarget;

    public Shooter(ShooterIO io) {
        io_ = io;
        inputs_ = new ShooterIOInputsAutoLogged();
        shooterTarget = RotationsPerSecond.zero();
        hoodTarget = Radians.zero();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Shooter", inputs_);
        Logger.recordOutput("shooter/shooterTarget", shooterTarget.in(RotationsPerSecond));
        Logger.recordOutput("shooter/hoodTarget", hoodTarget);
    }

    // Shooter Methods

    public void setShooterVelocity(AngularVelocity vel) {
        shooterTarget = vel;
        io_.setShooterVelocity(vel);
    }

    public void setShooterVoltage(Voltage vol) {
        shooterTarget = RotationsPerSecond.of(0.0);
        io_.setShooterVoltage(vol);
    }

    public void stopShooter() {
        shooterTarget = RotationsPerSecond.of(0.0);
        io_.stopShooter();
    }

    public AngularVelocity getShooterVelocity() {
        return inputs_.shooter1Velocity;
    }
    
    public boolean isShooterReady() {
        return Math.abs(inputs_.shooter1Velocity.in(RotationsPerSecond) - shooterTarget.in(RotationsPerSecond)) < ShooterConstants.shooterTolerance.in(RevolutionsPerSecond); 
    }

    public Voltage getShooterVoltage() {
        return inputs_.shooter1Voltage;
    }

    public Current getShooterCurrent() {
        return (inputs_.shooter1Current)
                .plus(inputs_.shooter2Current)
                .plus(inputs_.shooter3Current)
                .plus(inputs_.shooter4Current);
    }

    public Command setShooterVelocityCommand(Shooter shooter, AngularVelocity vel) {
        return Commands.runOnce(() -> shooter.setShooterVelocity(vel), shooter)
        .andThen(Commands.waitUntil(() -> shooter.isShooterReady())).withName("Set Shooter Velocity");
    }

    public Command stopShooterCommand(Shooter shooter) {
        return Commands.runOnce(() -> shooter.stopShooter(), shooter)
        .andThen(Commands.waitUntil(() -> shooter.isShooterReady())).withName("Stop Shooter");
    }

    public Command setShooterVoltageCommand(Shooter shooter, Voltage vol) {
        return Commands.runOnce(() -> shooter.setShooterVoltage(vol), shooter).withName("Set Shooter Voltage");
    }

    // Hood Methods
    public void setHoodAngle(Angle pos) {
        hoodTarget = pos;
        io_.setHoodPosition(pos);
    }

    public void setHoodVoltage(Voltage vol) {
        hoodTarget = Radians.of(0);
        io_.setHoodVoltage(vol);
    }

    public Angle getHoodAngle() {
        return inputs_.hoodPosition;
    }

    public Voltage getHoodVoltage() {
        return inputs_.hoodVoltage;
    }

    public Command hoodToPosCommand(Shooter shooter, Angle pos) {
        return Commands.runOnce(() -> shooter.setHoodAngle(pos), shooter).withName("Set Hood Position");
    }

    public Command setHoodVoltageCommand(Shooter shooter, Voltage vol) {
        return Commands.runOnce(() -> shooter.setHoodVoltage(vol), shooter).withName("Set Hood Voltage");
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


    public Command hoodSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return hoodIdRoutine().quasistatic(dir);
    }

    public Command hoodSysIdDynamic(SysIdRoutine.Direction dir) {
        return hoodIdRoutine().dynamic(dir);
    }

    private SysIdRoutine hoodIdRoutine() {
        Voltage step = Volts.of(7);
        Time to = Seconds.of(10.0);
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, (state) -> Logger.recordOutput("ShooterSysIdTestState", state.toString()));

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                (volts) -> io_.setHoodVoltage(volts),
                null,
                this);

        return new SysIdRoutine(cfg, mfg);
    }
}
