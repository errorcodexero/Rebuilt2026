package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final ShooterIO io_;
    private final ShooterIOInputsAutoLogged inputs_;
    private AngularVelocity shooterTarget;

    public Shooter(ShooterIO io) {
        io_ = io;
        inputs_ = new ShooterIOInputsAutoLogged();
        shooterTarget = RotationsPerSecond.zero();
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("Grabber", inputs_);

        Logger.recordOutput("shooter/target", shooterTarget.in(RotationsPerSecond));
        //Logger.recordOutput("shooter/atTarget", )

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
        shooterTarget = RotationsPerSecond.of(0);
        io_.stopShooter();
    }

    public AngularVelocity getShooterVelocity() {
        return inputs_.shooter1Velocity;
    }
    
    public boolean isShooterReady() {
        return inputs_.shooter1Velocity == shooterTarget; 
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

    // Hood Methods

    // Sys ID

}
