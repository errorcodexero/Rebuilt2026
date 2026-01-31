package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

        public boolean shooter1Ready = false;
        public Voltage shooter1Voltage = Volts.zero();
        public Current shooter1Current = Amps.zero();
        public AngularVelocity shooter1Velocity = RadiansPerSecond.zero();

        public boolean shooter2Ready = false;
        public Voltage shooter2Voltage = Volts.zero();
        public Current shooter2Current = Amps.zero();
        public AngularVelocity shooter2Velocity = RadiansPerSecond.zero();

        public boolean shooter3Ready = false;
        public Voltage shooter3Voltage = Volts.zero();
        public Current shooter3Current = Amps.zero();
        public AngularVelocity shooter3Velocity = RadiansPerSecond.zero();

        public boolean shooter4Ready = false;
        public Voltage shooter4Voltage = Volts.zero();
        public Current shooter4Current = Amps.zero();
        public AngularVelocity shooter4Velocity = RadiansPerSecond.zero();
        
        public Angle hoodPosition = Radians.zero();
        public Voltage hoodVoltage = Volts.zero();
        public Current hoodCurrent = Amps.zero();
        public AngularVelocity hoodVelocity = RadiansPerSecond.zero();
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setHoodVoltage(Voltage vol) {}

    public default void setShooterVelocity(AngularVelocity vel) {}

    public default void setShooterVoltage(Voltage vol) {}

    public default void stopShooter() {}

    public default void setHoodPosition(Angle pos) {}
}
