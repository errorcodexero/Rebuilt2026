package frc.robot.subsystems.hopper;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public boolean feederConnected = false;
        public AngularVelocity feederVelocity = DegreesPerSecond.of(0);
        public AngularAcceleration feederAcceleration = DegreesPerSecondPerSecond.of(0) ;
        public Voltage feederVoltage = Volts.of(0.0);
        public Current feederCurrent = Amps.of(0.0);
        public Temperature feederTemp = Celsius.zero();

        public boolean scramblerConnected = false;
        public AngularVelocity scramblerVelocity = DegreesPerSecond.of(0);
        public AngularAcceleration scramblerAcceleration = DegreesPerSecondPerSecond.of(0) ;
        public Voltage scramblerVoltage = Volts.of(0.0);
        public Current scramblerCurrent = Amps.of(0.0);
        public Temperature scramblerTemp = Celsius.zero();
    }
    public default void updateInputs(HopperIOInputs inputs) {}
    
    public default void setFeederVoltage(Voltage voltage) {}

    public default void setFeederVelocity(AngularVelocity velocity) {}

    public default void setScramblerVoltage(Voltage voltage) {}
    
    public default void setScramblerVelocity(AngularVelocity velocity) {}
}