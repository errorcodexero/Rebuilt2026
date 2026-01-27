package frc.robot.subsystems.hopper;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public interface HopperIO {
    public static class HopperIOInputs {

        public AngularVelocity feederVelocity = DegreesPerSecond.of(0);
        public Voltage feederVoltage = Volts.of(0.0);
        public Current feederCurrent = Amps.of(0.0);

        public AngularVelocity scramblerVelocity = DegreesPerSecond.of(0);
        public Voltage scramblerVoltage = Volts.of(0.0);
        public Current scramblerCurrent = Amps.of(0.0);
    }
    public default void updateInputs(HopperIOInputs inputs) {}
    
    public default void setFeederVoltage(Voltage voltage) {}

    public default void setFeederVelocity(AngularVelocity velocity) {}

    public default void setScramblerVoltage(Voltage voltage) {}
    
    public default void setScramblerVelocity(AngularVelocity velocity) {}
}
