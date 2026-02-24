package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean allConnected = false;

        public Voltage shooter1Voltage = Volts.zero();
        public Current shooter1Current = Amps.zero();
        public AngularVelocity shooter1Velocity = RadiansPerSecond.zero();

        public Voltage shooter2Voltage = Volts.zero();
        public Current shooter2Current = Amps.zero();
        public AngularVelocity shooter2Velocity = RadiansPerSecond.zero();

        public Voltage shooter3Voltage = Volts.zero();
        public Current shooter3Current = Amps.zero();
        public AngularVelocity shooter3Velocity = RadiansPerSecond.zero();

<<<<<<< michael-shooting-while-moving
=======
        public Voltage shooter4Voltage = Volts.zero();
        public Current shooter4Current = Amps.zero();
        public AngularVelocity shooter4Velocity = RadiansPerSecond.zero();

>>>>>>> main
        public AngularVelocity wheelVelocity = RadiansPerSecond.zero();
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVelocity(AngularVelocity vel) {}

    public default void setVoltage(Voltage vol) {}

    public default void stop() {}
}
