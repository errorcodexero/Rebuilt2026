package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {

    public default void updateInputs(ClimberInputs inputs) {}

    @AutoLog
    public static class ClimberInputs {
        public Angle position = Degrees.zero();
        public Voltage voltage = Volts.zero();
    }

}
