package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {

    @AutoLog
    public static class ClimberInputs {
        boolean oneConnected = false;
        Voltage oneVolts = Volts.zero();
        Current oneCurrent = Amps.zero();
        Angle onePosition = Radians.zero();

        boolean twoConnected = false;
        Voltage twoVolts = Volts.zero();
        Current twoCurrent = Amps.zero();
        Angle twoPosition = Radians.zero();
    }

    public static class ClimberOutputs {
        Angle oneSetpoint = Degrees.zero();
        Angle twoSetpoint = Degrees.zero(); 
    }

    public default void updateInputs(ClimberInputs inputs) {}

    public default void applyOutputs(ClimberOutputs outputs) {}

}
