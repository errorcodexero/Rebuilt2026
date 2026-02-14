package frc.robot.climber;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface climberIO {

    @AutoLog
    public static class ClimberInputs {
       public boolean oneConnected = false;
       public Voltage oneVolts = Volts.zero();
       public Current oneCurrent = Amps.zero();
       public Angle onePosition = Radians.zero();

       public  boolean twoConnected = false;
      public Voltage twoVolts = Volts.zero();
       public Current twoCurrent = Amps.zero();
       public Angle twoPosition = Radians.zero();
    }

    public static class ClimberOutputs implements climberIOinputs {
      public Angle oneSetpoint = Degrees.zero();
      public Angle twoSetpoint = Degrees.zero(); 
    }

    public default void updateInputs(ClimberInputs inputs) {}

    public default void applyOutputs(ClimberOutputs outputs) {}

}
