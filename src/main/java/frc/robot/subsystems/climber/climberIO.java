package frc.robot.subsystems.climber;

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

public interface ClimberIO {

    @AutoLog
    public class ClimberIOInputs {
      public Voltage deployVolts = Volts.zero();
      public Current deployCurrent = Amps.zero();
      public Angle deployPosition = Radians.zero();

      public Voltage climbVolts = Volts.zero();
      public Current climbCurrent = Amps.zero();
      public Angle climbPosition = Radians.zero();
    }
      
    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void setDeploy(Angle deploymentAngle) {}

    public default void setClimb(Angle climbAngle) {}

    public default void stopDeploy() {}

    public default void stopClimb() {}
}
