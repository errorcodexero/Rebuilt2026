package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public class ClimberIOInputs {
      public Voltage deployVolts = Volts.of(0);
      public Current deployCurrent = Amps.of(0);
      public Angle deployPosition = Degrees.of(0);
      public AngularVelocity deployVelocity = DegreesPerSecond.of(0);

      public Voltage twistVolts = Volts.of(0);
      public Current twistCurrent = Amps.of(0);
      public Angle twistPosition = Degrees.of(0);
      public AngularVelocity twistVelocity = DegreesPerSecond.of(0);
    }
      
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setDeployAngle(Angle deploymentAngle) {}

    public default void setDeployVelocity(AngularVelocity deployVelocity) {}

    public default void setDeployVoltage(Voltage deployVoltage) {}

    public default void setTwistAngle(Angle climbAngle) {}

    public default void setTwistVelocity(AngularVelocity climbVelocity) {}

    public default void setTwistVoltage(Voltage climbVoltage) {}

    public default void stopDeploy() {}

    public default void stopTwist() {}
}
