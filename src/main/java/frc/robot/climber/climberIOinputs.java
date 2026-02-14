package frc.robot.climber;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;


    @AutoLog
    public interface climberIOinputs{
      public double appliedvolts= 0.0;
      public double supplyCurrentAmps = 0.0;
      public double TorqueCurrentAmps = 0.0;
      
      public Angle positon = Units.Rotations.of(0);
      public AngularVelocity velocity = Units.RotationsPerSecond.of(0);
      public Angle oneSetPoint = null;

    
         //methods that exist in the climberio class 
       public default void updateInputs(climberIOinputs inputs) {}

       public default void setVoltage(double volts) {}
    }
  




