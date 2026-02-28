package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
      // public boolean oneConnected = false;
      // public Voltage oneVolts = Volts.zero();
      // public Current oneCurrent = Amps.zero();
      //  public Angle onePosition = Radians.zero();

        public Angle deployPostion = Radians.zero();
        public Voltage deployVolts = Volts.zero();
        public Current deployCurrent = Amps.zero(); 
        public AngularVelocity deployvelocity = RadiansPerSecond.zero(); 

        public Angle twistPostion = Radians.zero();
        public Voltage twistVolts = Volts.zero();
        public Current twistCurrent = Amps.zero();
        public AngularVelocity twistVelocity = RadiansPerSecond.zero();
      

       public  boolean twoConnected = false;
      public Voltage twoVolts = Volts.zero();
       public Current twoCurrent = Amps.zero();
       public Angle twoPosition = Radians.zero();
    }

    public static class ClimberIOOutputs {
      public Angle oneSetpoint = Degrees.zero();
      public Angle twoSetpoint = Degrees.zero(); 
    }
     // methods for angles 
      public Angle  deployPosition = Radians.zero();
      public Angle  twistpostion = Radians.zero(); 

      // interface for primary methods 
     public default void setDeployAngle(Angle angle) {}
     public default void  setTwistAngle (Angle angle) {}


    //interface for secondary methods 
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void applyOutputs(ClimberIOOutputs outputs) {}
     
    public default void setDeployVoltage (Voltage voltage ) {}

    public default void setTwistVoltage (Voltage voltage)  {}

    public default void setDeployVelocity (AngularVelocity velocity) {}

    public default void  setTwistVelocity (AngularVelocity velocity) {}

    public default void stopDeploy () {}

    public default void stopTwist() {}

    
    


}
