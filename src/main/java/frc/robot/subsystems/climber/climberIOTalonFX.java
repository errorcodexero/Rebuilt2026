  package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
//import frc.robot.climber.climberIO.ClimberInputs;


public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX Climbmotor ;
    private final TalonFX deployClimb ;

    /// arm motor control requests 
       private final VoltageOut armVoltageRequest = new VoltageOut(0);
       private final VelocityVoltage armVelocityRequest = new VelocityVoltage(0.0);
       
       ///elevator motor control requests 
        private final VoltageOut elevatorVoltageRequest = new VoltageOut(0);
        private final VelocityVoltage elevatorVelocityRequest = new VelocityVoltage(0.0);
        
        // arm motor status signals 
    private final StatusSignal<Angle> motorOnePosition;
    private final StatusSignal<Voltage> motorOneVoltage;
    private final StatusSignal<Current> motorOneCurrent;

      // elevator motor status siginals
    private final StatusSignal<Angle> motorTwoPosition;
    private final StatusSignal<Voltage> motorTwoVoltage;
    private final StatusSignal<Current> motorTwoCurrent;

    private final List<BaseStatusSignal> motorOneSignals;
    private final List<BaseStatusSignal> motorTwoSignals;
    private TalonFX deployMotor;
    private TalonFX deployClimber; 
    
        public climberIOTalonFX(int motorID, int climbID){
        deployMotor= new TalonFX(motorID);
        deployClimb  = new TalonFX(climbID);

        //status signal initilzation 
       motorOnePosition = deployMotor.getPosition();
       motorOneVoltage = deployMotor.getMotorVoltage();
       motorOneCurrent = deployMotor.getSupplyCurrent();
        
      motorOneSignals = List.of(motorOnePosition, motorOneVoltage, motorOneCurrent );
      ;
         motorTwoPosition = deployClimb .getPosition();
         motorTwoVoltage = deployClimb .getMotorVoltage();
         motorTwoCurrent = deployClimb .getSupplyCurrent();

         motorTwoSignals = List.of(motorTwoPosition, motorTwoVoltage, motorTwoCurrent);
    }

    public void ClimberIOTalonFX() {
        //arm motor configuration 
         deployMotor= new TalonFX(ClimberConstants.deployMotorID); 
        TalonFXConfiguration motorConfig  = new TalonFXConfiguration();
        motorConfig .Slot0.kP=0.0;
        motorConfig .Slot0.kI=0.0;
        motorConfig .Slot0.kD=0.0;
        motorConfig .Slot0.kS=0.0;
        motorConfig .Slot0.kV=0.0;
        tryUntilOk(5, () -> motorOne.getConfigurator().apply(motorOneConfiguration, 0.25));
         motorConfig s.CurrentLimits.StatorCurrentLimit = ClimberConstants.armCurrentLimit.in(Amps);
        tryUntilOk(5, () -> armmotor.getConfigurator().apply(motorConfig , 0.25));


        // elevator motor configuration
        deployClimb  = new TalonFX(ClimberConstants.climbMotorID);
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        climbConfig.Slot0.kP=0.0;
        climbConfig.Slot0.kL=0.0; 
        climbConfig.slot0.kS=0.0;
        climbConfig.slot0.kV=0.0;
        tryUntilOk(5, () -> motorTwo.getConfigurator().apply(motorTwoConfiguration, 0.25);
        climbConfig.CurrentLimits.StatorCurrentLimit= ClimberConstants.elevatorCurrentLimit.in(Amps);
        tryUntilOk(5, () -> deployClimb .getConfigurator().apply(motorConfig , 0.25));

           
        TalonFXConfiguration motorOneConfiguration = new TalonFXConfiguration();
        motorOneConfiguration.Feedback.FeedbackRemoteSensorID = 3;
        motorOneConfiguration.Feedback.SensorToMechanismRatio = 1;

        
        TalonFXConfiguration motorTwoConfiguration = new TalonFXConfiguration();
        motorTwoConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> motorTwo.getConfigurator().apply(motorTwoConfiguration, 0.25));

        motorOnePosition = motorOne.getPosition();
        motorOneVoltage = motorOne.getMotorVoltage();
        motorOneCurrent = motorOne.getStatorCurrent();

        motorTwoPosition = motorTwo.getPosition();
        motorTwoVoltage = motorTwo.getMotorVoltage();
        motorTwoCurrent = motorTwo.getStatorCurrent();

        motorOneSignals = List.of(
            motorOnePosition,
            motorOneVoltage,
            motorOneCurrent
        );

        motorTwoSignals = List.of(
            motorTwoPosition,
            motorTwoVoltage,
            motorTwoCurrent
        );

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50, motorOneSignals));
        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50, motorTwoSignals));
    }
}
     

