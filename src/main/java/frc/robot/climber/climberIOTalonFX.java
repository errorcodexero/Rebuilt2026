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


public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX armmotor;
    private final TalonFX elevatormotor;

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

    public ClimberIOTalonFX() {
        //arm motor configuration 
         armmotor = new TalonFX(ClimberConstants.MotoroneID); 
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.Slot0;
        armConfig.slot0;
        armConfig.slot0;
        armConfig.slot0;
        armConfig.slot0;
        tryUntilOk(5, () -> motorOne.getConfigurator().apply(motorOneConfiguration, 0.25)
         ArmConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.armCurrentLimit.in(Amps);


        // elevator motor configuration
        elevatormotor = new TalonFX(ClimberConstants.MotortwoID);
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.Slot0;
        elevatorConfig.Slot0;
        elevatorConfig.slot0;
        elevatorConfig.slot0;
        tryUntilOk(5, () -> motorTwo.getConfigurator().apply(motorTwoConfiguration, 0.25)
        elevatorConfig.CurrentLimits.StatorCurrentLimit= ClimberConstants.elevatorCurrentLimit.in(Amps);

        //status signal initilzation 
        
         

       
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

    @Override
    public void updateInputs(ClimberInputs inputs) {
        var motorOneStatus = BaseStatusSignal.refreshAll(motorOneSignals);
        var motorTwoStatus = BaseStatusSignal.refreshAll(motorTwoSignals);

        inputs.oneConnected = oneIsOkDebounce.calculate(motorOneStatus.isOK());
        inputs.twoConnected = twoIsOkDebounce.calculate(motorTwoStatus.isOK());

        inputs.onePosition = motorOnePosition.getValue();
        inputs.oneVolts = motorOneVoltage.getValue();
        inputs.oneCurrent = motorOneCurrent.getValue();

        inputs.twoPosition = motorTwoPosition.getValue();
        inputs.twoVolts = motorTwoVoltage.getValue();
        inputs.twoCurrent = motorTwoCurrent.getValue();
    }

    @Override
    public void applyOutputs(ClimberOutputs outputs) {
        motorOne.setControl(new PositionVoltage(outputs.oneSetpoint));
        motorTwo.setControl(new PositionVoltage(outputs.twoSetpoint));
    }
}

// climber goes above swerve 
// limelight also goes on climber 
// climber attaches to the shooter 
// cammera has to face the front to see the limelights
// swerve io follows a closed loop system 
// inside heckshaft drives the intake (arm)