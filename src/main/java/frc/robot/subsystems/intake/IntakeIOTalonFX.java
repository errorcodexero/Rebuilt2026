package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;     
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.CANBus; 


public class IntakeIOTalonFX implements IntakeIO {

    //Creating motor objects
    public final TalonFX rollerMotor;
    public final TalonFX pivotMotor;

    //Pivot motor control requests
    private final MotionMagicVoltage pivotAngleRequest= new MotionMagicVoltage(Degrees.of(0));
    private final VoltageOut pivotVoltageRequest= new VoltageOut(Volts.of(0));

    //Roller motor control requests
    private final VoltageOut rollerVoltageRequest = new VoltageOut(Volts.of(0));
    private final VelocityVoltage rollerVelocityRequest= new VelocityVoltage(DegreesPerSecond.of(0));

    //Pivot status signals
    private StatusSignal<Angle> pivotAngleSignal;
    private StatusSignal<AngularVelocity> pivotAngularVelocitySignal;
    private StatusSignal<Current> pivotCurrentAmpsSignal;
    private StatusSignal<Voltage> pivotAppliedVoltsSignal;

    //Roller status signals
    private StatusSignal<AngularVelocity> rollerAngularVelocitySignal;
    private StatusSignal<Voltage> rollerAppliedVoltsSignal;
    private StatusSignal<Current> rollerCurrentAmpsSignal; 

    public IntakeIOTalonFX(CANBus pivotBus, CANBus rollerBus) {
        // Initialize motor objects
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorCANID, rollerBus);
        pivotMotor = new TalonFX(IntakeConstants.pivotMotorCANID, pivotBus);

        // Configuration for the pivot motor
        final TalonFXConfiguration pivotConfigs= new TalonFXConfiguration();

        //Setting pivot motor to Brake mode
        pivotConfigs.MotorOutput.NeutralMode= NeutralModeValue.Brake;

        //Current Limit Configurations
        pivotConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.currentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        //PID Configurations
        pivotConfigs.Slot0.kP= IntakeConstants.pivotKP;
        pivotConfigs.Slot0.kD= IntakeConstants.pivotKD;
        pivotConfigs.Slot0.kV= IntakeConstants.pivotKV;
        pivotConfigs.Slot0.kI= IntakeConstants.pivotKI;
        pivotConfigs.Slot0.kA= IntakeConstants.pivotKA;
        pivotConfigs.Slot0.kG= IntakeConstants.pivotKG;
        pivotConfigs.Slot0.kS= IntakeConstants.pivotKS;

        //Soft Limit Configurations
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable= true;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold= IntakeConstants.pivotMaxAngle.in(Degrees);
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable= true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold= IntakeConstants.pivotMinAngle.in(Degrees);

        //Motion Magic Configurations
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity= IntakeConstants.pivotCruiseVelocity.in(DegreesPerSecond); 
        pivotConfigs.MotionMagic.MotionMagicAcceleration= IntakeConstants.pivotCruiseAcceleration.in(DegreesPerSecondPerSecond); 
        pivotConfigs.MotionMagic.MotionMagicJerk= IntakeConstants.pivotMaxJerk; 

        //Used to apply configs once instead of having multiple iterations to do this
        //Also trys the configuartion 5 times until it receives an OK status signal 
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfigs, 0.25));


        // Configuration for the roller motor
        final TalonFXConfiguration rollerConfigs= new TalonFXConfiguration();
        
        //Setting roller motor to Coast mode
        rollerConfigs.MotorOutput.NeutralMode= NeutralModeValue.Coast;

        //Current Limit Configurations
        rollerConfigs.CurrentLimits.StatorCurrentLimit= IntakeConstants.currentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable= true;

        //PID Configurations
        rollerConfigs.Slot0.kP= IntakeConstants.rollerKP;
        rollerConfigs.Slot0.kD= IntakeConstants.rollerKD;
        rollerConfigs.Slot0.kV= IntakeConstants.rollerKV;
        rollerConfigs.Slot0.kI= IntakeConstants.rollerKI;
        rollerConfigs.Slot0.kA= IntakeConstants.rollerKA;
        rollerConfigs.Slot0.kS= IntakeConstants.rollerKS;
        rollerConfigs.Slot0.kG= IntakeConstants.rollerKG;

        //Used to apply configs once instead of having multiple iterations to do this
        //Also trys the configuartion 5 times until it receives an OK status signal 
        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfigs, 0.25));

        // Initialize all status signals
        pivotAngleSignal = pivotMotor.getPosition();
        pivotAngularVelocitySignal = pivotMotor.getVelocity();
        rollerAngularVelocitySignal = rollerMotor.getVelocity();
        rollerAppliedVoltsSignal = rollerMotor.getSupplyVoltage();
        pivotAppliedVoltsSignal = pivotMotor.getSupplyVoltage();
        rollerCurrentAmpsSignal = rollerMotor.getStatorCurrent();
        pivotCurrentAmpsSignal = pivotMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(20, rollerAppliedVoltsSignal, rollerCurrentAmpsSignal,pivotAppliedVoltsSignal, pivotCurrentAmpsSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivotAngleSignal,pivotAngularVelocitySignal,rollerAngularVelocitySignal);

        // Optimize CAN bus for these parent devices-motors
        ParentDevice.optimizeBusUtilizationForAll(rollerMotor, pivotMotor);
        
    }
    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(
            pivotAngleSignal,
            pivotAngularVelocitySignal,
            rollerAngularVelocitySignal,
            rollerAppliedVoltsSignal,
            pivotAppliedVoltsSignal,
            rollerCurrentAmpsSignal,
            pivotCurrentAmpsSignal
        );

        inputs.PivotAngle = pivotAngleSignal.getValue();
        inputs.PivotAngularVelocity = pivotAngularVelocitySignal.getValue();
        inputs.RollerAngularVelocity = rollerAngularVelocitySignal.getValue();
        inputs.RollerAppliedVolts = rollerAppliedVoltsSignal.getValue();
        inputs.PivotAppliedVolts = pivotAppliedVoltsSignal.getValue();
        inputs.RollerCurrentAmps = rollerCurrentAmpsSignal.getValue();
        inputs.PivotCurrentAmps = pivotCurrentAmpsSignal.getValue();
    }

    @Override
    public void setRollerVoltage(Voltage volts) {
        // Convert Voltage -> numeric volts and send via Phoenix voltage request
        rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }

    @Override
    public void setPivotAngle(Angle angle) {
        // Create Motion Magic control request with desired angle
        pivotMotor.setControl(pivotAngleRequest.withPosition(angle));
    }

    @Override
    public void setRollerVelocity(AngularVelocity velocity) {
        // Create Velocity control request with desired velocity
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(velocity));
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        // Create Voltage control request with desired voltage
        pivotMotor.setControl(pivotVoltageRequest.withOutput(voltage));
    }

    @Override
    public void stopRoller() {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(0));
    }  
}