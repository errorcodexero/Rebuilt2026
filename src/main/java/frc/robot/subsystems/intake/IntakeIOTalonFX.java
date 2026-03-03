package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage; 


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

    private final Debouncer pivotConnectedDebounce =
        new Debouncer(0.25, DebounceType.kFalling);

    private final Debouncer rollerConnectedDebounce =
        new Debouncer(0.25, DebounceType.kFalling);

    //Pivot status signals
    private StatusSignal<Angle> pivotAngleSignal;
    private StatusSignal<AngularVelocity> pivotAngularVelocitySignal;
    private StatusSignal<Current> pivotCurrentAmpsSignal;
    private StatusSignal<Voltage> pivotAppliedVoltsSignal;

    //Roller status signals
    private StatusSignal<AngularVelocity> rollerAngularVelocitySignal;
    private StatusSignal<Voltage> rollerAppliedVoltsSignal;
    private StatusSignal<Current> rollerCurrentAmpsSignal; 

    //Separate Motion Magic Profiles for the pivot motor depending on intake pivot position
    private MotionMagicConfigs deployedWaitingMotionMagic;
    private MotionMagicConfigs stowedDeployedMotionMagic;
    private MotionMagicConfigs deployedStowedMotionMagic;

    public IntakeIOTalonFX(CANBus canbus) {
        // Initialize motor objects
        rollerMotor = new TalonFX(IntakeConstants.CANID.rollerMotorCANID, canbus);
        pivotMotor = new TalonFX(IntakeConstants.CANID.pivotMotorCANID, canbus);

        // Configuration for the pivot motor
        final TalonFXConfiguration pivotConfigs= new TalonFXConfiguration();

        //Setting pivot motor to Brake mode
        pivotConfigs.MotorOutput.NeutralMode= NeutralModeValue.Brake;

        //Current Limit Configurations
        pivotConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.CurrentLimits.currentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        //PID Configurations
        pivotConfigs.Slot0.kP= IntakeConstants.PID.pivotKP;
        pivotConfigs.Slot0.kD= IntakeConstants.PID.pivotKD;
        pivotConfigs.Slot0.kV= IntakeConstants.PID.pivotKV;
        pivotConfigs.Slot0.kI= IntakeConstants.PID.pivotKI;
        pivotConfigs.Slot0.kA= IntakeConstants.PID.pivotKA;
        pivotConfigs.Slot0.kG= IntakeConstants.PID.pivotKG;
        pivotConfigs.Slot0.kS= IntakeConstants.PID.pivotKS;

        //Soft Limit Configurations
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable= true;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold= IntakeConstants.Positions.pivotMaxAngle.in(Degrees);
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable= true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold= IntakeConstants.Positions.pivotMinAngle.in(Degrees);

        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfigs, 0.25));


        //Motion Magic Configurations for deployed to waiting positons
        final MotionMagicConfigs deployedWaitingConfigs= new MotionMagicConfigs();

        deployedWaitingConfigs.MotionMagicCruiseVelocity= IntakeConstants.MotionMagic.deployedToWaitingVelocity.in(RotationsPerSecond); 
        deployedWaitingConfigs.MotionMagicAcceleration= IntakeConstants.MotionMagic.deployedToWaitingAcceleration.in(RotationsPerSecondPerSecond); 
        deployedWaitingConfigs.MotionMagicJerk= IntakeConstants.MotionMagic.jerk; 

        this.deployedWaitingMotionMagic= deployedWaitingConfigs;


        //Motion Magic Configuration for stowed to deployed positions
        final MotionMagicConfigs stowedDeployedConfigs= new MotionMagicConfigs();

        stowedDeployedConfigs.MotionMagicCruiseVelocity= IntakeConstants.MotionMagic.stowedToDeployedVelocity.in(RotationsPerSecond);
        stowedDeployedConfigs.MotionMagicAcceleration= IntakeConstants.MotionMagic.stowedToDeployedAcceleration.in(RotationsPerSecondPerSecond);
        stowedDeployedConfigs.MotionMagicJerk= IntakeConstants.MotionMagic.jerk;

        this.stowedDeployedMotionMagic= stowedDeployedConfigs;

        //Motion Magic Configuration for deployed to stowed positions
        final MotionMagicConfigs deployedStowedConfigs= new MotionMagicConfigs();

        deployedStowedConfigs.MotionMagicCruiseVelocity= IntakeConstants.MotionMagic.deployedToStowedVelocity.in(RotationsPerSecond);
        deployedStowedConfigs.MotionMagicAcceleration= IntakeConstants.MotionMagic.deployedToStowedAcceleration.in(RotationsPerSecondPerSecond);
        deployedStowedConfigs.MotionMagicJerk= IntakeConstants.MotionMagic.jerk;
        
        this.deployedStowedMotionMagic= deployedStowedConfigs;
        
        // Configuration for the roller motor
        final TalonFXConfiguration rollerConfigs= new TalonFXConfiguration();
        
        //Setting roller motor to Coast mode
        rollerConfigs.MotorOutput.NeutralMode= NeutralModeValue.Coast;

        //Current Limit Configurations
        rollerConfigs.CurrentLimits.StatorCurrentLimit= IntakeConstants.CurrentLimits.currentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable= true;

        //PID Configurations
        rollerConfigs.Slot0.kP= IntakeConstants.PID.rollerKP;
        rollerConfigs.Slot0.kD= IntakeConstants.PID.rollerKD;
        rollerConfigs.Slot0.kV= IntakeConstants.PID.rollerKV;
        rollerConfigs.Slot0.kI= IntakeConstants.PID.rollerKI;
        rollerConfigs.Slot0.kA= IntakeConstants.PID.rollerKA;
        rollerConfigs.Slot0.kS= IntakeConstants.PID.rollerKS;
        rollerConfigs.Slot0.kG= IntakeConstants.PID.rollerKG;

        //Used to apply configs once instead of having multiple iterations to do this
        //Also trys the configuartion 5 times until it receives an OK status signal 
        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfigs, 0.25));

        // Initialize all status signals
        pivotAngleSignal = pivotMotor.getPosition();
        pivotAngularVelocitySignal = pivotMotor.getVelocity();
        rollerAngularVelocitySignal = rollerMotor.getVelocity();
        rollerAppliedVoltsSignal = rollerMotor.getMotorVoltage();
        pivotAppliedVoltsSignal = pivotMotor.getMotorVoltage();
        rollerCurrentAmpsSignal = rollerMotor.getStatorCurrent();
        pivotCurrentAmpsSignal = pivotMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(20, rollerAppliedVoltsSignal, rollerCurrentAmpsSignal,pivotAppliedVoltsSignal, pivotCurrentAmpsSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivotAngleSignal,pivotAngularVelocitySignal,rollerAngularVelocitySignal);

        // Optimize CAN bus for these parent devices-motors
        ParentDevice.optimizeBusUtilizationForAll(rollerMotor, pivotMotor);
        
    }
    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        var pivotStatus = BaseStatusSignal.refreshAll(
            pivotAngleSignal,
            pivotAngularVelocitySignal,
            pivotAppliedVoltsSignal,
            pivotCurrentAmpsSignal
        );

        var rollerStatus = BaseStatusSignal.refreshAll(
            rollerAngularVelocitySignal,
            rollerAppliedVoltsSignal,
            rollerCurrentAmpsSignal
        );

        inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
        inputs.PivotAngle = pivotAngleSignal.getValue();
        inputs.PivotAngularVelocity = pivotAngularVelocitySignal.getValue();
        inputs.PivotAppliedVolts = pivotAppliedVoltsSignal.getValue();
        inputs.PivotCurrentAmps = pivotCurrentAmpsSignal.getValue();

        inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
        inputs.RollerAngularVelocity = rollerAngularVelocitySignal.getValue();
        inputs.RollerAppliedVolts = rollerAppliedVoltsSignal.getValue();
        inputs.RollerCurrentAmps = rollerCurrentAmpsSignal.getValue();
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

    @Override
    public void deployedWaitingPivot(){
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(deployedWaitingMotionMagic, 0.25));
    }
    @Override
    public void stowedDeployedPivot(){
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(stowedDeployedMotionMagic, 0.25));
    }

    @Override
    public void deployedStowedPivot(){
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(deployedStowedMotionMagic, 0.25));
    }
}