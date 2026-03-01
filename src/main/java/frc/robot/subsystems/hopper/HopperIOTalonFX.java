package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {
    
    //Creating Motor Objects
    protected final TalonFX feederMotor;
    protected final TalonFX scramblerMotor;

    //Feeder Control Requests
    private final VoltageOut feederVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage feederVelocityRequest  = new VelocityVoltage(0.0);

    //Scrambler Control Requests
    private final VoltageOut scramblerVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage scramblerVelocityRequest  = new VelocityVoltage(0.0);

    //Feeder Status Signals
    private final StatusSignal<AngularVelocity> feederAngularVelocitySignal;
    private final StatusSignal<Voltage> feederVoltageSignal;
    private final StatusSignal<Current> feederCurrentSignal; 

    //Scrambler Status Signals
    private final StatusSignal<AngularVelocity> scramblerAngularVelocitySignal;   
    private final StatusSignal<Voltage> scramblerVoltageSignal;
    private final StatusSignal<Current> scramblerCurrentSignal;

    private final Debouncer feederConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer scramblerConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public HopperIOTalonFX(CANBus canBus) {
        
        //Feeder Motor Configuration
        feederMotor = new TalonFX(HopperConstants.feederMotorCANID, canBus);
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = HopperConstants.feederKP;
        feederConfig.Slot0.kI = HopperConstants.feederKI;
        feederConfig.Slot0.kD = HopperConstants.feederKD;
        feederConfig.Slot0.kS = HopperConstants.feederKS;
        feederConfig.Slot0.kV = HopperConstants.feederKV;
        feederConfig.Slot0.kA = HopperConstants.feederKA;

        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive ;
        feederConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.feederCurrentLimit ;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig, 0.25));

        //Status Signals Initialization
        feederAngularVelocitySignal = feederMotor.getVelocity();
        feederVoltageSignal = feederMotor.getMotorVoltage();
        feederCurrentSignal = feederMotor.getSupplyCurrent();        

        // Scrambler Motor Configuration
        scramblerMotor = new TalonFX(HopperConstants.scramblerMotorCANID);

        TalonFXConfiguration scramblerConfig = new TalonFXConfiguration();

        scramblerConfig.Slot0.kP = HopperConstants.scramblerKP;
        scramblerConfig.Slot0.kI = HopperConstants.scramblerKI;
        scramblerConfig.Slot0.kD = HopperConstants.scramblerKD;
        scramblerConfig.Slot0.kS = HopperConstants.scramblerKS;
        scramblerConfig.Slot0.kV = HopperConstants.scramblerKV;
        scramblerConfig.Slot0.kA = HopperConstants.scramblerKA;
        scramblerConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.scramblerCurrentLimit.in(Amps);
        scramblerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> scramblerMotor.getConfigurator().apply(scramblerConfig, 0.25));



        scramblerAngularVelocitySignal = scramblerMotor.getVelocity();
        scramblerVoltageSignal = scramblerMotor.getMotorVoltage();
        scramblerCurrentSignal = scramblerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            feederAngularVelocitySignal, feederVoltageSignal, feederCurrentSignal);
        
        feederMotor.optimizeBusUtilization();
        
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            scramblerAngularVelocitySignal, scramblerVoltageSignal, scramblerCurrentSignal);
        
        scramblerMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        StatusCode feederStatus = BaseStatusSignal.refreshAll(
            feederAngularVelocitySignal, feederVoltageSignal, feederCurrentSignal);
        
        StatusCode scramblerStatus = BaseStatusSignal.refreshAll(
            scramblerAngularVelocitySignal, scramblerVoltageSignal, scramblerCurrentSignal);
        
        inputs.feederConnected = feederConnectedDebounce.calculate(feederStatus.isOK());
        inputs.feederVelocity = feederAngularVelocitySignal.getValue();
        inputs.feederVoltage = feederVoltageSignal.getValue();
        inputs.feederCurrent = feederCurrentSignal.getValue();

        inputs.scramblerConnected = scramblerConnectedDebounce.calculate(scramblerStatus.isOK());
        inputs.scramblerVelocity = scramblerAngularVelocitySignal.getValue();
        inputs.scramblerVoltage = scramblerVoltageSignal.getValue();
        inputs.scramblerCurrent = scramblerCurrentSignal.getValue();
    }

    @Override
    public void setFeederVoltage(Voltage voltage) {
        feederMotor.setControl(feederVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setFeederVelocity(AngularVelocity velocity) {
        feederMotor.setControl(feederVelocityRequest.withVelocity(velocity));
    }

    @Override
    public void setScramblerVoltage(Voltage voltage) {
        scramblerMotor.setControl(scramblerVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setScramblerVelocity(AngularVelocity velocity) {
        scramblerMotor.setControl(scramblerVelocityRequest.withVelocity(velocity));
    }

    public void stopFeeder() {
        feederMotor.setControl(new StaticBrake());
    }

    public void stopScrambler() {
        scramblerMotor.setControl(new StaticBrake());
    }
}