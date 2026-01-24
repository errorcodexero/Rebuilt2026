package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.generated.CompTunerConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;


public final class IntakeIOHardware implements IntakeIO {

    private final TalonFX rollerMotor;
    private final TalonFX pivotMotor;
    private StatusSignal<Angle> pivotAngleSignal;
    private StatusSignal<AngularVelocity> pivotAngularVelocitySignal;
    private StatusSignal<Angle> rollerAngleSignal;
    private StatusSignal<AngularVelocity> rollerAngularVelocitySignal;
    private StatusSignal<Voltage> rollerAppliedVoltsSignal;
    private StatusSignal<Voltage> pivotAppliedVoltsSignal;
    private StatusSignal<Current> rollerCurrentAmpsSignal;
    private StatusSignal<Current> pivotCurrentAmpsSignal;

    // Control requests
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0.0);

    public IntakeIOHardware() {
    rollerMotor = new TalonFX(IntakeConstants.rollerMotorCANID, CompTunerConstants.kCANBus);
    pivotMotor = new TalonFX(IntakeConstants.pivotMotorCANID, CompTunerConstants.kCANBus);
        var limitConfigsPivot = new CurrentLimitsConfigs();
        limitConfigsPivot.SupplyCurrentLimit = IntakeConstants.currentLimit;
        limitConfigsPivot.SupplyCurrentLimitEnable = true;
        pivotMotor.getConfigurator().apply(limitConfigsPivot);

        var limitConfigsRoller= new CurrentLimitsConfigs();
        limitConfigsRoller.SupplyCurrentLimit= IntakeConstants.currentLimit;
        limitConfigsRoller.SupplyCurrentLimitEnable= true;
        rollerMotor.getConfigurator().apply(limitConfigsRoller);

        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = IntakeConstants.pivotKP;
        pivotSlot0Configs.kD= IntakeConstants.pivotKD;
        pivotSlot0Configs.kV= IntakeConstants.pivotKV;
        pivotSlot0Configs.kI= IntakeConstants.pivotKI;
        pivotMotor.getConfigurator().apply(pivotSlot0Configs);

        Slot0Configs rollerSlot0Configs= new Slot0Configs();
        rollerSlot0Configs.kP= IntakeConstants.rollerKP;
        rollerSlot0Configs.kD= IntakeConstants.rollerKD;
        rollerSlot0Configs.kV= IntakeConstants.rollerKV;
        rollerSlot0Configs.kI= IntakeConstants.rollerKI;
        rollerMotor.getConfigurator().apply(rollerSlot0Configs); 

        SoftwareLimitSwitchConfigs pivotSoftLimitSwitchConfigs= new SoftwareLimitSwitchConfigs();
        pivotSoftLimitSwitchConfigs.ForwardSoftLimitEnable= true;
        pivotSoftLimitSwitchConfigs.ForwardSoftLimitThreshold= IntakeConstants.deployedAngle.in(Degrees);
        pivotSoftLimitSwitchConfigs.ReverseSoftLimitEnable= true;
        pivotSoftLimitSwitchConfigs.ReverseSoftLimitThreshold= IntakeConstants.stowedAngle.in(Degrees);
        pivotMotor.getConfigurator().apply(pivotSoftLimitSwitchConfigs);

        var MotionMagicConfigsPivot= new MotionMagicConfigs();
        MotionMagicConfigsPivot.MotionMagicCruiseVelocity= IntakeConstants.pivotMaxVelocity.in(DegreesPerSecond); 
        MotionMagicConfigsPivot.MotionMagicAcceleration= 0; 
        pivotMotor.getConfigurator().apply(MotionMagicConfigsPivot);

        

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, rollerAngularVelocitySignal, rollerAppliedVoltsSignal, rollerCurrentAmpsSignal, pivotAngularVelocitySignal, pivotAppliedVoltsSignal, pivotCurrentAmpsSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(20.0, rollerAngleSignal, pivotAngleSignal);

        // Optimize CAN bus for these parent devices
        ParentDevice.optimizeBusUtilizationForAll(rollerMotor, pivotMotor);
    }

}