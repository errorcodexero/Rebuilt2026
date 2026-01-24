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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;


public final class IntakeIOHardware implements IntakeIO {

    private final TalonFX rollerMotor;
    private final TalonFX pivotMotor;

    private StatusSignal <Angle> pivotAngleSignal;
    private StatusSignal <AngularVelocity> pivotAngularVelocitySignal;
    private StatusSignal <Angle> rollerAngleSignal;
    private StatusSignal <AngularVelocity> rollerAngularVelocitySignal;
    private StatusSignal <Voltage> rollerAppliedVoltsSignal;
    private StatusSignal <Voltage> pivotAppliedVoltsSignal;
    private StatusSignal <Current> rollerCurrentAmpsSignal;
    private StatusSignal <Current> pivotCurrentAmpsSignal;

    public IntakeIOHardware() {
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorCANID);
        pivotMotor = new TalonFX(IntakeConstants.pivotMotorCANID);
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimit = IntakeConstants.currentLimit;
        limitConfigs.SupplyCurrentLimitEnable = true;
        pivotMotor.getConfigurator().apply(limitConfigs);

        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = IntakeConstants.pivotKP;
        pivotSlot0Configs.kD= IntakeConstants.pivotKD;
        pivotSlot0Configs.kV= IntakeConstants.pivotKV;
        pivotSlot0Configs.kI= IntakeConstants.pivotKI;
        pivotMotor.getConfigurator().apply(pivotSlot0Configs);
    }
        



}