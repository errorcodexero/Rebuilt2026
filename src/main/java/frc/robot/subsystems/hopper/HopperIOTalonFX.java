package frc.robot.subsystems.hopper;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.*;

import static edu.wpi.first.units.Units.Amps;

public class HopperIOTalonFX implements HopperIO {
    private TalonFX feederMotor;
    private TalonFX scramblerMotor;

    private final VelocityVoltage feederVelocityRequest  = new VelocityVoltage(0.0);
    private final VelocityVoltage scramblerVelocityRequest  = new VelocityVoltage(0.0);

    public HopperIOTalonFX() {
        
        //Feeder Motor Configuration
        feederMotor = new TalonFX(HopperConstants.feederMotorCANID);
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = HopperConstants.feederKP;
        feederConfig.Slot0.kI = HopperConstants.feederKI;
        feederConfig.Slot0.kD = HopperConstants.feederKD;
        feederConfig.Slot0.kS = HopperConstants.feederKS;
        feederConfig.Slot0.kV = HopperConstants.feederKV;
        feederConfig.Slot0.kA = HopperConstants.feederKA;

        feederMotor.getConfigurator().apply(feederConfig);

        feederConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.feederCurrentLimit.in(Amps);

        //Scrambler Motor Configuration
        scramblerMotor = new TalonFX(HopperConstants.scramblerMotorCANID);
        TalonFXConfiguration scramblerConfig = new TalonFXConfiguration();

        scramblerConfig.Slot0.kP = HopperConstants.scramblerKP;
        scramblerConfig.Slot0.kI = HopperConstants.scramblerKI;
        scramblerConfig.Slot0.kD = HopperConstants.scramblerKD;
        scramblerConfig.Slot0.kS = HopperConstants.scramblerKS;
        scramblerConfig.Slot0.kV = HopperConstants.scramblerKV;
        scramblerConfig.Slot0.kA = HopperConstants.scramblerKA;

        scramblerMotor.getConfigurator().apply(scramblerConfig);

        scramblerConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.scramblerCurrentLimit.in(Amps);
    }
}