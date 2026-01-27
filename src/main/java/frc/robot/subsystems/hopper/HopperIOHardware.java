package frc.robot.subsystems.hopper;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.*;

public class HopperIOHardware implements HopperIO {
    private TalonFX feederMotor;

    private final VelocityVoltage feederVelocityRequest  = new VelocityVoltage(0.0);

    public HopperIOHardware() {
        feederMotor = new TalonFX(HopperConstants.feederMotorCANID);
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = HopperConstants.feederKP;
        feederConfig.Slot0.kI = HopperConstants.feederKI;
        feederConfig.Slot0.kD = HopperConstants.feederKD;
        feederConfig.Slot0.kS = HopperConstants.feederKS;
        feederConfig.Slot0.kV = HopperConstants.feederKV;
        feederConfig.Slot0.kA = HopperConstants.feederKA;
    }



}
