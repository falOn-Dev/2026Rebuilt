package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public IntakeRollerIOTalonFX(
            int motorId,
            boolean inverted,
            double gearing,
            PIDConstants pidConstants,
            FFConstants ffConstants) {
        motor = new TalonFX(motorId, Constants.MECHANISM_CAN_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.SUPPLY_LIMIT.in(Units.Amps);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.STATOR_LIMIT.in(Units.Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = gearing;

        config.Slot0 = new Slot0Configs()
                .withKP(pidConstants.kP())
                .withKI(pidConstants.kI())
                .withKD(pidConstants.kD())
                .withKS(ffConstants.kS())
                .withKV(ffConstants.kV());

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent, statorCurrent);

        motor.optimizeBusUtilization();

        PhoenixUtil.registerSignals(position, velocity, appliedVoltage, supplyCurrent, statorCurrent);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, supplyCurrent, statorCurrent);

        inputs.appliedVoltage.mut_replace(appliedVoltage.getValue());
        inputs.angularPosition.mut_replace(position.getValue());
        inputs.angularVelocity.mut_replace(velocity.getValue());
        inputs.linearVelocity.mut_replace(
                inputs.angularVelocity.in(Units.RadiansPerSecond) * IntakeRollerConstants.ROLLER_RADIUS.in(Units.Meters),
                Units.MetersPerSecond);
        inputs.supplyCurrent.mut_replace(supplyCurrent.getValue());
        inputs.statorCurrent.mut_replace(statorCurrent.getValue());
    }

    @Override
    public void requestVoltage(Voltage volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void requestVelocity(AngularVelocity velocity) {
        motor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }
}
