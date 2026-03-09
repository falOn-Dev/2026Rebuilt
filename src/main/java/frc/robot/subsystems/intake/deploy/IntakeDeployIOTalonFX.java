package frc.robot.subsystems.intake.deploy;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.UnitConversions;

public class IntakeDeployIOTalonFX implements IntakeDeployIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftAppliedVoltage;
    private final StatusSignal<Current> leftSupplyCurrent;
    private final StatusSignal<Current> leftStatorCurrent;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightAppliedVoltage;
    private final StatusSignal<Current> rightSupplyCurrent;
    private final StatusSignal<Current> rightStatorCurrent;

    private final MotionMagicVoltage leftPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final MotionMagicVoltage rightPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);

    private final NeutralOut leftNeutralRequest = new NeutralOut();
    private final NeutralOut rightNeutralRequest = new NeutralOut();

    public IntakeDeployIOTalonFX(
            int leftMotorId,
            int rightMotorId,
            boolean leftInverted,
            boolean rightInverted,
            double gearing,
            PIDConstants pidConstants,
            FFConstants ffConstants,
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration) {
        leftMotor = new TalonFX(leftMotorId, Constants.MECHANISM_CAN_BUS);
        rightMotor = new TalonFX(rightMotorId, Constants.MECHANISM_CAN_BUS);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = leftInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.Feedback.SensorToMechanismRatio = gearing;
        leftConfig.Slot0 = new Slot0Configs()
                .withKP(pidConstants.kP())
                .withKI(pidConstants.kI())
                .withKD(pidConstants.kD())
                .withKS(ffConstants.kS())
                .withKV(ffConstants.kV())
                .withKA(ffConstants.kA())
                .withKG(ffConstants.kG());
        leftConfig.MotionMagic.withMotionMagicCruiseVelocity(
                UnitConversions.linearVelocityToAngularVelocity(maxVelocity, IntakeDeployConstants.PINION_PITCH_RADIUS));
        leftConfig.MotionMagic.withMotionMagicAcceleration(
                UnitConversions.linearAccelerationToAngularAcceleration(
                        maxAcceleration,
                        IntakeDeployConstants.PINION_PITCH_RADIUS));

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = rightInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.Feedback.SensorToMechanismRatio = gearing;
        rightConfig.Slot0 = new Slot0Configs()
                .withKP(pidConstants.kP())
                .withKI(pidConstants.kI())
                .withKD(pidConstants.kD())
                .withKS(ffConstants.kS())
                .withKV(ffConstants.kV())
                .withKA(ffConstants.kA())
                .withKG(ffConstants.kG());
        rightConfig.MotionMagic.withMotionMagicCruiseVelocity(
                UnitConversions.linearVelocityToAngularVelocity(maxVelocity, IntakeDeployConstants.PINION_PITCH_RADIUS));
        rightConfig.MotionMagic.withMotionMagicAcceleration(
                UnitConversions.linearAccelerationToAngularAcceleration(
                        maxAcceleration,
                        IntakeDeployConstants.PINION_PITCH_RADIUS));

        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig, 0.25));

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftAppliedVoltage = leftMotor.getMotorVoltage();
        leftSupplyCurrent = leftMotor.getSupplyCurrent();
        leftStatorCurrent = leftMotor.getStatorCurrent();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightAppliedVoltage = rightMotor.getMotorVoltage();
        rightSupplyCurrent = rightMotor.getSupplyCurrent();
        rightStatorCurrent = rightMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leftPosition,
                leftVelocity,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftStatorCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightStatorCurrent);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        PhoenixUtil.registerSignals(
                leftPosition,
                leftVelocity,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftStatorCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightStatorCurrent);
    }

    @Override
    public void updateInputs(IntakeDeployInputs inputs) {
        inputs.isConnected = BaseStatusSignal.isAllGood(
                leftPosition,
                leftVelocity,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftStatorCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightStatorCurrent);

        inputs.leftAppliedVoltage.mut_replace(leftAppliedVoltage.getValue());
        inputs.leftAngularPosition.mut_replace(leftPosition.getValue());
        inputs.leftLinearPosition.mut_replace(
                UnitConversions.angularPositionToLinearPosition(leftPosition.getValue(), IntakeDeployConstants.PINION_PITCH_RADIUS));
        inputs.leftAngularVelocity.mut_replace(leftVelocity.getValue());
        inputs.leftLinearVelocity.mut_replace(
                UnitConversions.angularVelocityToLinearVelocity(leftVelocity.getValue(), IntakeDeployConstants.PINION_PITCH_RADIUS));
        inputs.leftSupplyCurrent.mut_replace(leftSupplyCurrent.getValue());
        inputs.leftStatorCurrent.mut_replace(leftStatorCurrent.getValue());

        inputs.rightAppliedVoltage.mut_replace(rightAppliedVoltage.getValue());
        inputs.rightAngularPosition.mut_replace(rightPosition.getValue());
        inputs.rightLinearPosition.mut_replace(
                UnitConversions.angularPositionToLinearPosition(rightPosition.getValue(), IntakeDeployConstants.PINION_PITCH_RADIUS));
        inputs.rightAngularVelocity.mut_replace(rightVelocity.getValue());
        inputs.rightLinearVelocity.mut_replace(
                UnitConversions.angularVelocityToLinearVelocity(rightVelocity.getValue(), IntakeDeployConstants.PINION_PITCH_RADIUS));
        inputs.rightSupplyCurrent.mut_replace(rightSupplyCurrent.getValue());
        inputs.rightStatorCurrent.mut_replace(rightStatorCurrent.getValue());
    }

    @Override
    public void requestLeftVoltage(Voltage voltage) {
        leftMotor.setControl(leftVoltageRequest.withOutput(voltage));
    }

    @Override
    public void requestRightVoltage(Voltage voltage) {
        rightMotor.setControl(rightVoltageRequest.withOutput(voltage));
    }

    @Override
    public void requestExtension(Distance extension) {
        Angle position = UnitConversions.linearPositionToAngularPosition(extension, IntakeDeployConstants.PINION_PITCH_RADIUS);
        leftMotor.setControl(leftPositionRequest.withPosition(position));
        rightMotor.setControl(rightPositionRequest.withPosition(position));
    }

    @Override
    public void stopLeft() {
        leftMotor.setControl(leftNeutralRequest);
    }

    @Override
    public void stopRight() {
        rightMotor.setControl(rightNeutralRequest);
    }

    @Override
    public void setLeftEncoderPosition(Angle position) {
        leftMotor.setPosition(position);
    }

    @Override
    public void resetLeftEncoderPosition() {
        leftMotor.setPosition(0.0);
    }

    @Override
    public void setRightEncoderPosition(Angle position) {
        rightMotor.setPosition(position);
    }

    @Override
    public void resetRightEncoderPosition() {
        rightMotor.setPosition(0.0);
    }

}
