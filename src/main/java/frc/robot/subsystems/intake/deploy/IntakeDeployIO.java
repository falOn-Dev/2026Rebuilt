package frc.robot.subsystems.intake.deploy;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeDeployIO {
    @AutoLog
    public static class IntakeDeployInputs {
        // Left Side Data
        public MutVoltage leftAppliedVoltage = Units.Volts.mutable(0.0);
        public MutAngle leftAngularPosition = Units.Rotations.mutable(0.0);
        public MutDistance leftLinearPosition = Units.Meters.mutable(0.0);

        public MutAngularVelocity leftAngularVelocity = Units.RotationsPerSecond.mutable(0.0);
        public MutLinearVelocity leftLinearVelocity = Units.MetersPerSecond.mutable(0.0);

        public MutCurrent leftSupplyCurrent = Units.Amps.mutable(0.0);
        public MutCurrent leftStatorCurrent = Units.Amps.mutable(0.0);

        // Right Side Data
        public MutVoltage rightAppliedVoltage = Units.Volts.mutable(0.0);
        public MutAngle rightAngularPosition = Units.Rotations.mutable(0.0);
        public MutDistance rightLinearPosition = Units.Meters.mutable(0.0);

        public MutAngularVelocity rightAngularVelocity = Units.RotationsPerSecond.mutable(0.0);
        public MutLinearVelocity rightLinearVelocity = Units.MetersPerSecond.mutable(0.0);

        public MutCurrent rightSupplyCurrent = Units.Amps.mutable(0.0);
        public MutCurrent rightStatorCurrent = Units.Amps.mutable(0.0);
    }

    public default void updateInputs(IntakeDeployInputs inputs) {
    }

    public default void requestLeftVoltage(Voltage voltage) {
    }

    public default void requestRightVoltage(Voltage voltage) {
    }

    public default void requestExtension(Distance extension) {
    }

    public default void stopLeft() {
    }

    public default void stopRight() {
    }

    public default void stop() {
        stopLeft();
        stopRight();
    }

    public default void setLeftEncoderPosition(Angle position) {}
    public default void resetLeftEncoderPosition() {}

    public default void setRightEncoderPosition(Angle position) {}
    public default void resetRightEncoderPosition() {}
}
