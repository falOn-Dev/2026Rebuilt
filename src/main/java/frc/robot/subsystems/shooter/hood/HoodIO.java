package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public MutVoltage appliedVoltage = Units.Volts.mutable(0.0);
        public MutAngle angularPosition = Units.Rotations.mutable(0.0);
        public MutAngularVelocity angularVelocity = Units.RotationsPerSecond.mutable(0.0);
        public MutCurrent supplyCurrent = Units.Amps.mutable(0.0);
        public MutCurrent statorCurrent = Units.Amps.mutable(0.0);
    }

    public default void updateInputs(HoodIOInputs inputs) {
    }

    public default void requestVoltage(Voltage voltage) {
    }

    public default void requestPosition(Angle position) {
    }

    public default void setEncoderPosition(Angle position) {
    }

    public default void resetEncoderPosition() {
    }

    public default void stop() {
    }
}
