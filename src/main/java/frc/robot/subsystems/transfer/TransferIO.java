package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface TransferIO {
    @AutoLog
    public static class TransferIOInputs {
        public boolean isConnected = false;
        public MutVoltage appliedVoltage = Units.Volts.mutable(0.0);
        public MutAngle angularPosition = Units.Rotations.mutable(0.0);
        public MutAngularVelocity angularVelocity = Units.RotationsPerSecond.mutable(0.0);
        public MutLinearVelocity linearVelocity = Units.MetersPerSecond.mutable(0.0);
        public MutCurrent supplyCurrent = Units.Amps.mutable(0.0);
        public MutCurrent statorCurrent = Units.Amps.mutable(0.0);
    }

    public default void updateInputs(TransferIOInputs inputs) {
    }

    public default void requestVoltage(Voltage voltage) {
    }

    public default void requestVelocity(AngularVelocity velocity) {
    }

    public default void requestLinearVelocity(LinearVelocity velocity) {
        requestVelocity(Units.RadiansPerSecond.of(velocity.in(Units.MetersPerSecond) / TransferConstants.ROLLER_RADIUS.in(Units.Meters)));
    }

    public default void stop() {
    }
}
