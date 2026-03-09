package frc.robot.subsystems.shooter.flywheel;

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

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean isConnected = false;
        public MutVoltage appliedVoltage = Units.Volts.mutable(0.0);
        public MutAngle angularPosition = Units.Rotations.mutable(0.0);
        public MutAngularVelocity angularVelocity = Units.RotationsPerSecond.mutable(0.0);
        public MutLinearVelocity linearVelocity = Units.MetersPerSecond.mutable(0.0);
        public MutCurrent supplyCurrent = Units.Amps.mutable(0.0);
        public MutCurrent statorCurrent = Units.Amps.mutable(0.0);
    }

    public default void updateInputs(FlywheelIOInputs inputs) {
    }

    public default void requestVoltage(Voltage voltage) {
    }

    public default void requestAngularVelocity(AngularVelocity velocity) {
    }

    public default void requestLinearVelocity(LinearVelocity velocity) {
        requestAngularVelocity(
                Units.RadiansPerSecond
                        .of(velocity.in(Units.MetersPerSecond) / FlywheelConstants.FLYWHEEL_RADIUS.in(Units.Meters)));
    }

    public default void stop() {
    }
}
