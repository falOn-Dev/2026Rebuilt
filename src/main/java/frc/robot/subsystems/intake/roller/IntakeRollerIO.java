package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** IntakeRollerIO */
public interface IntakeRollerIO {

    @AutoLog
    public static class IntakeRollerIOInputs {
        public boolean isConnected = false;
        public MutVoltage appliedVoltage = Units.Volts.mutable(0.0);
        public MutAngle angularPosition = Units.Rotation.mutable(0.0);
        public MutAngularVelocity angularVelocity = Units.RotationsPerSecond.mutable(0.0);
        public MutLinearVelocity linearVelocity = Units.MetersPerSecond.mutable(0.0);
        public MutCurrent supplyCurrent = Units.Amps.mutable(0.0);
        public MutCurrent statorCurrent = Units.Amps.mutable(0.0);
    }

    public default void updateInputs(IntakeRollerIOInputs inputs) {
    }

    public default void requestVoltage(Voltage volts) {
    }

    public default void requestVelocity(AngularVelocity velocity) {

    }

    public default void stop() {
        requestVoltage(Units.Volts.mutable(0.0));
    }
}
