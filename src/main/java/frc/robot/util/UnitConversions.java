package frc.robot.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public final class UnitConversions {
    public static LinearVelocity angularVelocityToLinearVelocity(AngularVelocity angularVelocity, Distance radius) {
        return Units.MetersPerSecond.of(radius.in(Units.Meters) * angularVelocity.in(Units.RadiansPerSecond));
    }

    public static Distance angularPositionToLinearPosition(Angle angle, Distance radius) {
        return Units.Meters.of(radius.in(Units.Meters) * angle.in(Units.Radians));
    }

    public static AngularVelocity linearVelocityToAngularVelocity(LinearVelocity linearVelocity, Distance radius) {
        return Units.RadiansPerSecond.of(linearVelocity.in(Units.MetersPerSecond) / radius.in(Units.Meters));
    }

    public static LinearAcceleration angularAccelerationToLinearAcceleration(
            AngularAcceleration angularAcceleration,
            Distance radius) {
        return Units.MetersPerSecondPerSecond.of(
                radius.in(Units.Meters) * angularAcceleration.in(Units.RadiansPerSecondPerSecond));
    }

    public static AngularAcceleration linearAccelerationToAngularAcceleration(
            LinearAcceleration linearAcceleration,
            Distance radius) {
        return Units.RadiansPerSecondPerSecond.of(
                linearAcceleration.in(Units.MetersPerSecondPerSecond) / radius.in(Units.Meters));
    }

    public static Angle linearPositionToAngularPosition(Distance distance, Distance radius) {
        return Units.Radians.of(distance.in(Units.Meters) / radius.in(Units.Meters));
    }
}
