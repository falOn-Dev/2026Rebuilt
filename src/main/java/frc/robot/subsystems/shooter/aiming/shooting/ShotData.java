package frc.robot.subsystems.shooter.aiming.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * ShotData
 */
public record ShotData(Angle targetHoodAngle, LinearVelocity targetFlywheelVelocity)
        implements Interpolatable<ShotData> {
    @Override
    public ShotData interpolate(ShotData endValue, double t) {
        return new ShotData(
                Units.Radians.of(
                        MathUtil.interpolate(
                                this.targetHoodAngle().in(Units.Radians),
                                endValue.targetHoodAngle().in(Units.Radians),
                                t)),
                Units.MetersPerSecond.of(
                        MathUtil.interpolate(
                                this.targetFlywheelVelocity().in(Units.MetersPerSecond),
                                endValue.targetFlywheelVelocity().in(Units.MetersPerSecond),
                                t)));
    }
}
