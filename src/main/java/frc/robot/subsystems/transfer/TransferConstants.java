package frc.robot.subsystems.transfer;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.FFConstants;

public final class TransferConstants {
    public static final int MOTOR_ID = 14;

    public static final boolean INVERTED = true;
    public static final double GEARING = 24 / 18;
    public static final AngularVelocity MAX_VELOCITY = Units.RPM.of(5800.0).div(GEARING);
    public static final AngularVelocity FEED_VELOCITY = Units.RPM.of(4000);

    public static final Current STATOR_LIMIT = Units.Amps.of(120.0);
    public static final Current SUPPLY_LIMIT = Units.Amps.of(60.0);

    public static final FFConstants FF_CONSTANTS = new FFConstants(0.05, 12.0 / MAX_VELOCITY.in(Units.RotationsPerSecond), 0.0, 0.0);

    public static final Distance ROLLER_RADIUS = Units.Inches.of(0.625);

    public static final MomentOfInertia ROLLER_MOI = Units.KilogramSquareMeters.of(0.0000242074);
}
