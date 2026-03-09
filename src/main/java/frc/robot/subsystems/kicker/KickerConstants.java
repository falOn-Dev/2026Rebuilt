package frc.robot.subsystems.kicker;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.FFConstants;

public final class KickerConstants {
    public static final int MOTOR_ID = 13;
    public static final boolean INVERTED = true;

    public static final double GEARING = 38/12;
    public static final AngularVelocity MAX_SPEED = Units.RPM.of(7300.0).div(GEARING);
    public static final AngularVelocity FEED_SPEED = Units.RPM.of(2000);
    public static final MomentOfInertia MOI = Units.KilogramSquareMeters.of(0.0006586766);

    public static final FFConstants FF_CONSTANTS = new FFConstants(0.0, 0.0, 0.0, 0.0);

    public static final Current STATOR_LIMIT = Units.Amps.of(80.0);
    public static final Current SUPPLY_LIMIT = Units.Amps.of(60.0);
}
