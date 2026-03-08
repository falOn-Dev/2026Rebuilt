package frc.robot.subsystems.kicker;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public final class KickerConstants {
    public static final int MOTOR_ID = 13;
    public static final boolean INVERTED = true;

    public static final double GEARING = 38/12;
    public static final AngularVelocity MAX_SPEED = Units.RPM.of(5800.0).div(GEARING);

    public static final Current STATOR_LIMIT = Units.Amps.of(80.0);
    public static final Current SUPPLY_LIMIT = Units.Amps.of(60.0);
}
