package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.shooter.aiming.shooting.ShotData;

public final class AimingConstants {

    // Cant implement InverseInterpolator on Distance, so K is a double in meters :/
    public static final InterpolatingTreeMap<Double, ShotData> SHOT_DATA_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

    static {
        SHOT_DATA_MAP.put(1.2, new ShotData(Units.Degrees.of(6.0), Units.MetersPerSecond.of(17.29)));
        SHOT_DATA_MAP.put(1.84, new ShotData(Units.Degrees.of(12.0), Units.MetersPerSecond.of(17.29)));
        SHOT_DATA_MAP.put(2.8, new ShotData(Units.Degrees.of(15.5), Units.MetersPerSecond.of(18.08)));
        SHOT_DATA_MAP.put(3.8, new ShotData(Units.Degrees.of(19.0), Units.MetersPerSecond.of(19.42)));
        SHOT_DATA_MAP.put(4.8, new ShotData(Units.Degrees.of(22.0), Units.MetersPerSecond.of(21.01)));
        SHOT_DATA_MAP.put(5.8, new ShotData(Units.Degrees.of(25.0), Units.MetersPerSecond.of(22.34)));
        SHOT_DATA_MAP.put(6.8, new ShotData(Units.Degrees.of(28.0), Units.MetersPerSecond.of(24.20)));

        TOF_MAP.put(1.2, 1.24);
        TOF_MAP.put(1.84, 1.14);
        TOF_MAP.put(2.8, 1.13);
        TOF_MAP.put(3.8, 1.20);
        TOF_MAP.put(4.8, 1.24);
        TOF_MAP.put(5.8, 1.36);
        TOF_MAP.put(6.8, 1.41);
    }

}
