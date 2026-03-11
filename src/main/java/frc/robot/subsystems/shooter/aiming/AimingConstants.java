package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.shooter.aiming.shooting.ShotData;
import frc.robot.util.math.TwoVariablePolynomial3rdDegree;

public final class AimingConstants {

    public static final TwoVariablePolynomial3rdDegree hoodPolynomial = TwoVariablePolynomial3rdDegree
            .from(Filesystem.getDeployDirectory().getPath() + "/hoodPolynomial.json");
    public static final TwoVariablePolynomial3rdDegree flywheelPolynomial = TwoVariablePolynomial3rdDegree
            .from(Filesystem.getDeployDirectory().getPath() + "/flywheelPolynomial.json");
    public static final TwoVariablePolynomial3rdDegree tofPolynomial = TwoVariablePolynomial3rdDegree
            .from(Filesystem.getDeployDirectory().getPath() + "/tofPolynomial.json");

    // Cant implement InverseInterpolator on Distance, so K is a double in meters :/
    public static final InterpolatingTreeMap<Double, ShotData> SHOT_DATA_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();
/*
 * private static final double[] SHOT_MAP_LEFT_RPM_REAL = { 3250.0, 3350.0, 3550.0, 3800.0, 4200.0, 4300.0, 4650.0 };
    private static final double[] SHOT_MAP_RIGHT_RPM_REAL = { 3250.0, 3350.0, 3550.0, 3800.0, 4200.0, 4300.0, 4650.0 };
    private static final double[] SHOT_MAP_HOOD_ANGLE_DEG_REAL = {6.0, 14.5, 17.5, 21.5, 24.5, 27.0, 30.0};
    
 */
    static {
        SHOT_DATA_MAP.put(1.0, new ShotData(Units.Degrees.of(6.0), Units.MetersPerSecond.of(17.29)));
        SHOT_DATA_MAP.put(2.0, new ShotData(Units.Degrees.of(14.5), Units.MetersPerSecond.of(17.82)));
        SHOT_DATA_MAP.put(3.0, new ShotData(Units.Degrees.of(17.5), Units.MetersPerSecond.of(18.89)));
        SHOT_DATA_MAP.put(4.0, new ShotData(Units.Degrees.of(21.5), Units.MetersPerSecond.of(20.22)));
        SHOT_DATA_MAP.put(5.0, new ShotData(Units.Degrees.of(24.5), Units.MetersPerSecond.of(22.34)));
        SHOT_DATA_MAP.put(6.0, new ShotData(Units.Degrees.of(27.0), Units.MetersPerSecond.of(22.87)));
        SHOT_DATA_MAP.put(7.0, new ShotData(Units.Degrees.of(30.0), Units.MetersPerSecond.of(24.73)));

        TOF_MAP.put(1.2, 1.24);
        TOF_MAP.put(1.84, 1.14);
        TOF_MAP.put(2.8, 1.13);
        TOF_MAP.put(3.8, 1.20);
        TOF_MAP.put(4.8, 1.24);
        TOF_MAP.put(5.8, 1.36);
        TOF_MAP.put(6.8, 1.41);
    }
}
