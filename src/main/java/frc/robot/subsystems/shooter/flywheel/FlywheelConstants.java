package frc.robot.subsystems.shooter.flywheel;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

public final class FlywheelConstants {
    public static final int LEFT_MOTOR_ID = 10;
    public static final int RIGHT_MOTOR_ID = 11;
    
    public static final boolean LEFT_INVERTED = false;
    public static final boolean RIGHT_INVERTED = true;

    public static final double GEARING = 36/32;
    public static final AngularVelocity MAX_VELOCITY = Units.RPM.of(5800.0).div(GEARING);
    public static final AngularVelocity IDLE_VELOCITY = Units.RPM.of(2000.0);

    public static final PIDConstants LEFT_PID = new PIDConstants(10, 0, 0);
    public static final PIDConstants RIGHT_PID = LEFT_PID;

    public static final FFConstants LEFT_FF = new FFConstants(0.1876, 0.002, 0, 0);
    public static final FFConstants RIGHT_FF = LEFT_FF;

    public static final Distance FLYWHEEL_RADIUS = Units.Inches.of(2.0);
    public static final MomentOfInertia FLYWHEEL_MOI = Units.KilogramSquareMeters.of(0.0007528205);

    public static final Current STATOR_LIMIT = Units.Amps.of(120.0);
    public static final Current SUPPLY_LIMIT = Units.Amps.of(60.0);
}
