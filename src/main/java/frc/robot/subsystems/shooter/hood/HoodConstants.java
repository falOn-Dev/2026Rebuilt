package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

public final class HoodConstants {
    public static final int MOTOR_ID = 12;
    public static final boolean INVERTED = true;

    public static final double GEARING = 72.0;

    public static final Angle MIN_ANGLE = Units.Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Units.Degrees.of(30.0);

    public static final PIDConstants PID_CONSTANTS = new PIDConstants(200.0, 0.0, 5.0);
    public static final FFConstants FF_CONSTANTS = new FFConstants(0.34, 7.0, 0.0, 0.04);
    
    public static final Current STATOR_LIMIT = Units.Amps.of(80.0);
    public static final Current SUPPLY_LIMIT = Units.Amps.of(40.0);
    
    public static final AngularVelocity MM_VELOCITY = Units.RotationsPerSecond.of(1.0);
    public static final AngularAcceleration MM_ACCEL = Units.RotationsPerSecondPerSecond.of(10.0);

}
