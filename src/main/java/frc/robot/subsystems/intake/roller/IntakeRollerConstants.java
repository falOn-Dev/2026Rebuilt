package frc.robot.subsystems.intake.roller;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

public final class IntakeRollerConstants {
    public static final int MOTOR_ID = 25;
    public static final boolean INVERTED = false;

    public static final double GEARING = 32/12;
    public static final AngularVelocity MAX_VELOCITY = Units.RPM.of(7300).div(GEARING);
    public static final AngularVelocity INTAKE_VELOCITY = Units.RPM.of(5000.0);
    public static final Distance ROLLER_RADIUS = Units.Inches.of(1.0);
    public static final MomentOfInertia MOI = Units.KilogramSquareMeters.of(0.0001519341);

    public static final PIDConstants PID_CONSTANTS = new PIDConstants(10.0, 0.0, 0.0);
    public static final FFConstants FF_CONSTANTS = new FFConstants(0.0, 0.12, 0.0, 0.0);

    public static final Current STATOR_LIMIT = Units.Amps.of(120.0);
    public static final Current SUPPLY_LIMIT = Units.Amps.of(60.0);
}
