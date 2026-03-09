package frc.robot.subsystems.intake.deploy;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

public final class IntakeDeployConstants {
    public static final int LEFT_MOTOR_ID = 23;
    public static final int RIGHT_MOTOR_ID = 24;

    public static final boolean LEFT_INVERT = true;
    public static final boolean RIGHT_INVERT = false;

    public static final double GEARING = (60 / 10) * (18 / 18);

    public static final Distance PINION_PITCH_RADIUS = Units.Inches.of(0.5);

    // estimate w/ Mass * Pitch Radius ^ 2
    public static final MomentOfInertia MOI = Units.KilogramSquareMeters.of(0.000512);

    public static final Distance MAX_EXTENSION = Units.Inches.of(12.0);
    public static final Distance INTAKE_POSITION = Units.Inches.of(11.5);
    public static final Distance STOW_POSITION = Units.Inches.of(3.5);

    public static final PIDConstants PID_CONSTANTS = new PIDConstants(4.0, 0.5, 0.0);
    public static final FFConstants FF_CONSTANTS = new FFConstants(0.0, 0.0, 0.0, 0.0);

    public static final LinearVelocity MM_VELOCITY = Units.MetersPerSecond.of(2.0);
    public static final LinearAcceleration MM_ACCELERATION = Units.MetersPerSecondPerSecond.of(10.0);

    public static final Current HOMING_THRESHOLD = Units.Amps.of(20.0);
    public static final Voltage HOMING_VOLTAGE = Units.Volts.of(-3.0);
}
