package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

public final class FlywheelConstants {
    public static int LEFT_MOTOR_ID = 10;
    public static int RIGHT_MOTOR_ID = 11;
    
    public static boolean LEFT_INVERTED = false;
    public static boolean RIGHT_INVERTED = true;

    public static double SHOOTER_GEARING = 36/32;
    public static AngularVelocity MAX_SPEED = Units.RPM.of(5800.0);

    public static PIDConstants LEFT_PID = new PIDConstants(10, 0, 0);
    public static PIDConstants RIGHT_PID = LEFT_PID;

    public static FFConstants LEFT_FF = new FFConstants(0.1876, 0.002, 0, 0);
    public static FFConstants RIGHT_FF = LEFT_FF;

    public static Distance FLYWHEEL_RADIUS = Units.Inches.of(2.0);


}
