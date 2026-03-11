package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public final class DriveConstants {
        public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(TunerConstants.kSpeedAt12Volts,
                        Units.MetersPerSecondPerSecond.of(20.0), Units.RotationsPerSecond.of(2.0),
                        Units.RotationsPerSecondPerSecond.of(4.0));

        public static final PathConstraints SLOWMODE_CONSTRAINTS = new PathConstraints(Units.MetersPerSecond.of(5.0),
                        Units.MetersPerSecondPerSecond.of(20.0), Units.RotationsPerSecond.of(2.0),
                        Units.RotationsPerSecondPerSecond.of(4.0));

        public static final PathConstraints SIM_CONSTRAINTS = new PathConstraints(TunerConstants.kSpeedAt12Volts,
                        Units.MetersPerSecondPerSecond.of(100.0), Units.RotationsPerSecond.of(2.0),
                        Units.RotationsPerSecondPerSecond.of(40.0));

        public static final Distance ROBOT_WIDTH = Units.Inches.of(35.125);
        public static final Distance ROBOT_LENGTH = Units.Inches.of(35.125);
        public static final Distance BUMPER_HEIGHT = Units.Inches.of(6.0);
}
