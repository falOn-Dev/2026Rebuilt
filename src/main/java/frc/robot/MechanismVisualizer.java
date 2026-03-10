package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.hood.HoodConstants;

public final class MechanismVisualizer {
    private static MechanismVisualizer instance;

    public static MechanismVisualizer getInstance() {
        if (instance == null) {
            instance = new MechanismVisualizer();
        }
        return instance;
    }

    private final Pose3d intakeBasePose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    private static final Angle intakeAngle = Degrees.of(-12.0);
    private static final Translation3d intakeExtensionAxis = new Translation3d(
            Math.cos(intakeAngle.in(Units.Radians)),
            0.0,
            Math.sin(intakeAngle.in(Units.Radians)));

    private final Pose3d hoodBasePose = new Pose3d(HoodConstants.HOOD_BASE.getTranslation(), new Rotation3d());

    public void update(Distance intakeExtension, Angle hoodAngle) {
        Pose3d hoodPose = hoodBasePose.transformBy(new Transform3d(new Translation3d(),
                new Rotation3d(Units.Degree.zero(), hoodAngle.unaryMinus(), Units.Degree.zero())));

        Pose3d intakePose = intakeBasePose.transformBy(
                new Transform3d(
                        intakeExtensionAxis.times(intakeExtension.in(Units.Meters)),
                        new Rotation3d()));

        Logger.recordOutput("MechanismVisualizer/Components", hoodPose, intakePose);
    }
}
