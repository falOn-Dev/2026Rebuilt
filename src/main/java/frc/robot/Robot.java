// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.deploy.IntakeDeploy;
import frc.robot.subsystems.intake.deploy.IntakeDeployConstants;
import frc.robot.subsystems.intake.deploy.IntakeDeployIO;
import frc.robot.subsystems.intake.deploy.IntakeDeployIOSim;
import frc.robot.subsystems.intake.deploy.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRollerConstants;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerIOTalonFX;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PhoenixUtil;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    public final Drive drive;
    public final Intake intake;

    public final CommandXboxController controller = new CommandXboxController(0);

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        switch (Constants.CURRENT_MODE) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                intake = new Intake(
                        new IntakeDeploy(
                                new IntakeDeployIOTalonFX(
                                        IntakeDeployConstants.LEFT_MOTOR_ID,
                                        IntakeDeployConstants.RIGHT_MOTOR_ID,
                                        IntakeDeployConstants.LEFT_INVERT,
                                        IntakeDeployConstants.RIGHT_INVERT,
                                        IntakeDeployConstants.GEARING,
                                        IntakeDeployConstants.PID_CONSTANTS,
                                        IntakeDeployConstants.FF_CONSTANTS,
                                        IntakeDeployConstants.MM_VELOCITY,
                                        IntakeDeployConstants.MM_ACCELERATION)),
                        new IntakeRoller(
                                new IntakeRollerIOTalonFX(
                                        IntakeRollerConstants.MOTOR_ID,
                                        IntakeRollerConstants.INVERTED,
                                        IntakeRollerConstants.GEARING,
                                        IntakeRollerConstants.PID_CONSTANTS,
                                        IntakeRollerConstants.FF_CONSTANTS)));
                break;

            case SIM:
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                intake = new Intake(
                        new IntakeDeploy(
                                new IntakeDeployIOSim(
                                        IntakeDeployConstants.GEARING,
                                        IntakeDeployConstants.PID_CONSTANTS,
                                        IntakeDeployConstants.FF_CONSTANTS,
                                        IntakeDeployConstants.MOI)),
                        new IntakeRoller(
                                new IntakeRollerIOSim(
                                        IntakeRollerConstants.GEARING,
                                        IntakeRollerConstants.PID_CONSTANTS,
                                        IntakeRollerConstants.FF_CONSTANTS,
                                        IntakeRollerConstants.MOI)));
                break;
            default:
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });

                intake = new Intake(
                        new IntakeDeploy(new IntakeDeployIO() {
                        }),
                        new IntakeRoller(new IntakeRollerIO() {
                        }));
                break;
        }

        configureBindings();
    }

    public void configureBindings() {
        DoublePressTracker intakeRetractTracker = new DoublePressTracker(controller.leftTrigger());
        Trigger retractIntake = new Trigger(() -> intakeRetractTracker.get());

        // Deploy intake while left trigger is held and the intake has been homed since robot startup
        // If the intake is already deployed it should just spin the roller up, untested currently
        controller.leftTrigger().and(intake.isInit.negate()).whileTrue(intake.intake());

        // Pull intake in if its deployed and double tap trigger
        retractIntake.and(intake.isDeployed).onTrue(intake.stow());
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();
        PhoenixUtil.refreshAll();
        LoggedTracer.record("PhoenixRefresh");

        CommandScheduler.getInstance().run();
        LoggedTracer.record("CommandRun");
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        CommandScheduler.getInstance().schedule(intake.home());
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
