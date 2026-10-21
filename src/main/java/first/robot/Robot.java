/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import first.robot.simulation.DrivetrainSim;
import first.robot.simulation.FuelSim;
import first.robot.simulation.SingleFlywheelSim;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;

/**
 * The methods in this class are called automatically as described in the OpModeRobot documentation.
 * OpMode classes anywhere in the package (or sub-packages) where this class is located are
 * automatically registered to display in the Driver Station. If you change the name of this class
 * or the package after creating this project, you must also update the Main.java file in the
 * project.
 */
public class Robot extends OpModeRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private final SparkMax leftLeader = new SparkMax(CANPort.CAN_S0, 0, MotorType.kBrushless);

  private final SparkMax leftFollower = new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(CANPort.CAN_S0, 3, MotorType.kBrushless);
  public final SparkMax intakeLauncher = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);
  public final SparkMax feeder = new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless);

  public final DifferentialDrive drivetrain =
      new DifferentialDrive(leftLeader::setThrottle, rightLeader::setThrottle);

  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

  private final DrivetrainSim drivetrainSim = new DrivetrainSim(leftLeader, rightLeader);

  private final SingleFlywheelSim intakeLauncherSim =
      SingleFlywheelSim.forIntakeLauncher(intakeLauncher);

  private final SingleFlywheelSim feederSim = SingleFlywheelSim.forFeeder(feeder);

  public Robot() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(true);
    leftLeader.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(
        leftConfig.follow(leftLeader),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(false);
    rightLeader.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(
        rightConfig.follow(rightLeader),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.periodic();
    intakeLauncherSim.periodic();
    feederSim.periodic();
    FuelSim.periodic();
  }
}
