package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SurusAltSistemi;

/**
 * Runs a single drive motor at a fixed speed while the command is active.
 * Use from SmartDashboard to test motor wiring/direction safely.
 */
public class MotorTestKomutu extends Command {
  public static final int FRONT_LEFT = 1;
  public static final int FRONT_RIGHT = 2;
  public static final int REAR_LEFT = 3;
  public static final int REAR_RIGHT = 4;

  private final SurusAltSistemi drive;
  private final int motorIndex;
  private final double speed;

  public MotorTestKomutu(SurusAltSistemi drive, int motorIndex, double speed) {
    this.drive = drive;
    this.motorIndex = motorIndex;
    this.speed = speed;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // nothing
  }

  @Override
  public void execute() {
    switch (motorIndex) {
      case FRONT_LEFT:
        drive.testSetFrontLeft(speed);
        break;
      case FRONT_RIGHT:
        drive.testSetFrontRight(speed);
        break;
      case REAR_LEFT:
        drive.testSetRearLeft(speed);
        break;
      case REAR_RIGHT:
        drive.testSetRearRight(speed);
        break;
      default:
        // unknown
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.tumMotorlariDurdur();
  }

  @Override
  public boolean isFinished() {
    return false; // run while active
  }
}
