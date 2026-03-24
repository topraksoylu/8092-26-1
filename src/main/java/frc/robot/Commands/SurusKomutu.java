// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sabitler.SurusKontrolSabitleri;
import frc.robot.Subsystems.SurusAltSistemi;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SurusKomutu extends Command {
  @FunctionalInterface
  public interface SurusCikisi {
    void sur(double yHizi, double xHizi, double zDonus);
  }

  private final SurusCikisi surusCikisi;
  private final DoubleSupplier yHizi;
  private final DoubleSupplier xHizi;
  private final DoubleSupplier zDonusu;
  private final SlewRateLimiter ySinirlayici = new SlewRateLimiter(SurusKontrolSabitleri.OTELEME_SINIRLAMA_ORANI);
  private final SlewRateLimiter xSinirlayici = new SlewRateLimiter(SurusKontrolSabitleri.OTELEME_SINIRLAMA_ORANI);
  private final SlewRateLimiter zSinirlayici = new SlewRateLimiter(SurusKontrolSabitleri.DONUS_SINIRLAMA_ORANI);


  public SurusKomutu(DoubleSupplier yHizi, DoubleSupplier xHizi, DoubleSupplier zDonusu, SurusAltSistemi surusAltSistemi) {
    this(yHizi, xHizi, zDonusu, surusAltSistemi::drive, surusAltSistemi);
  }

  SurusKomutu(DoubleSupplier yHizi, DoubleSupplier xHizi, DoubleSupplier zDonusu, SurusCikisi surusCikisi, Subsystem... gereksinimler) {
    this.surusCikisi = surusCikisi;
    this.yHizi = yHizi;
    this.xHizi = xHizi;
    this.zDonusu = zDonusu;

    if (gereksinimler != null && gereksinimler.length > 0) {
      addRequirements(gereksinimler);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Bu kurulumda ileri yon icin Y ekseni ters cevriliyor.
    double y = eksenSekillendir(-yHizi.getAsDouble(), SurusKontrolSabitleri.OTELEME_OLCEGI);
    double x = eksenSekillendir(xHizi.getAsDouble(), SurusKontrolSabitleri.YANLAMASINA_OLCEK);
    double z = eksenSekillendir(zDonusu.getAsDouble(), SurusKontrolSabitleri.DONUS_OLCEGI);

    double yKomutu = ySinirlayici.calculate(y);
    double xKomutu = xSinirlayici.calculate(x);
    double zKomutu = zSinirlayici.calculate(z);

    surusCikisi.sur(yKomutu, xKomutu, zKomutu);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  static double eksenSekillendir(double hamGirdi, double olcek) {
    double oluBolgeli = MathUtil.applyDeadband(hamGirdi, SurusKontrolSabitleri.OLU_BOLGE);
    return Math.copySign(oluBolgeli * oluBolgeli, oluBolgeli) * olcek;
  }
}
