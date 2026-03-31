// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sabitler.SurusKontrolSabitleri;
import frc.robot.Subsystems.SurusAltSistemi;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SurusKomutu extends Command {
  @FunctionalInterface
  public interface SurusCikisi {
    void sur(double xHizi, double yHizi, double zDonus);
  }

  private final SurusCikisi surusCikisi;
  private final DoubleSupplier xHizi;
  private final DoubleSupplier yHizi;
  private final DoubleSupplier zDonusu;


  public SurusKomutu(DoubleSupplier xHizi, DoubleSupplier yHizi, DoubleSupplier zDonusu, SurusAltSistemi surusAltSistemi) {
    this(xHizi, yHizi, zDonusu, surusAltSistemi::drive, surusAltSistemi);
  }

  SurusKomutu(DoubleSupplier xHizi, DoubleSupplier yHizi, DoubleSupplier zDonusu, SurusCikisi surusCikisi, Subsystem... gereksinimler) {
    this.surusCikisi = surusCikisi;
    this.xHizi = xHizi;
    this.yHizi = yHizi;
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
    // WPILib NWU conventionu: xSpeed=ileri(+), ySpeed=sol(+)
    double hamX = yHizi.getAsDouble();   // Axis 1 (ileri/geri)
    double hamY = xHizi.getAsDouble();   // Axis 0 (yanal)
    double hamZ = zDonusu.getAsDouble(); // Axis 2 (donus)

    double x = eksenSekillendir(-hamX, SurusKontrolSabitleri.OTELEME_OLCEGI);
    // PS4 left X sag itildiginde pozitif gelir; +hamY ile sag hareket saglanir.
    double y = eksenSekillendir(hamY, SurusKontrolSabitleri.YANLAMASINA_OLCEK);
    double z = eksenSekillendir(hamZ, SurusKontrolSabitleri.DONUS_OLCEGI);

    surusCikisi.sur(x, y, z);

    // Ham eksen degerleri (joystick'ten gelen ham deger)
    SmartDashboard.putNumber("Surus/HamEksen_Ileri", hamX);
    SmartDashboard.putNumber("Surus/HamEksen_Yanal", hamY);
    SmartDashboard.putNumber("Surus/HamEksen_Donus", hamZ);
    // Olubolgeli + kare yanit + olcekli degerler
    SmartDashboard.putNumber("Surus/SekillendX", x);
    SmartDashboard.putNumber("Surus/SekillendY", y);
    SmartDashboard.putNumber("Surus/SekillendZ", z);
    // Nihai komutlar (lineer + deadband + olcek)
    SmartDashboard.putNumber("Surus/KomutX", x);
    SmartDashboard.putNumber("Surus/KomutY", y);
    SmartDashboard.putNumber("Surus/KomutZ", z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Komut bitince fren yap - gecikme olmamali
    surusCikisi.sur(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  static double eksenSekillendir(double hamGirdi, double olcek) {
    double oluBolgeli = MathUtil.applyDeadband(hamGirdi, SurusKontrolSabitleri.OLU_BOLGE);
    return oluBolgeli * olcek;
  }
}
