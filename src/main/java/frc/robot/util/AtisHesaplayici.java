package frc.robot.util;

import frc.robot.Sabitler.ModulSabitleri;

/**
 * Mesafeye gore atici RPM hesaplar.
 * Degerler saha testinde kalibre edilmelidir.
 */
public final class AtisHesaplayici {
    private AtisHesaplayici() {}

    public static double hesaplaHedefRpm(double mesafeMetre) {
        double[] mesafeler = ModulSabitleri.ATIS_MESAFE_TABLOSU_METRE;
        double[] rpmler = ModulSabitleri.ATIS_RPM_TABLOSU;
        if (mesafeler.length == 0 || mesafeler.length != rpmler.length) {
            return ModulSabitleri.ATICI_HEDEF_RPM;
        }

        if (mesafeMetre <= mesafeler[0]) {
            return rpmler[0];
        }

        int son = mesafeler.length - 1;
        if (mesafeMetre >= mesafeler[son]) {
            return rpmler[son];
        }

        for (int i = 0; i < son; i++) {
            double x0 = mesafeler[i];
            double x1 = mesafeler[i + 1];
            if (mesafeMetre >= x0 && mesafeMetre <= x1) {
                double y0 = rpmler[i];
                double y1 = rpmler[i + 1];
                double oran = (mesafeMetre - x0) / (x1 - x0);
                return y0 + (y1 - y0) * oran;
            }
        }

        return ModulSabitleri.ATICI_HEDEF_RPM;
    }
}
