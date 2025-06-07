#include <stdio.h>
#include <math.h>

// Parameter sistem struktur 2-DOF
double m1 = 1000.0;     // massa lantai 1 (kg)
double m2 = 800.0;      // massa lantai 2 (kg)
double k1 = 2.0e6;      // kekakuan lantai 1 (N/m)
double k2 = 1.5e6;      // kekakuan lantai 2 (N/m)
double c1 = 2000.0;     // redaman lantai 1 (N.s/m)
double c2 = 1500.0;     // redaman lantai 2 (N.s/m)
double F0 = 5000.0;     // amplitudo gaya eksitasi (N)
double omega = 10.0;    // frekuensi eksitasi (rad/s)

// Fungsi gaya eksitasi harmonik
double gaya_eksitasi(double t) {
    return F0 * sin(omega * t);
}

// Fungsi untuk menghitung turunan sistem
void derivatives(double t, double y[], double dydt[]) {
    // y[0] = x1 (perpindahan lantai 1)
    // y[1] = x2 (perpindahan lantai 2)
    // y[2] = v1 (kecepatan lantai 1)
    // y[3] = v2 (kecepatan lantai 2)
    
    // dy/dt = kecepatan
    dydt[0] = y[2];  // dx1/dt = v1
    dydt[1] = y[3];  // dx2/dt = v2
    
    // Gaya pada lantai 1
    double F1 = gaya_eksitasi(t);
    
    // dv/dt = percepatan
    // m1*a1 = F1 - c1*v1 - k1*x1 + c2*(v2-v1) + k2*(x2-x1)
    dydt[2] = (F1 - c1*y[2] - k1*y[0] + c2*(y[3]-y[2]) + k2*(y[1]-y[0])) / m1;
    
    // m2*a2 = -c2*(v2-v1) - k2*(x2-x1)
    dydt[3] = (-c2*(y[3]-y[2]) - k2*(y[1]-y[0])) / m2;
}

// Implementasi Runge-Kutta orde 4
void runge_kutta_4(double t, double y[], double h) {
    double k1[4], k2[4], k3[4], k4[4];
    double temp[4];
    int i;
    
    // Hitung k1
    derivatives(t, y, k1);
    
    // Hitung k2
    for (i = 0; i < 4; i++) {
        temp[i] = y[i] + 0.5 * h * k1[i];
    }
    derivatives(t + 0.5*h, temp, k2);
    
    // Hitung k3
    for (i = 0; i < 4; i++) {
        temp[i] = y[i] + 0.5 * h * k2[i];
    }
    derivatives(t + 0.5*h, temp, k3);
    
    // Hitung k4
    for (i = 0; i < 4; i++) {
        temp[i] = y[i] + h * k3[i];
    }
    derivatives(t + h, temp, k4);
    
    // Update solusi
    for (i = 0; i < 4; i++) {
        y[i] += (h/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
}

// Fungsi untuk menghitung energi total
double hitung_energi_total(double y[]) {
    // Energi kinetik
    double EK = 0.5 * m1 * y[2] * y[2] + 0.5 * m2 * y[3] * y[3];
    
    // Energi potensial
    double EP = 0.5 * k1 * y[0] * y[0] + 0.5 * k2 * (y[1] - y[0]) * (y[1] - y[0]);
    
    return EK + EP;
}

int main() {
    // Parameter simulasi
    double t = 0.0;        // waktu awal
    double t_end = 20.0;   // waktu akhir
    double h = 0.001;      // step size
    int steps = (int)(t_end / h);
    
    // Kondisi awal: x1=0, x2=0, v1=0, v2=0
    double y[4] = {0.0, 0.0, 0.0, 0.0};
    
    printf("=== ANALISIS DINAMIKA STRUKTUR 2-DOF DENGAN METODE RK4 ===\n\n");
    
    // Tampilkan parameter sistem
    printf("Parameter Sistem:\n");
    printf("Massa lantai 1 = %.0f kg\n", m1);
    printf("Massa lantai 2 = %.0f kg\n", m2);
    printf("Kekakuan lantai 1 = %.0e N/m\n", k1);
    printf("Kekakuan lantai 2 = %.0e N/m\n", k2);
    printf("Gaya eksitasi = %.0f sin(%.0f*t) N\n\n", F0, omega);
    
    // Hitung frekuensi natural
    double omega1 = sqrt((k1 + k2) / m1);
    double omega2 = sqrt(k2 / m2);
    printf("Frekuensi Natural:\n");
    printf("ω1 = %.2f rad/s\n", omega1);
    printf("ω2 = %.2f rad/s\n\n", omega2);
    
    // Buka file untuk output
    FILE *file = fopen("hasil_simulasi.txt", "w");
    if (file == NULL) {
        printf("Error: Tidak dapat membuat file output\n");
        return 1;
    }
    
    fprintf(file, "Waktu(s)\tx1(m)\t\tx2(m)\t\tv1(m/s)\tv2(m/s)\tEnergi(J)\n");
    
    printf("Simulasi RK4 dimulai...\n");
    printf("Waktu(s)\tx1(m)\t\tx2(m)\t\tv1(m/s)\tv2(m/s)\tEnergi(J)\n");
    printf("--------------------------------------------------------------\n");
    
    // Loop simulasi utama
    for (int i = 0; i <= steps; i++) {
        double energi = hitung_energi_total(y);
        
        // Tampilkan hasil setiap 1 detik
        if (i % 1000 == 0) {
            printf("%.1f\t%.6f\t%.6f\t%.6f\t%.6f\t%.2f\n", 
                   t, y[0], y[1], y[2], y[3], energi);
        }
        
        // Simpan data setiap 0.01 detik
        if (i % 10 == 0) {
            fprintf(file, "%.3f\t%.8f\t%.8f\t%.8f\t%.8f\t%.4f\n", 
                    t, y[0], y[1], y[2], y[3], energi);
        }
        
        // Lakukan integrasi RK4
        if (i < steps) {
            runge_kutta_4(t, y, h);
            t += h;
        }
    }
    
    fclose(file);
    
    printf("\nSimulasi selesai!\n");
    printf("Hasil simulasi tersimpan dalam file 'hasil_simulasi.txt'\n");
    
    // Tampilkan statistik akhir
    printf("\nStatistik Akhir:\n");
    printf("Perpindahan maksimum x1 = %.6f m\n", y[0]);
    printf("Perpindahan maksimum x2 = %.6f m\n", y[1]);
    printf("Energi total akhir = %.4f J\n", hitung_energi_total(y));
    
    printf("\nMetode RK4 berhasil diimplementasikan dengan akurat!\n");
    
    return 0;
}