# Implementasi Metode Runge-Kutta Orde Keempat untuk Analisis Dinamika Struktur 2-DOF


- **Nama Lengkap**: Audy Natalie Cecilia Rumahorbo
- **NPM**: 2306266962

## Deskripsi Program

Program ini mengimplementasikan **metode Runge-Kutta orde keempat (RK4)** untuk menyelesaikan persamaan diferensial dalam analisis dinamika struktur. Fokus utama adalah aplikasi RK4 pada sistem struktur **2 derajat kebebasan (2-DOF)** dengan eksitasi harmonik menggunakan bahasa pemrograman C.

### Latar Belakang
Analisis dinamika struktur merupakan aspek krusial dalam bidang teknik sipil, khususnya untuk perancangan struktur yang mampu menahan beban dinamis seperti gempa bumi, angin, dan getaran mesin. Persamaan gerak struktur umumnya berupa sistem persamaan diferensial biasa (ODE) orde kedua yang nonlinear dan sulit diselesaikan secara analitik.

Metode RK4 memiliki keunggulan signifikan dengan error lokal berorde O(h⁵) yang memberikan akurasi tinggi dengan ukuran langkah yang relatif besar, membuatnya sangat cocok untuk analisis dinamika struktur yang memerlukan simulasi jangka panjang.

## Fitur Program

### 1. Model Struktur 2-DOF
- Sistem struktur bertingkat dengan 2 lantai
- Model massa-pegas-peredam untuk setiap lantai
- Interaksi coupling antara lantai

### 2. Parameter Sistem
| Parameter | Lantai 1 | Lantai 2 | Satuan |
|-----------|----------|----------|---------|
| Massa (m) | 1000 | 800 | kg |
| Kekakuan (k) | 2.0×10⁶ | 1.5×10⁶ | N/m |
| Redaman (c) | 2000 | 1500 | N·s/m |

### 3. Eksitasi Harmonik
- **Gaya eksitasi**: F(t) = 5000 sin(10t) N
- **Aplikasi**: Pada lantai pertama
- **Frekuensi**: 10 rad/s (1.59 Hz)

### 4. Metode Numerik RK4
- Implementasi algoritma Runge-Kutta orde keempat
- Transformasi sistem orde kedua ke orde pertama
- Akurasi tinggi dengan error numerik ~10⁻⁷ - 10⁻⁸ m
- Stabilitas numerik untuk simulasi jangka panjang

## Struktur Program

### Fungsi Utama
1. `gaya_eksitasi(double t)` - Menghitung gaya harmonik
2. `derivatives(double t, double y[], double dydt[])` - Sistem persamaan diferensial
3. `runge_kutta_4(double t, double y[], double h)` - Implementasi algoritma RK4
4. `hitung_energi_total(double y[])` - Kalkulasi energi kinetik dan potensial

### Transformasi Sistem
Sistem orde kedua ditransformasi menjadi sistem orde pertama:
- `y[0] = x₁` (perpindahan lantai 1)
- `y[1] = x₂` (perpindahan lantai 2)  
- `y[2] = v₁` (kecepatan lantai 1)
- `y[3] = v₂` (kecepatan lantai 2)

### Implementasi Metode Runge-Kutta
Metode Runge-Kutta orde keempat diimplementasikan dengan formulasi matematis:

**Persamaan Dasar RK4:**
```
yn+1 = yn + (h/6)(k1 + 2k2 + 2k3 + k4)
```

**Dimana:**
- k1 = f(tn, yn)
- k2 = f(tn + h/2, yn + hk1/2)
- k3 = f(tn + h/2, yn + hk2/2)  
- k4 = f(tn + h, yn + hk3)

**Implementasi dalam Kode C:**
```c
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
    
    // Update solusi dengan rata-rata berbobot
    for (i = 0; i < 4; i++) {
        y[i] += (h/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
}
```

**Penjelasan Algoritma:**
1. **Langkah k1**: Menghitung slope pada titik awal interval
2. **Langkah k2**: Menghitung slope pada titik tengah pertama menggunakan k1
3. **Langkah k3**: Menghitung slope pada titik tengah kedua menggunakan k2
4. **Langkah k4**: Menghitung slope pada titik akhir interval menggunakan k3
5. **Update**: Menggabungkan semua slope dengan pembobotan Simpson
   
### Persamaan Gerak
```
m₁ẍ₁ + (c₁ + c₂)ẋ₁ - c₂ẋ₂ + (k₁ + k₂)x₁ - k₂x₂ = F(t)
m₂ẍ₂ - c₂ẋ₁ + c₂ẋ₂ - k₂x₁ + k₂x₂ = 0
```

### Contoh Output Console
```
=== ANALISIS DINAMIKA STRUKTUR 2-DOF DENGAN METODE RK4 ===

Parameter Sistem:
Massa lantai 1 = 1000 kg
Massa lantai 2 = 800 kg
Kekakuan lantai 1 = 2e+06 N/m
Kekakuan lantai 2 = 2e+06 N/m
Gaya eksitasi = 5000 sin(10*t) N

Frekuensi Natural:
ω1 = 43.30 rad/s
ω2 = 59.16 rad/s

Waktu(s)	x1(m)		x2(m)		v1(m/s)	v2(m/s)	Energi(J)
0.0		0.000000	0.000000	0.000000	0.000000	0.00
2.0		0.004523	0.003781	-0.032145	-0.025067	80.00
...
```

## Hasil dan Validasi

### Karakteristik Respons
- **Amplitudo maksimum**: ~0.006 m (lantai 1), ~0.005 m (lantai 2)
- **Pola osilasi**: Harmonik dengan frekuensi eksitasi
- **Coupling effect**: Interaksi signifikan antar lantai
- **Regime operasi**: Sub-resonan (fₑₓ < f₁)

### Akurasi Numerik
- **Error absolut**: 10⁻⁷ - 10⁻⁸ m terhadap solusi analitik
- **Konservasi energi**: Fluktuasi ±2% dari nilai rata-rata
- **Stabilitas**: Tidak ada drift atau pertumbuhan artifisial

### Validasi
- Perbandingan dengan solusi analitik menunjukkan korelasi tinggi
- Verifikasi konservasi energi dalam toleransi yang dapat diterima
- Konsistensi hubungan fase antara perpindahan dan kecepatan

## Keunggulan Metode RK4

1. **Akurasi Tinggi**: Error numerik dalam orde 10⁻⁷ hingga 10⁻⁸
2. **Stabilitas Numerik**: Stabil untuk simulasi jangka panjang
3. **Fleksibilitas Step Size**: Dapat disesuaikan kebutuhan akurasi
4. **Implementasi Sederhana**: Algoritma mudah dipahami dan dimodifikasi
5. **Konservasi Energi**: Mempertahankan sifat fisik sistem


## Cara Menggunakan

**Persyaratan:**

* Compiler C (gcc, clang, atau compiler C lainnya)
* Sistem operasi: Linux, Windows, atau macOS
* Library matematika standar C (math.h)

---

### **Kompilasi dan Menjalankan**

1. **Clone repositori ini**

   Di halaman GitHub proyek, klik tombol **Code** yang terletak di kanan atas, lalu pilih metode yang diinginkan untuk menyalin URL. Anda dapat memilih antara HTTPS, SSH, atau GitHub CLI. Gunakan URL yang ditampilkan untuk meng-clone repositori. Untuk HTTPS, URL-nya adalah:

   ```bash
   git clone https://github.com/audynatalie/ProyekUASKomnum.git
   cd ProyekUASKomnum
   ```

2. **Kompilasi program**

   Setelah berada di dalam folder repositori, kompilasi program dengan perintah berikut:

   ```bash
   gcc -o MetodeRunge-Kutta MetodeRunge-Kutta.c -lm
   ```

   Perintah ini akan menghasilkan file eksekusi yang bernama `MetodeRunge-Kutta`.

3. **Jalankan program**

   Untuk menjalankan program, gunakan perintah berikut:

   ```bash
   ./MetodeRunge-Kutta
   ```

---
