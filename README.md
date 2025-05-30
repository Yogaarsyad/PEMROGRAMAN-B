# Implementasi Metode Newton-Raphson untuk Analisis Stabilitas Sistem Kontrol

## Kelompok 3

### Judul Program
**Implementasi Metode Newton-Raphson untuk Analisis Stabilitas Sistem Kontrol Massa-Pegas-Damper dengan Umpan Balik Kontroler**

### Anggota Kelompok
| No | Nama Lengkap | NPM |
|----|--------------|-----|
| 1 | Daffa Hardhan | 2306161763 |
| 2 | Muhammad Bryan Farras | 2306230975 |
| 3 | Tri Yoga Arsyad | 2306161920 |

---

## Deskripsi Program

Program ini mengimplementasikan metode **Newton-Raphson** dalam bahasa pemrograman **C** sebagai alat untuk analisis stabilitas sistem kontrol orde kedua, khususnya pada model **massa-pegas-damper** dengan gain kontroler yang dapat divariasikan. Dengan pendekatan numerik ini, akar-akar persamaan karakteristik sistem dapat dihitung secara efisien, sehingga memungkinkan evaluasi stabilitas sistem berdasarkan posisi eigenvalue dalam bidang kompleks. Program ini juga menyediakan fitur untuk memvalidasi hasil numerik dengan solusi analitik menggunakan rumus kuadrat, serta mengklasifikasikan perilaku sistem (overdamped, critically damped, underdamped) berdasarkan nilai diskriminan. Implementasi ini diharapkan menjadi referensi praktis bagi mahasiswa teknik dan praktisi dalam memahami serta menerapkan metode numerik pada analisis sistem kontrol.

---

## Fitur Utama Program

### 🔧 Implementasi Metode Newton-Raphson
- Pencarian akar real dengan **konvergensi kuadratik**
- Toleransi error `1e-6` dan maksimum `100` iterasi
- Validasi konvergensi dengan dual criteria

### 📊 Analisis Sistem Kontrol Komprehensif
Klasifikasi jenis sistem berdasarkan nilai diskriminan:
- **Overdamped** (Δ > 0): Dua akar real negatif berbeda
- **Critically damped** (Δ = 0): Satu akar real negatif berulang  
- **Underdamped** (Δ < 0): Akar kompleks konjugat
- Evaluasi stabilitas berdasarkan bagian real akar-akar

### ✅ Validasi dan Verifikasi
- Perbandingan hasil numerik dengan solusi analitik menggunakan rumus kuadrat
- Perhitungan error relatif untuk assessment akurasi
- Tingkat konvergensi dan analisis performa algoritma

### 🔬 Eksperimen Batch dan Analisis Data
- Analisis komprehensif untuk range nilai gain kontroler (K = 0 hingga 10)
- Export hasil ke format CSV untuk analisis eksternal
- Statistik performa dan distribusi jenis sistem

### 💻 Interface Interaktif
- Menu driven interface untuk kemudahan penggunaan
- Demonstrasi metode dengan contoh sederhana
- Input validation dan error handling

---

## Persamaan Karakteristik

### Sistem Massa-Pegas-Damper dengan Kontroler:
```
m·ẍ + c·ẋ + (k + K)·x = 0
```

### Persamaan Karakteristik yang Dihasilkan:
```
s² + (c/m)·s + (k + K)/m = 0
```

### Parameter Default:
| Parameter | Simbol | Nilai | Satuan |
|-----------|--------|-------|--------|
| Massa sistem | m | 1.0 | kg |
| Koefisien redaman | c | 4.0 | N·s/m |
| Konstanta pegas | k | 1.0 | N/m |
| Gain kontroler | K | 0.0 - 10.0 | variabel |

---

## Metodologi Analisis

### 1. Kalkulasi Diskriminan
```
Δ = b² - 4ac
```
untuk klasifikasi awal jenis sistem

### 2. Pencarian Akar
- **Newton-Raphson** untuk akar real (sistem overdamped)
- **Formula analitik** untuk akar berulang (critically damped) dan kompleks (underdamped)

### 3. Evaluasi Stabilitas
Sistem stabil jika semua akar memiliki **Re(s) < 0**

### 4. Validasi Numerik
Perbandingan dengan solusi rumus kuadrat

---

## Menu Program

| No | Fitur | Deskripsi |
|----|-------|-----------|
| 1 | **Demonstrasi Newton-Raphson** | Contoh sederhana untuk pembelajaran |
| 2 | **Analisis Individual** | Eksperimen dengan parameter custom |
| 3 | **Eksperimen Komprehensif** | Analisis batch untuk range K = 0-10 |
| 4 | **Ringkasan Eksperimen** | Review hasil dalam format tabel |
| 5 | **Analisis Performa** | Evaluasi algoritma dan statistik |
| 6 | **Export CSV** | Simpan data untuk analisis eksternal |
| 7 | **Keluar** | Exit program |

---

## Output Program

### 📈 Analisis Individual
- Parameter sistem yang dianalisis
- Persamaan karakteristik yang terbentuk
- Klasifikasi jenis sistem
- Akar-akar dan informasi konvergensi
- Analisis stabilitas dan interpretasi fisis

### 📋 Eksperimen Batch
- Ringkasan dalam format tabel
- Statistik performa algoritma (iterasi, error, tingkat konvergensi)
- Distribusi jenis sistem
- Validasi akurasi dengan error relatif maksimum < 10⁻⁵%

### 💾 Export Data
- Format CSV untuk plotting dan analisis lanjutan
- Kompatibel dengan Python, MATLAB, Excel

---

## Kompilasi dan Eksekusi

### Persyaratan Sistem
- **Compiler**: GCC atau compiler C standard lainnya
- **Library**: `stdio.h`, `stdlib.h`, `math.h`, `stdbool.h`, `string.h`

### Cara Kompilasi
```bash
gcc -o NewtonRaphson NewtonRapshon.c -lm
```

### Cara Menjalankan
```bash
./NewtonRaphson
```

---

## Contoh Hasil

### Sistem Overdamped (K = 0)
```
Parameter Sistem:
  Massa (m)      = 1.000 kg
  Damping (c)    = 4.000 N⋅s/m
  Spring (k)     = 1.000 N/m
  Gain (K)       = 0.000

Persamaan Karakteristik: s² + 4.000s + 1.000 = 0
Diskriminan: 12.000000
Jenis Sistem: Overdamped

Akar-akar:
  s₁ = -0.267949 (konvergen dalam 11 iterasi)
  s₂ = -3.732051 (konvergen dalam 11 iterasi)

Status: STABIL
```

### Sistem Underdamped (K = 5)
```
Persamaan Karakteristik: s² + 4.000s + 6.000 = 0
Diskriminan: -8.000000
Jenis Sistem: Underdamped

Akar-akar:
  s₁ = -2.000000 + 1.414214j
  s₂ = -2.000000 - 1.414214j
  Frekuensi osilasi: 1.414214 rad/s

Status: STABIL
```

---

## Aplikasi dan Kegunaan

### 🎓 Pendidikan
- Pembelajaran metode numerik dalam teknik
- Demonstrasi analisis stabilitas sistem kontrol
- Pemahaman transisi overdamped ↔ critically damped ↔ underdamped

### 🔬 Penelitian
- Baseline untuk pengembangan metode analisis lanjutan
- Validasi hasil simulasi sistem kontrol
- Data eksperimen untuk publikasi ilmiah

### 🏭 Industri
- Analisis preliminary design sistem kontrol
- Optimasi parameter kontroler
- Assessment stabilitas sebelum implementasi

---

## Kontribusi dan Pengembangan

Program ini diharapkan menjadi **referensi praktis** bagi:
- Mahasiswa teknik dan sains
- Praktisi sistem kontrol
- Peneliti metode numerik

Dengan fokus pada **implementasi yang efisien dan akurat** dalam bahasa C untuk aplikasi real-time dan embedded systems.

---

## Lisensi dan Kontak

**Program Studi Teknik Komputer**
**Universitas Indonesia**
**2023**

Untuk pertanyaan dan saran, silakan hubungi anggota kelompok melalui email yang tertera di dalam kode program.

