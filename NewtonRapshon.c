/*
 * ============================================================================
 * TUGAS PEMROGRAMAN B - KOMPUTASI NUMERIK
 * Implementasi Metode Newton-Raphson untuk Analisis Stabilitas Sistem Kontrol
 *
 * DESKRIPSI PROGRAM:
 * Program ini mengimplementasikan metode Newton-Raphson untuk menganalisis
 * stabilitas sistem kontrol massa-pegas-damper dengan umpan balik kontroler.
 * Program dapat menentukan jenis sistem (overdamped, critically damped,
 * underdamped) dan menganalisis stabilitas berdasarkan lokasi akar-akar
 * persamaan karakteristik.
 *
 * FITUR UTAMA:
 * 1. Implementasi metode Newton-Raphson untuk pencarian akar real
 * 2. Analisis sistem kontrol individual dan komprehensif
 * 3. Klasifikasi jenis sistem berdasarkan diskriminan
 * 4. Validasi hasil numerik dengan solusi analitik
 * 5. Export hasil ke file CSV untuk analisis lanjutan
 * 6. Analisis performa algoritma konvergensi
 *
 * Kelompok 3:
 * 1. Daffa Hardhan
 * 2. Muhammad Bryan Farras
 * 3. Tri Yoga Arsyad
 *
 * Program Studi Teknik Komputer
 * Universitas Indonesia
 * 2023
 * ============================================================================
 */

#include <stdio.h>    // Input/output standard
#include <stdlib.h>   // Fungsi utilitas umum (malloc, exit, dll)
#include <math.h>     // Fungsi matematika (sqrt, fabs, dll)
#include <stdbool.h>  // Tipe data boolean
#include <string.h>   // Manipulasi string

// ============================================================================
// KONSTANTA DAN DEFINISI
// ============================================================================

// Konstanta untuk algoritma Newton-Raphson
#define MAX_ITERATIONS 100          // Batas maksimum iterasi untuk mencegah infinite loop
#define TOLERANCE 1e-6              // Toleransi konvergensi (presisi 6 digit desimal)

// Konstanta untuk manajemen data
#define MAX_SYSTEMS 50              // Kapasitas maksimum sistem untuk eksperimen komprehensif
#define MAX_FILENAME 256            // Panjang maksimum nama file untuk keamanan buffer

// ============================================================================
// STRUKTUR DATA
// ============================================================================

/**
 * Struktur untuk menyimpan bilangan kompleks
 * Digunakan untuk merepresentasikan akar kompleks pada sistem underdamped
 */
typedef struct {
    double real;    // Bagian real dari bilangan kompleks
    double imag;    // Bagian imajiner dari bilangan kompleks
} Complex;

/**
 * Struktur untuk parameter sistem kontrol massa-pegas-damper
 * Merepresentasikan persamaan diferensial: m*x'' + c*x' + (k+K)*x = 0
 */
typedef struct {
    double massa;       // Massa (m) dalam kg - inersia sistem
    double damping;     // Koefisien redaman (c) dalam N⋅s/m - resistansi gerakan
    double spring;      // Konstanta pegas (k) dalam N/m - kekakuan sistem
    double gain;        // Gain kontroler (K) - parameter kontrol yang dapat diatur
} SystemParameters;

/**
 * Struktur untuk menyimpan hasil lengkap analisis Newton-Raphson
 * Berisi semua informasi yang diperlukan untuk evaluasi sistem
 */
typedef struct {
    Complex root1;          // Akar pertama (dapat real atau kompleks)
    Complex root2;          // Akar kedua (konjugat dari root1 jika kompleks)
    int iterations1;        // Jumlah iterasi Newton-Raphson untuk akar pertama
    int iterations2;        // Jumlah iterasi Newton-Raphson untuk akar kedua
    double error1;          // Error konvergensi akhir untuk akar pertama
    double error2;          // Error konvergensi akhir untuk akar kedua
    bool converged1;        // Status konvergensi akar pertama (true/false)
    bool converged2;        // Status konvergensi akar kedua (true/false)
    int system_type;        // Klasifikasi sistem: 0=overdamped, 1=critical, 2=underdamped
    bool is_stable;         // Status stabilitas sistem (true jika semua akar Re(s)<0)
    double discriminant;    // Nilai diskriminan untuk klasifikasi jenis akar
} AnalysisResult;

/**
 * Struktur untuk konfigurasi solver Newton-Raphson
 * Encapsulates parameter dan konfigurasi untuk proses iterasi
 */
typedef struct {
    double tolerance;       // Toleransi konvergensi yang digunakan
    int max_iterations;     // Maksimum iterasi yang diizinkan
    SystemParameters params; // Parameter sistem yang sedang dianalisis
} NewtonRaphsonSolver;

// ============================================================================
// VARIABEL GLOBAL
// ============================================================================

// Array untuk menyimpan hasil eksperimen komprehensif
// Memungkinkan analisis batch dan perbandingan multiple sistem
AnalysisResult experiment_results[MAX_SYSTEMS];
int num_experiments = 0;  // Counter jumlah eksperimen yang telah dilakukan

// ============================================================================
// DEKLARASI FUNGSI
// ============================================================================

// Fungsi untuk interface dan presentasi
void print_header(void);                    // Menampilkan header program dengan info kelompok
void print_separator(char character, int length);  // Utility untuk mencetak garis pemisah

// Fungsi matematika inti
double characteristic_equation(double s, SystemParameters params);      // f(s) = s² + (c/m)s + (k+K)/m
double characteristic_derivative(double s, SystemParameters params);    // f'(s) = 2s + (c/m)
double calculate_discriminant(SystemParameters params);                 // Δ = b² - 4ac

// Fungsi algoritma Newton-Raphson
double newton_raphson_real(NewtonRaphsonSolver* solver, double initial_guess, int* iterations, double* final_error, bool* converged);

// Fungsi analisis sistem
AnalysisResult analyze_control_system(SystemParameters params);         // Analisis lengkap sistem
void validate_with_analytical(SystemParameters params, AnalysisResult result);  // Validasi numerik vs analitik

// Fungsi output dan reporting
void print_analysis_result(SystemParameters params, AnalysisResult result);     // Display hasil individual
void print_experiment_summary(void);                                           // Ringkasan eksperimen batch
void save_results_to_csv(const char* filename);                               // Export ke CSV

// Fungsi demonstrasi dan eksperimen
void demonstrate_newton_raphson(void);       // Demo metode dengan contoh sederhana
void run_comprehensive_experiment(void);     // Eksperimen untuk range nilai K
void performance_analysis(void);             // Analisis performa algoritma

// ============================================================================
// IMPLEMENTASI FUNGSI
// ============================================================================

/**
 * Menampilkan header program dengan informasi kelompok
 * Fungsi utility untuk branding dan identifikasi program
 */
void print_header(void) {
    print_separator('=', 80);
    printf("TUGAS PEMROGRAMAN B - KOMPUTASI NUMERIK\n");
    printf("Implementasi Metode Newton-Raphson untuk Analisis Stabilitas Sistem Kontrol\n\n");
    printf("Kelompok 3:\n");
    printf("1. Daffa Hardhan\n");
    printf("2. Muhammad Bryan Farras\n");
    printf("3. Tri Yoga Arsyad\n\n");
    printf("Program Studi Teknik Komputer\n");
    printf("Universitas Indonesia\n");
    print_separator('=', 80);
    printf("\n");
}

/**
 * Fungsi persamaan karakteristik: f(s) = s² + (c/m)s + (k+K)/m
 * 
 * Ini adalah persamaan karakteristik dari sistem massa-pegas-damper:
 * m*x'' + c*x' + (k+K)*x = 0
 * Transformasi Laplace memberikan: m*s²*X(s) + c*s*X(s) + (k+K)*X(s) = 0
 * Persamaan karakteristik: s² + (c/m)*s + (k+K)/m = 0
 * 
 * @param s Nilai variabel s (variabel Laplace)
 * @param params Parameter sistem kontrol (m, c, k, K)
 * @return Nilai fungsi karakteristik pada titik s
 */
double characteristic_equation(double s, SystemParameters params) {
    double b = params.damping / params.massa;                    // Koefisien linear (c/m)
    double c = (params.spring + params.gain) / params.massa;     // Koefisien konstanta (k+K)/m
    
    return s * s + b * s + c;  // Persamaan kuadrat standar
}

/**
 * Fungsi turunan persamaan karakteristik: f'(s) = 2s + (c/m)
 * 
 * Turunan diperlukan untuk metode Newton-Raphson: x_{n+1} = x_n - f(x_n)/f'(x_n)
 * Untuk f(s) = s² + bs + c, maka f'(s) = 2s + b
 * 
 * @param s Nilai variabel s
 * @param params Parameter sistem kontrol
 * @return Nilai turunan fungsi karakteristik pada titik s
 */
double characteristic_derivative(double s, SystemParameters params) {
    double b = params.damping / params.massa;
    
    return 2.0 * s + b;  // Turunan persamaan kuadrat
}

/**
 * Implementasi metode Newton-Raphson untuk mencari akar real
 * 
 * ALGORITMA:
 * 1. Mulai dengan tebakan awal x₀
 * 2. Iterasi: x_{n+1} = x_n - f(x_n)/f'(x_n)
 * 3. Lanjutkan sampai |x_{n+1} - x_n| < toleransi atau |f(x_n)| < toleransi
 * 4. Atau sampai mencapai maksimum iterasi
 * 
 * KONVERGENSI:
 * - Kuadratik jika tebakan awal dekat dengan akar
 * - Dapat divergen jika f'(x) ≈ 0 atau tebakan awal buruk
 * 
 * @param solver Pointer ke struktur solver berisi konfigurasi
 * @param initial_guess Tebakan awal x₀
 * @param iterations Output: jumlah iterasi yang diperlukan
 * @param final_error Output: error konvergensi akhir
 * @param converged Output: status konvergensi (true/false)
 * @return Nilai akar yang ditemukan
 */
double newton_raphson_real(NewtonRaphsonSolver* solver, double initial_guess,
                          int* iterations, double* final_error, bool* converged) {
    
    double x = initial_guess;  // Nilai iterasi saat ini
    double fx, dfx, x_new;     // Variabel untuk f(x), f'(x), dan x_baru
    
    // Inisialisasi output parameters
    *converged = false;
    *iterations = 0;
    *final_error = 0.0;
    
    // Loop utama Newton-Raphson
    for (int i = 0; i < solver->max_iterations; i++) {
        // Evaluasi fungsi dan turunannya pada titik saat ini
        fx = characteristic_equation(x, solver->params);
        dfx = characteristic_derivative(x, solver->params);
        
        // Cek kondisi degenerate: turunan terlalu kecil (near-zero derivative)
        // Ini dapat menyebabkan pembagian dengan nol atau step size yang sangat besar
        if (fabs(dfx) < solver->tolerance) {
            printf("Peringatan: Turunan terlalu kecil pada iterasi %d\n", i);
            break;
        }
        
        // Hitung titik baru menggunakan formula Newton-Raphson
        x_new = x - fx / dfx;
        
        // Hitung error sebagai selisih absolut antara iterasi berturut-turut
        *final_error = fabs(x_new - x);
        
        // Cek kriteria konvergensi (dual criteria):
        // 1. Perubahan x kecil: |x_{n+1} - x_n| < tolerance
        // 2. Nilai fungsi kecil: |f(x_n)| < tolerance
        if (*final_error < solver->tolerance || fabs(fx) < solver->tolerance) {
            *iterations = i + 1;
            *converged = true;
            return x_new;
        }
        
        // Update untuk iterasi berikutnya
        x = x_new;
    }
    
    // Jika mencapai batas maksimum iterasi tanpa konvergensi
    *iterations = solver->max_iterations;
    return x;  // Return nilai terakhir meskipun tidak konvergen
}

/**
 * Menghitung diskriminan persamaan kuadrat ax² + bx + c = 0
 * 
 * DISKRIMINAN (Δ = b² - 4ac) menentukan sifat akar:
 * - Δ > 0: Dua akar real berbeda (overdamped)
 * - Δ = 0: Satu akar real berulang (critically damped)  
 * - Δ < 0: Dua akar kompleks konjugat (underdamped)
 * 
 * Untuk sistem kontrol, ini menentukan jenis respons transient
 * 
 * @param params Parameter sistem kontrol
 * @return Nilai diskriminan
 */
double calculate_discriminant(SystemParameters params) {
    double a = 1.0;  // Koefisien s² selalu 1 dalam normalisasi
    double b = params.damping / params.massa;
    double c = (params.spring + params.gain) / params.massa;
    
    return b * b - 4.0 * a * c;  // Formula diskriminan standar
}

/**
 * Fungsi utama untuk analisis lengkap sistem kontrol
 * 
 * LOGIKA ANALISIS:
 * 1. Hitung diskriminan untuk klasifikasi awal
 * 2. Berdasarkan diskriminan, tentukan metode pencarian akar:
 *    - Δ > 0: Gunakan Newton-Raphson untuk dua akar real
 *    - Δ = 0: Hitung langsung akar berulang
 *    - Δ < 0: Hitung langsung akar kompleks
 * 3. Evaluasi stabilitas berdasarkan bagian real akar
 * 
 * STABILITAS SISTEM:
 * Sistem stabil ⟺ semua akar memiliki Re(s) < 0
 * 
 * @param params Parameter sistem kontrol yang akan dianalisis
 * @return Struktur hasil analisis lengkap
 */
AnalysisResult analyze_control_system(SystemParameters params) {
    AnalysisResult result = {0}; // Zero-initialization untuk keamanan
    
    // Setup solver dengan konfigurasi default
    NewtonRaphsonSolver solver;
    solver.tolerance = TOLERANCE;
    solver.max_iterations = MAX_ITERATIONS;
    solver.params = params;
    
    // Hitung diskriminan untuk klasifikasi jenis sistem
    result.discriminant = calculate_discriminant(params);
    
    // Asumsi awal: sistem stabil (akan diubah jika ditemukan akar tidak stabil)
    result.is_stable = true;
    
    if (result.discriminant > TOLERANCE) {
        // ===== KASUS OVERDAMPED: Dua akar real berbeda =====
        result.system_type = 0;
        
        printf("Sistem Overdamped: Mencari dua akar real...\n");
        
        // Strategi pencarian akar: gunakan dua tebakan awal berbeda
        // Tebakan pertama: -0.1 (untuk akar yang mungkin dekat dengan nol)
        result.root1.real = newton_raphson_real(&solver, -0.1,
                                               &result.iterations1,
                                               &result.error1,
                                               &result.converged1);
        result.root1.imag = 0.0;  // Akar real memiliki bagian imajiner nol
        
        // Tebakan kedua: -10.0 (untuk akar yang lebih negatif)
        result.root2.real = newton_raphson_real(&solver, -10.0,
                                               &result.iterations2,
                                               &result.error2,
                                               &result.converged2);
        result.root2.imag = 0.0;
        
        // Evaluasi stabilitas: kedua akar harus negatif
        if (result.root1.real >= 0 || result.root2.real >= 0) {
            result.is_stable = false;
        }
        
    } else if (fabs(result.discriminant) <= TOLERANCE) {
        // ===== KASUS CRITICALLY DAMPED: Satu akar real berulang =====
        result.system_type = 1;
        
        printf("Sistem Critically Damped: Satu akar real berulang...\n");
        
        // Untuk akar berulang: s = -b/(2a)
        double b = params.damping / params.massa;
        result.root1.real = -b / 2.0;
        result.root1.imag = 0.0;
        result.root2 = result.root1; // Akar kedua identik dengan pertama
        
        // Set parameter iterasi (tidak menggunakan Newton-Raphson untuk kasus ini)
        result.iterations1 = 1;  // Simbolik: dihitung langsung
        result.iterations2 = 1;
        result.error1 = 0.0;     // Tidak ada error numerik
        result.error2 = 0.0;
        result.converged1 = true; // Selalu "konvergen" karena analitik
        result.converged2 = true;
        
        // Evaluasi stabilitas: akar harus negatif
        if (result.root1.real >= 0) {
            result.is_stable = false;
        }
        
    } else {
        // ===== KASUS UNDERDAMPED: Akar kompleks konjugat =====
        result.system_type = 2;
        
        printf("Sistem Underdamped: Akar kompleks konjugat...\n");
        
        // Hitung akar kompleks menggunakan rumus kuadrat
        double a = 1.0;
        double b = params.damping / params.massa;
        double c = (params.spring + params.gain) / params.massa;
        
        // Untuk Δ < 0: s = (-b ± j√|Δ|) / (2a)
        double real_part = -b / (2.0 * a);                    // Bagian real
        double imag_part = sqrt(-result.discriminant) / (2.0 * a);  // Bagian imajiner
        
        // Akar pertama: s₁ = σ + jω
        result.root1.real = real_part;
        result.root1.imag = imag_part;
        
        // Akar kedua: s₂ = σ - jω (konjugat kompleks)
        result.root2.real = real_part;
        result.root2.imag = -imag_part;
        
        // Set parameter iterasi (tidak menggunakan Newton-Raphson untuk akar kompleks)
        result.iterations1 = 1;
        result.iterations2 = 1;
        result.error1 = 0.0;
        result.error2 = 0.0;
        result.converged1 = true;
        result.converged2 = true;
        
        // Evaluasi stabilitas: bagian real harus negatif
        if (real_part >= 0) {
            result.is_stable = false;
        }
    }
    
    return result;
}

/**
 * Validasi hasil numerik dengan solusi analitik
 * 
 * TUJUAN:
 * - Memverifikasi akurasi implementasi Newton-Raphson
 * - Menghitung error relatif untuk assessment kualitas
 * - Memberikan confidence pada hasil numerik
 * 
 * METODE VALIDASI:
 * - Untuk akar real: gunakan rumus kuadrat sebagai referensi
 * - Untuk akar kompleks: bandingkan dengan formula analitik
 * - Hitung error relatif: |numerik - analitik| / |analitik| × 100%
 * 
 * @param params Parameter sistem yang dianalisis
 * @param result Hasil analisis numerik yang akan divalidasi
 */
void validate_with_analytical(SystemParameters params, AnalysisResult result) {
    printf("\n--- VALIDASI DENGAN SOLUSI ANALITIK ---\n");
    
    // Ekstrak koefisien persamaan kuadrat as² + bs + c = 0
    double a = 1.0;
    double b = params.damping / params.massa;
    double c = (params.spring + params.gain) / params.massa;
    
    printf("Persamaan kuadrat: s² + %.6fs + %.6f = 0\n", b, c);
    printf("Diskriminan = %.6f\n", result.discriminant);
    
    if (result.discriminant >= 0) {
        // ===== VALIDASI AKAR REAL =====
        double sqrt_disc = sqrt(result.discriminant);
        double analytical_root1 = (-b + sqrt_disc) / (2.0 * a);  // Akar pertama
        double analytical_root2 = (-b - sqrt_disc) / (2.0 * a);  // Akar kedua
        
        printf("Solusi analitik: s₁ = %.6f, s₂ = %.6f\n", 
               analytical_root1, analytical_root2);
        printf("Solusi numerik:  s₁ = %.6f, s₂ = %.6f\n", 
               result.root1.real, result.root2.real);
        
        // Hitung error relatif (dalam persen)
        double error1 = fabs(result.root1.real - analytical_root1) / fabs(analytical_root1) * 100.0;
        double error2 = fabs(result.root2.real - analytical_root2) / fabs(analytical_root2) * 100.0;
        
        printf("Error relatif:   %.6f%%, %.6f%%\n", error1, error2);
        
    } else {
        // ===== VALIDASI AKAR KOMPLEKS =====
        double real_part = -b / (2.0 * a);
        double imag_part = sqrt(-result.discriminant) / (2.0 * a);
        
        printf("Solusi analitik: s = %.6f ± %.6fj\n", real_part, imag_part);
        printf("Solusi numerik:  s = %.6f ± %.6fj\n", 
               result.root1.real, result.root1.imag);
        
        // Hitung error relatif untuk bagian real dan imajiner
        double error_real = fabs(result.root1.real - real_part) / fabs(real_part) * 100.0;
        double error_imag = fabs(result.root1.imag - imag_part) / fabs(imag_part) * 100.0;
        
        printf("Error relatif:   Real: %.6f%%, Imag: %.6f%%\n", error_real, error_imag);
    }
}

/**
 * Mencetak hasil analisis individual dalam format yang mudah dibaca
 * 
 * OUTPUT MELIPUTI:
 * - Parameter sistem input
 * - Persamaan karakteristik yang terbentuk
 * - Klasifikasi jenis sistem
 * - Akar-akar dan informasi konvergensi
 * - Analisis stabilitas dan interpretasi fisis
 * 
 * @param params Parameter sistem yang dianalisis
 * @param result Hasil analisis yang akan ditampilkan
 */
void print_analysis_result(SystemParameters params, AnalysisResult result) {
    print_separator('-', 60);
    printf("HASIL ANALISIS SISTEM KONTROL\n");
    print_separator('-', 60);
    
    // ===== TAMPILKAN PARAMETER SISTEM =====
    printf("Parameter Sistem:\n");
    printf("  Massa (m)      = %.3f kg\n", params.massa);
    printf("  Damping (c)    = %.3f N⋅s/m\n", params.damping);
    printf("  Spring (k)     = %.3f N/m\n", params.spring);
    printf("  Gain (K)       = %.3f\n", params.gain);
    
    // ===== TAMPILKAN PERSAMAAN KARAKTERISTIK =====
    double b = params.damping / params.massa;
    double c = (params.spring + params.gain) / params.massa;
    printf("\nPersamaan Karakteristik: s² + %.3fs + %.3f = 0\n", b, c);
    printf("Diskriminan: %.6f\n", result.discriminant);
    
    // ===== TAMPILKAN KLASIFIKASI SISTEM =====
    const char* system_types[] = {"Overdamped", "Critically Damped", "Underdamped"};
    printf("Jenis Sistem: %s\n", system_types[result.system_type]);
    
    // ===== TAMPILKAN AKAR-AKAR =====
    printf("\nAkar-akar:\n");
    if (result.system_type == 2) { // Underdamped: akar kompleks
        printf("  s₁ = %.6f + %.6fj\n", result.root1.real, result.root1.imag);
        printf("  s₂ = %.6f - %.6fj\n", result.root2.real, result.root2.imag);
        printf("  Frekuensi osilasi: %.6f rad/s\n", fabs(result.root1.imag));
    } else { // Overdamped atau Critical: akar real
        printf("  s₁ = %.6f", result.root1.real);
        if (result.converged1) {
            printf(" (konvergen dalam %d iterasi, error: %.2e)\n", 
                   result.iterations1, result.error1);
        } else {
            printf(" (tidak konvergen)\n");
        }
        
        if (result.system_type == 0) { // Overdamped: tampilkan akar kedua
            printf("  s₂ = %.6f", result.root2.real);
            if (result.converged2) {
                printf(" (konvergen dalam %d iterasi, error: %.2e)\n", 
                       result.iterations2, result.error2);
            } else {
                printf(" (tidak konvergen)\n");
            }
        }
    }
    
    // ===== ANALISIS STABILITAS =====
    printf("\nAnalisis Stabilitas:\n");
    printf("  Status: %s\n", result.is_stable ? "STABIL" : "TIDAK STABIL");
    
    if (result.is_stable) {
        printf("  Semua akar memiliki bagian real negatif\n");
        if (result.system_type == 2) {
            printf("  Sistem akan berosilasi dengan amplitude yang meredam\n");
        }
    } else {
        printf("  Terdapat akar dengan bagian real positif atau nol\n");
    }
}

/**
 * Mencetak ringkasan eksperimen komprehensif dalam format tabel
 * 
 * FUNGSI:
 * - Menampilkan hasil batch experiment dalam format tabel terstruktur
 * - Memberikan overview cepat untuk perbandingan sistem
 * - Menunjukkan trend perubahan dengan variasi parameter K
 * 
 * FORMAT OUTPUT:
 * - Kolom K: nilai gain kontroler
 * - Kolom Akar 1 & 2: nilai akar sistem
 * - Kolom Jenis: klasifikasi sistem
 * - Kolom Stabilitas: status stabilitas
 * - Kolom Iterasi: informasi konvergensi
 */
void print_experiment_summary(void) {
    if (num_experiments == 0) {
        printf("Tidak ada data eksperimen untuk ditampilkan.\n");
        return;
    }
    
    print_separator('=', 80);
    printf("RINGKASAN EKSPERIMEN KOMPREHENSIF\n");
    print_separator('=', 80);
    
    // Header tabel
    printf("%-5s %-12s %-12s %-15s %-10s %-8s\n", 
           "K", "Akar 1", "Akar 2", "Jenis", "Stabilitas", "Iterasi");
    print_separator('-', 80);
    
    // Data eksperimen
    for (int i = 0; i < num_experiments; i++) {
        AnalysisResult result = experiment_results[i];
        
        // Hitung gain K berdasarkan increment 1.0 (sesuai setup eksperimen)
        double K = i * 1.0;
        
        const char* types[] = {"Overdamped", "Critical", "Underdamped"};
        const char* stability = result.is_stable ? "Stabil" : "Tidak Stabil";
        
        if (result.system_type == 2) { // Underdamped: tampilkan sebagai kompleks
            printf("%-5.1f %-12s %-12s %-15s %-10s %-8s\n",
                   K, "Kompleks", "Kompleks", types[result.system_type], 
                   stability, "N/A");
        } else { // Real roots: tampilkan nilai numerik
            printf("%-5.1f %-12.6f %-12.6f %-15s %-10s %-8d\n",
                   K, result.root1.real, result.root2.real, 
                   types[result.system_type], stability, 
                   result.iterations1 + result.iterations2);
        }
    }
}

/**
 * Menyimpan hasil eksperimen ke file CSV untuk analisis eksternal
 * 
 * KEGUNAAN:
 * - Export data untuk plotting dengan tools eksternal (Python, MATLAB, Excel)
 * - Archival data untuk reproducibility
 * - Data sharing dan kolaborasi
 * 
 * FORMAT CSV:
 * - Header row dengan nama kolom
 * - Satu baris per eksperimen
 * - Semua data numerik dalam format standar
 * 
 * @param filename Nama file output (harus berekstensi .csv)
 */
void save_results_to_csv(const char* filename) {
    FILE* file = fopen(filename, "w");
    if (file == NULL) {
        printf("Error: Tidak dapat membuka file %s untuk ditulis.\n", filename);
        return;
    }
    
    // Tulis header CSV
    fprintf(file, "K,Root1_Real,Root1_Imag,Root2_Real,Root2_Imag,");
    fprintf(file, "System_Type,Is_Stable,Iterations1,Iterations2,");
    fprintf(file, "Error1,Error2,Discriminant\n");
    
    // Tulis data eksperimen
    for (int i = 0; i < num_experiments; i++) {
        AnalysisResult result = experiment_results[i];
        double K = i * 1.0; // Increment 1.0 sesuai setup eksperimen
        
        // Baris data dengan format yang konsisten
        fprintf(file, "%.1f,%.6f,%.6f,%.6f,%.6f,", 
                K, result.root1.real, result.root1.imag,
                result.root2.real, result.root2.imag);
        fprintf(file, "%d,%d,%d,%d,%.6e,%.6e,%.6f\n",
                result.system_type, result.is_stable ? 1 : 0,
                result.iterations1, result.iterations2,
                result.error1, result.error2, result.discriminant);
    }
    
    fclose(file);
    printf("Hasil eksperimen disimpan ke file: %s\n", filename);
}

/**
 * Demonstrasi metode Newton-Raphson dengan contoh sederhana
 * 
 * TUJUAN PEDAGOGIS:
 * - Menunjukkan cara kerja algoritma Newton-Raphson secara isolasi
 * - Memberikan pemahaman intuitive sebelum aplikasi kompleks
 * - Validasi implementasi dengan problem yang mudah diverifikasi
 * 
 * CONTOH YANG DIGUNAKAN:
 * f(x) = x² - 4 = 0 → akar analitik: x = ±2
 * - Sederhana namun non-trivial
 * - Akar yang mudah diverifikasi manual
 * - Menunjukkan sensitivitas terhadap tebakan awal
 */
void demonstrate_newton_raphson(void) {
    print_separator('=', 60);
    printf("DEMONSTRASI METODE NEWTON-RAPHSON\n");
    print_separator('=', 60);
    
    printf("Contoh: Mencari akar dari f(x) = x² - 4 = 0\n");
    printf("Akar analitik: x = ±2\n\n");
    
    // Setup sistem demonstrasi: x² - 4 = 0
    // Manipulasi parameter agar persamaan karakteristik menjadi s² - 4 = 0
    SystemParameters demo_params = {1.0, 0.0, -4.0, 0.0}; // m=1, c=0, k=-4, K=0
    
    NewtonRaphsonSolver solver;
    solver.tolerance = TOLERANCE;
    solver.max_iterations = MAX_ITERATIONS;
    solver.params = demo_params;
    
    // Variabel untuk menyimpan hasil iterasi
    int iterations;
    double error;
    bool converged;
    
    // ===== DEMO 1: PENCARIAN AKAR POSITIF =====
    printf("Mencari akar positif dengan tebakan awal x₀ = 1.0:\n");
    double root_pos = newton_raphson_real(&solver, 1.0, &iterations, &error, &converged);
    printf("Hasil: x = %.6f\n", root_pos);
    printf("Iterasi: %d\n", iterations);
    printf("Error akhir: %.2e\n", error);
    printf("Status: %s\n\n", converged ? "Konvergen" : "Tidak konvergen");
    
    // ===== DEMO 2: PENCARIAN AKAR NEGATIF =====
    printf("Mencari akar negatif dengan tebakan awal x₀ = -1.0:\n");
    double root_neg = newton_raphson_real(&solver, -1.0, &iterations, &error, &converged);
    printf("Hasil: x = %.6f\n", root_neg);
    printf("Iterasi: %d\n", iterations);
    printf("Error akhir: %.2e\n", error);
    printf("Status: %s\n", converged ? "Konvergen" : "Tidak konvergen");
}

/**
 * Menjalankan eksperimen komprehensif untuk berbagai nilai K
 * 
 * DESAIN EKSPERIMEN:
 * - Parameter tetap: m=1.0 kg, c=4.0 N⋅s/m, k=1.0 N/m  
 * - Parameter variabel: K dari 0.0 hingga 10.0 dengan increment 1.0
 * - Total 11 sistem yang dianalisis
 * 
 * TUJUAN:
 * - Menunjukkan efek gain kontroler terhadap karakteristik sistem
 * - Demonstrasi transisi dari overdamped → critical → underdamped
 * - Analisis stabilitas untuk range parameter yang realistis
 * 
 * PARAMETER SISTEM:
 * Persamaan karakteristik: s² + 4s + (1+K) = 0
 * Diskriminan: Δ = 16 - 4(1+K) = 12 - 4K
 * 
 * PREDIKSI TEORITIS:
 * - K < 3: Overdamped (Δ > 0)
 * - K = 3: Critical (Δ = 0)  
 * - K > 3: Underdamped (Δ < 0)
 */
void run_comprehensive_experiment(void) {
    print_separator('=', 60);
    printf("EKSPERIMEN KOMPREHENSIF\n");
    print_separator('=', 60);
    
    // Setup parameter dasar sistem
    SystemParameters base_params = {1.0, 4.0, 1.0, 0.0}; // m=1, c=4, k=1, K awal=0
    
    // Reset counter eksperimen untuk run baru
    num_experiments = 0;
    
    // Informasi eksperimen
    printf("Parameter tetap: m=1.0 kg, c=4.0 N.s/m, k=1.0 N/m\n");
    printf("Menganalisis sistem untuk K = 0.0 hingga K = 10.0 (increment 1.0)\n");
    printf("Persamaan karakteristik: s^2 + 4s + (1+K) = 0\n");
    printf("Diskriminan = 16 - 4(1+K) = 12 - 4K\n\n");
    
    // Loop utama eksperimen
    for (double K = 0.0; K <= 10.0 && num_experiments < MAX_SYSTEMS; K += 1.0) {
        SystemParameters params = base_params;
        params.gain = K;  // Update gain untuk iterasi ini
        
        // Prediksi berdasarkan diskriminan
        double discriminant = 16.0 - 4.0 * (1.0 + K);

        printf("K = %.1f: Diskriminan = %.1f → ", K, discriminant);
        if (discriminant > 0.01) printf("Overdamped\n");
        else if (fabs(discriminant) <= 0.01) printf("Critical Damped\n");
        else printf("Underdamped\n");
        
        // Lakukan analisis sistem untuk K saat ini
        AnalysisResult result = analyze_control_system(params);
        
        // Simpan hasil untuk analisis batch
        experiment_results[num_experiments] = result;
        num_experiments++;
        
        // Tampilkan hasil detail untuk K ini
        const char* types[] = {"Overdamped", "Critical", "Underdamped"};
        printf("Hasil: %s, %s", 
               types[result.system_type], 
               result.is_stable ? "Stabil" : "Tidak Stabil");
        
        // Format output tergantung jenis sistem
        if (result.system_type == 0) {
            printf(" (Akar: %.6f, %.6f)\n", result.root1.real, result.root2.real);
        } else if (result.system_type == 1) {
            printf(" (Akar berulang: %.6f)\n", result.root1.real);
        } else {
            printf(" (Akar: %.6f ± %.6fj)\n", result.root1.real, fabs(result.root1.imag));
        }
        printf("\n");
    }
    
    printf("Eksperimen selesai. Total %d sistem dianalisis.\n", num_experiments);
}

/**
 * Analisis performa algoritma Newton-Raphson
 * 
 * METRIK YANG DIANALISIS:
 * 1. Statistik Iterasi:
 *    - Rata-rata, minimum, maksimum iterasi
 *    - Distribusi konvergensi
 * 2. Statistik Error:
 *    - Rata-rata, minimum, maksimum error konvergensi
 * 3. Tingkat Keberhasilan:
 *    - Persentase konvergensi
 * 4. Distribusi Jenis Sistem:
 *    - Breakdown overdamped/critical/underdamped
 *    - Tingkat stabilitas
 * 
 * TUJUAN:
 * - Evaluasi robustness algoritma
 * - Identifikasi bottleneck performa
 * - Validasi parameter algoritma (toleransi, max iterasi)
 */
void performance_analysis(void) {
    print_separator('=', 60);
    printf("ANALISIS PERFORMA\n");
    print_separator('=', 60);
    
    if (num_experiments == 0) {
        printf("Tidak ada data eksperimen untuk analisis performa.\n");
        return;
    }
    
    // ===== INISIALISASI VARIABEL STATISTIK =====
    int total_iterations = 0;
    int max_iterations = 0;
    int min_iterations = MAX_ITERATIONS;
    int converged_count = 0;
    
    double total_error = 0.0;
    double max_error = 0.0;
    double min_error = 1.0;
    
    // ===== PENGUMPULAN STATISTIK UNTUK SISTEM OVERDAMPED =====
    // Catatan: Hanya sistem overdamped yang menggunakan Newton-Raphson iteratif
    for (int i = 0; i < num_experiments; i++) {
        AnalysisResult result = experiment_results[i];
        
        if (result.system_type == 0) { // Hanya untuk overdamped
            int iter = result.iterations1 + result.iterations2;
            double err = (result.error1 + result.error2) / 2.0;  // Error rata-rata
            
            // Update statistik iterasi
            total_iterations += iter;
            if (iter > max_iterations) max_iterations = iter;
            if (iter < min_iterations) min_iterations = iter;
            
            // Update statistik error
            total_error += err;
            if (err > max_error) max_error = err;
            if (err < min_error) min_error = err;
            
            // Count successful convergence
            if (result.converged1 && result.converged2) converged_count++;
        }
    }
    
    // ===== HITUNG DAN TAMPILKAN STATISTIK OVERDAMPED =====
    int overdamped_count = 0;
    for (int i = 0; i < num_experiments; i++) {
        if (experiment_results[i].system_type == 0) overdamped_count++;
    }
    
    if (overdamped_count > 0) {
        double avg_iterations = (double)total_iterations / overdamped_count;
        double avg_error = total_error / overdamped_count;
        double convergence_rate = (double)converged_count / overdamped_count * 100.0;
        
        printf("Statistik untuk sistem overdamped (%d sistem):\n", overdamped_count);
        printf("  Rata-rata iterasi: %.1f\n", avg_iterations);
        printf("  Iterasi minimum: %d\n", min_iterations);
        printf("  Iterasi maksimum: %d\n", max_iterations);
        printf("  Rata-rata error: %.2e\n", avg_error);
        printf("  Error minimum: %.2e\n", min_error);
        printf("  Error maksimum: %.2e\n", max_error);
        printf("  Tingkat konvergensi: %.1f%%\n", convergence_rate);
    }
    
    // ===== DISTRIBUSI JENIS SISTEM =====
    int overdamped = 0, critical = 0, underdamped = 0;
    int stable_count = 0;
    
    for (int i = 0; i < num_experiments; i++) {
        // Count system types
        switch (experiment_results[i].system_type) {
            case 0: overdamped++; break;
            case 1: critical++; break;
            case 2: underdamped++; break;
        }
        // Count stable systems
        if (experiment_results[i].is_stable) stable_count++;
    }
    
    printf("\nDistribusi jenis sistem:\n");
    printf("  Overdamped: %d (%.1f%%)\n", overdamped, 
           (double)overdamped / num_experiments * 100.0);
    printf("  Critical: %d (%.1f%%)\n", critical, 
           (double)critical / num_experiments * 100.0);
    printf("  Underdamped: %d (%.1f%%)\n", underdamped, 
           (double)underdamped / num_experiments * 100.0);
    printf("  Sistem stabil: %d (%.1f%%)\n", stable_count, 
           (double)stable_count / num_experiments * 100.0);
}

/**
 * Utility function untuk mencetak garis pemisah
 * 
 * KEGUNAAN:
 * - Formatting output untuk readability
 * - Konsistensi visual interface
 * - Separasi logis antar section
 * 
 * @param character Karakter yang akan diulang (biasanya '-' atau '=')
 * @param length Panjang garis yang diinginkan
 */
void print_separator(char character, int length) {
    for (int i = 0; i < length; i++) {
        printf("%c", character);
    }
    printf("\n");
}

// ============================================================================
// FUNGSI UTAMA
// ============================================================================

/**
 * Fungsi utama program - entry point dan menu interaktif
 * 
 * ARSITEKTUR MENU:
 * 1. Demo Newton-Raphson (untuk pembelajaran)
 * 2. Analisis individual (untuk eksperimen custom)
 * 3. Eksperimen komprehensif (untuk penelitian batch)
 * 4. Ringkasan eksperimen (untuk review hasil)
 * 5. Analisis performa (untuk evaluasi algoritma)
 * 6. Export CSV (untuk analisis eksternal)
 * 7. Exit program
 * 
 * DESIGN PATTERNS:
 * - Menu-driven interface untuk user experience
 * - Input validation untuk robustness
 * - Error handling untuk exceptional cases
 * - Clear screen untuk clean interface
 * 
 * @return 0 jika berhasil, non-zero jika error
 */
int main(void) {
    // Tampilkan header program dengan branding
    print_header();
    
    // Variabel untuk kontrol menu
    int choice;
    bool running = true;
    
    // ===== LOOP UTAMA MENU INTERAKTIF =====
    while (running) {
        printf("\n");
        print_separator('-', 60);
        printf("MENU UTAMA\n");
        print_separator('-', 60);
        printf("1. Demonstrasi Metode Newton-Raphson\n");
        printf("2. Analisis Sistem Kontrol Individual\n");
        printf("3. Eksperimen Komprehensif\n");
        printf("4. Tampilkan Ringkasan Eksperimen\n");
        printf("5. Analisis Performa\n");
        printf("6. Simpan Hasil ke File CSV\n");
        printf("7. Keluar\n");
        printf("Pilihan Anda: ");
        
        // ===== INPUT VALIDATION =====
        if (scanf("%d", &choice) != 1) {
            printf("Input tidak valid. Silakan masukkan angka.\n");
            // Clear input buffer untuk mencegah infinite loop
            int c;
            while ((c = getchar()) != '\n' && c != EOF);
            continue;
        }
        
        // ===== DISPATCH BERDASARKAN PILIHAN USER =====
        switch (choice) {
            case 1:
                // Demo metode Newton-Raphson dengan contoh sederhana
                demonstrate_newton_raphson();
                break;
                
            case 2: {
                // Analisis sistem individual dengan parameter custom
                SystemParameters params;
                printf("\nMasukkan parameter sistem:\n");
                printf("Massa (kg): ");
                scanf("%lf", &params.massa);
                printf("Damping (N⋅s/m): ");
                scanf("%lf", &params.damping);
                printf("Spring (N/m): ");
                scanf("%lf", &params.spring);
                printf("Gain kontroler: ");
                scanf("%lf", &params.gain);
                
                // Lakukan analisis dan tampilkan hasil
                AnalysisResult result = analyze_control_system(params);
                print_analysis_result(params, result);
                validate_with_analytical(params, result);
                break;
            }
            
            case 3:
                // Eksperimen batch untuk range parameter K
                run_comprehensive_experiment();
                break;
                
            case 4:
                // Tampilkan ringkasan hasil eksperimen sebelumnya
                print_experiment_summary();
                break;
                
            case 5:
                // Analisis performa algoritma Newton-Raphson
                performance_analysis();
                break;
                
            case 6: {
                // Export hasil ke file CSV
                char filename[MAX_FILENAME];
                printf("Masukkan nama file CSV (contoh: hasil.csv): ");
                scanf("%s", filename);
                save_results_to_csv(filename);
                break;
            }
            
            case 7:
                // Exit program dengan pesan terima kasih
                printf("Terima kasih telah menggunakan program ini!\n");
                running = false;
                break;
                
            default:
                // Handle invalid menu choice
                printf("Pilihan tidak valid. Silakan pilih 1-7.\n");
                break;
        }
        
        // ===== PAUSE DAN CLEAR SCREEN UNTUK UX =====
        if (running) {
            printf("\nTekan Enter untuk melanjutkan...");
            getchar(); // Konsumsi newline dari scanf sebelumnya
            getchar(); // Tunggu input Enter dari user
            system("clear || cls"); // Clear screen (Linux/Windows compatible)
        }
    }
    
    return 0; // Exit success
}