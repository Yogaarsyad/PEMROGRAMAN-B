/*
 * ============================================================================
 * TUGAS PEMROGRAMAN B - KOMPUTASI NUMERIK
 * Implementasi Metode Newton-Raphson untuk Analisis Stabilitas Sistem Kontrol
 *
 * Kelompok 3:
 * 1. Daffa Hardhan
 * 2. Muhammad Bryan Farras
 * 3. Tri Yoga Arsyad
 *
 * Program Studi Teknik Komputer
 * Universitas Indonesia
 * 2024
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

// ============================================================================
// KONSTANTA DAN DEFINISI
// ============================================================================

#define MAX_ITERATIONS 100          // Maksimum iterasi Newton-Raphson
#define TOLERANCE 1e-6              // Toleransi konvergensi
#define MAX_SYSTEMS 50              // Maksimum sistem yang dapat dianalisis
#define MAX_FILENAME 256            // Panjang maksimum nama file

// ============================================================================
// STRUKTUR DATA
// ============================================================================

/**
 * Struktur untuk menyimpan bilangan kompleks
 */
typedef struct {
    double real;    // Bagian real
    double imag;    // Bagian imajiner
} Complex;

/**
 * Struktur untuk parameter sistem kontrol massa-pegas-damper
 */
typedef struct {
    double massa;       // Massa (m) dalam kg
    double damping;     // Koefisien redaman (c) dalam N⋅s/m
    double spring;      // Konstanta pegas (k) dalam N/m
    double gain;        // Gain kontroler (K)
} SystemParameters;

/**
 * Struktur untuk hasil analisis Newton-Raphson
 */
typedef struct {
    Complex root1;          // Akar pertama
    Complex root2;          // Akar kedua
    int iterations1;        // Jumlah iterasi untuk akar pertama
    int iterations2;        // Jumlah iterasi untuk akar kedua
    double error1;          // Error akhir untuk akar pertama
    double error2;          // Error akhir untuk akar kedua
    bool converged1;        // Status konvergensi akar pertama
    bool converged2;        // Status konvergensi akar kedua
    int system_type;        // 0: overdamped, 1: critical, 2: underdamped
    bool is_stable;         // Status stabilitas sistem
    double discriminant;    // Nilai diskriminan
} AnalysisResult;

/**
 * Struktur untuk solver Newton-Raphson
 */
typedef struct {
    double tolerance;       // Toleransi konvergensi
    int max_iterations;     // Maksimum iterasi
    SystemParameters params; // Parameter sistem saat ini
} NewtonRaphsonSolver;

// ============================================================================
// VARIABEL GLOBAL
// ============================================================================

// Array untuk menyimpan hasil eksperimen
AnalysisResult experiment_results[MAX_SYSTEMS];
int num_experiments = 0;

// ============================================================================
// DEKLARASI FUNGSI
// ============================================================================

// Fungsi untuk menampilkan header program
void print_header(void);

// Fungsi untuk persamaan karakteristik f(s) = s² + (c/m)s + (k+K)/m
double characteristic_equation(double s, SystemParameters params);

// Fungsi untuk turunan persamaan karakteristik f'(s) = 2s + (c/m)
double characteristic_derivative(double s, SystemParameters params);

// Implementasi metode Newton-Raphson untuk mencari akar real
double newton_raphson_real(NewtonRaphsonSolver* solver, double initial_guess,
                          int* iterations, double* final_error, bool* converged);

// Fungsi untuk menghitung diskriminan persamaan kuadrat
double calculate_discriminant(SystemParameters params);

// Fungsi untuk analisis lengkap sistem kontrol
AnalysisResult analyze_control_system(SystemParameters params);

// Fungsi untuk validasi hasil dengan solusi analitik
void validate_with_analytical(SystemParameters params, AnalysisResult result);

// Fungsi untuk mencetak hasil analisis individual
void print_analysis_result(SystemParameters params, AnalysisResult result);

// Fungsi untuk mencetak ringkasan eksperimen
void print_experiment_summary(void);

// Fungsi untuk menyimpan hasil ke file CSV
void save_results_to_csv(const char* filename);

// Fungsi untuk demonstrasi metode Newton-Raphson
void demonstrate_newton_raphson(void);

// Fungsi untuk menjalankan eksperimen komprehensif
void run_comprehensive_experiment(void);

// Fungsi untuk analisis performa
void performance_analysis(void);

// Fungsi utilitas untuk mencetak garis pemisah
void print_separator(char character, int length);

// ============================================================================
// IMPLEMENTASI FUNGSI
// ============================================================================

/**
 * Menampilkan header program dengan informasi kelompok
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
 * @param s Nilai variabel s
 * @param params Parameter sistem kontrol
 * @return Nilai fungsi karakteristik pada s
 */
double characteristic_equation(double s, SystemParameters params) {
    double b = params.damping / params.massa;                    // Koefisien s
    double c = (params.spring + params.gain) / params.massa;     // Konstanta
    
    return s * s + b * s + c;
}

/**
 * Fungsi turunan persamaan karakteristik: f'(s) = 2s + (c/m)
 * 
 * @param s Nilai variabel s
 * @param params Parameter sistem kontrol
 * @return Nilai turunan fungsi karakteristik pada s
 */
double characteristic_derivative(double s, SystemParameters params) {
    double b = params.damping / params.massa;
    
    return 2.0 * s + b;
}

/**
 * Implementasi metode Newton-Raphson untuk mencari akar real
 * 
 * @param solver Pointer ke struktur solver
 * @param initial_guess Tebakan awal
 * @param iterations Pointer untuk menyimpan jumlah iterasi
 * @param final_error Pointer untuk menyimpan error akhir
 * @param converged Pointer untuk status konvergensi
 * @return Nilai akar yang ditemukan
 */
double newton_raphson_real(NewtonRaphsonSolver* solver, double initial_guess,
                          int* iterations, double* final_error, bool* converged) {
    
    double x = initial_guess;  // Nilai saat ini
    double fx, dfx, x_new;     // Variabel untuk perhitungan
    
    *converged = false;
    *iterations = 0;
    *final_error = 0.0;
    
    // Iterasi Newton-Raphson
    for (int i = 0; i < solver->max_iterations; i++) {
        // Hitung nilai fungsi dan turunannya
        fx = characteristic_equation(x, solver->params);
        dfx = characteristic_derivative(x, solver->params);
        
        // Periksa apakah turunan terlalu kecil (untuk menghindari pembagian dengan nol)
        if (fabs(dfx) < solver->tolerance) {
            printf("Peringatan: Turunan terlalu kecil pada iterasi %d\n", i);
            break;
        }
        
        // Hitung nilai baru menggunakan formula Newton-Raphson
        x_new = x - fx / dfx;
        
        // Hitung error
        *final_error = fabs(x_new - x);
        
        // Periksa kriteria konvergensi
        if (*final_error < solver->tolerance || fabs(fx) < solver->tolerance) {
            *iterations = i + 1;
            *converged = true;
            return x_new;
        }
        
        // Update nilai untuk iterasi berikutnya
        x = x_new;
    }
    
    // Jika mencapai maksimum iterasi tanpa konvergensi
    *iterations = solver->max_iterations;
    return x;
}

/**
 * Menghitung diskriminan persamaan kuadrat ax² + bx + c = 0
 * 
 * @param params Parameter sistem kontrol
 * @return Nilai diskriminan
 */
double calculate_discriminant(SystemParameters params) {
    double a = 1.0;
    double b = params.damping / params.massa;
    double c = (params.spring + params.gain) / params.massa;
    
    return b * b - 4.0 * a * c;
}

/**
 * Fungsi utama untuk analisis lengkap sistem kontrol
 * 
 * @param params Parameter sistem kontrol
 * @return Struktur hasil analisis
 */
AnalysisResult analyze_control_system(SystemParameters params) {
    AnalysisResult result = {0}; // Inisialisasi dengan nol
    
    // Setup solver
    NewtonRaphsonSolver solver;
    solver.tolerance = TOLERANCE;
    solver.max_iterations = MAX_ITERATIONS;
    solver.params = params;
    
    // Hitung diskriminan untuk menentukan jenis akar
    result.discriminant = calculate_discriminant(params);
    
    // Inisialisasi stabilitas sebagai true (akan diubah jika ada akar dengan bagian real positif)
    result.is_stable = true;
    
    if (result.discriminant > TOLERANCE) {
        // Dua akar real berbeda (Overdamped)
        result.system_type = 0;
        
        printf("Sistem Overdamped: Mencari dua akar real...\n");
        
        // Cari akar pertama dengan tebakan awal -0.1
        result.root1.real = newton_raphson_real(&solver, -0.1,
                                               &result.iterations1,
                                               &result.error1,
                                               &result.converged1);
        result.root1.imag = 0.0;
        
        // Cari akar kedua dengan tebakan awal -10.0
        result.root2.real = newton_raphson_real(&solver, -10.0,
                                               &result.iterations2,
                                               &result.error2,
                                               &result.converged2);
        result.root2.imag = 0.0;
        
        // Periksa stabilitas (semua akar harus memiliki bagian real negatif)
        if (result.root1.real >= 0 || result.root2.real >= 0) {
            result.is_stable = false;
        }
        
    } else if (fabs(result.discriminant) <= TOLERANCE) {
        // Satu akar real berulang (Critically Damped)
        result.system_type = 1;
        
        printf("Sistem Critically Damped: Satu akar real berulang...\n");
        
        // Hitung akar langsung menggunakan rumus -b/(2a)
        double b = params.damping / params.massa;
        result.root1.real = -b / 2.0;
        result.root1.imag = 0.0;
        result.root2 = result.root1; // Akar berulang
        
        // Set parameter iterasi (tidak menggunakan Newton-Raphson)
        result.iterations1 = 1;
        result.iterations2 = 1;
        result.error1 = 0.0;
        result.error2 = 0.0;
        result.converged1 = true;
        result.converged2 = true;
        
        // Periksa stabilitas
        if (result.root1.real >= 0) {
            result.is_stable = false;
        }
        
    } else {
        // Akar kompleks konjugat (Underdamped)
        result.system_type = 2;
        
        printf("Sistem Underdamped: Akar kompleks konjugat...\n");
        
        // Hitung akar kompleks menggunakan rumus kuadrat
        double a = 1.0;
        double b = params.damping / params.massa;
        double c = (params.spring + params.gain) / params.massa;
        
        double real_part = -b / (2.0 * a);
        double imag_part = sqrt(-result.discriminant) / (2.0 * a);
        
        result.root1.real = real_part;
        result.root1.imag = imag_part;
        result.root2.real = real_part;
        result.root2.imag = -imag_part;
        
        // Set parameter iterasi (tidak menggunakan Newton-Raphson untuk akar kompleks)
        result.iterations1 = 1;
        result.iterations2 = 1;
        result.error1 = 0.0;
        result.error2 = 0.0;
        result.converged1 = true;
        result.converged2 = true;
        
        // Periksa stabilitas (bagian real harus negatif)
        if (real_part >= 0) {
            result.is_stable = false;
        }
    }
    
    return result;
}

/**
 * Validasi hasil numerik dengan solusi analitik
 * 
 * @param params Parameter sistem kontrol
 * @param result Hasil analisis numerik
 */
void validate_with_analytical(SystemParameters params, AnalysisResult result) {
    printf("\n--- VALIDASI DENGAN SOLUSI ANALITIK ---\n");
    
    // Koefisien persamaan kuadrat as² + bs + c = 0
    double a = 1.0;
    double b = params.damping / params.massa;
    double c = (params.spring + params.gain) / params.massa;
    
    printf("Persamaan kuadrat: s² + %.6fs + %.6f = 0\n", b, c);
    printf("Diskriminan = %.6f\n", result.discriminant);
    
    if (result.discriminant >= 0) {
        // Akar real
        double sqrt_disc = sqrt(result.discriminant);
        double analytical_root1 = (-b + sqrt_disc) / (2.0 * a);
        double analytical_root2 = (-b - sqrt_disc) / (2.0 * a);
        
        printf("Solusi analitik: s₁ = %.6f, s₂ = %.6f\n", 
               analytical_root1, analytical_root2);
        printf("Solusi numerik:  s₁ = %.6f, s₂ = %.6f\n", 
               result.root1.real, result.root2.real);
        
        // Hitung error relatif
        double error1 = fabs(result.root1.real - analytical_root1) / fabs(analytical_root1) * 100.0;
        double error2 = fabs(result.root2.real - analytical_root2) / fabs(analytical_root2) * 100.0;
        
        printf("Error relatif:   %.6f%%, %.6f%%\n", error1, error2);
        
    } else {
        // Akar kompleks
        double real_part = -b / (2.0 * a);
        double imag_part = sqrt(-result.discriminant) / (2.0 * a);
        
        printf("Solusi analitik: s = %.6f ± %.6fj\n", real_part, imag_part);
        printf("Solusi numerik:  s = %.6f ± %.6fj\n", 
               result.root1.real, result.root1.imag);
        
        // Hitung error relatif
        double error_real = fabs(result.root1.real - real_part) / fabs(real_part) * 100.0;
        double error_imag = fabs(result.root1.imag - imag_part) / fabs(imag_part) * 100.0;
        
        printf("Error relatif:   Real: %.6f%%, Imag: %.6f%%\n", error_real, error_imag);
    }
}

/**
 * Mencetak hasil analisis individual
 * 
 * @param params Parameter sistem kontrol
 * @param result Hasil analisis
 */
void print_analysis_result(SystemParameters params, AnalysisResult result) {
    print_separator('-', 60);
    printf("HASIL ANALISIS SISTEM KONTROL\n");
    print_separator('-', 60);
    
    // Parameter sistem
    printf("Parameter Sistem:\n");
    printf("  Massa (m)      = %.3f kg\n", params.massa);
    printf("  Damping (c)    = %.3f N⋅s/m\n", params.damping);
    printf("  Spring (k)     = %.3f N/m\n", params.spring);
    printf("  Gain (K)       = %.3f\n", params.gain);
    
    // Persamaan karakteristik
    double b = params.damping / params.massa;
    double c = (params.spring + params.gain) / params.massa;
    printf("\nPersamaan Karakteristik: s² + %.3fs + %.3f = 0\n", b, c);
    printf("Diskriminan: %.6f\n", result.discriminant);
    
    // Jenis sistem
    const char* system_types[] = {"Overdamped", "Critically Damped", "Underdamped"};
    printf("Jenis Sistem: %s\n", system_types[result.system_type]);
    
    // Akar-akar
    printf("\nAkar-akar:\n");
    if (result.system_type == 2) { // Underdamped
        printf("  s₁ = %.6f + %.6fj\n", result.root1.real, result.root1.imag);
        printf("  s₂ = %.6f - %.6fj\n", result.root2.real, result.root2.imag);
        printf("  Frekuensi osilasi: %.6f rad/s\n", fabs(result.root1.imag));
    } else {
        printf("  s₁ = %.6f", result.root1.real);
        if (result.converged1) {
            printf(" (konvergen dalam %d iterasi, error: %.2e)\n", 
                   result.iterations1, result.error1);
        } else {
            printf(" (tidak konvergen)\n");
        }
        
        if (result.system_type == 0) { // Overdamped - dua akar berbeda
            printf("  s₂ = %.6f", result.root2.real);
            if (result.converged2) {
                printf(" (konvergen dalam %d iterasi, error: %.2e)\n", 
                       result.iterations2, result.error2);
            } else {
                printf(" (tidak konvergen)\n");
            }
        }
    }
    
    // Analisis stabilitas
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
 * Mencetak ringkasan eksperimen
 */
void print_experiment_summary(void) {
    if (num_experiments == 0) {
        printf("Tidak ada data eksperimen untuk ditampilkan.\n");
        return;
    }
    
    print_separator('=', 80);
    printf("RINGKASAN EKSPERIMEN KOMPREHENSIF\n");
    print_separator('=', 80);
    
    printf("%-5s %-12s %-12s %-15s %-10s %-8s\n", 
           "K", "Akar 1", "Akar 2", "Jenis", "Stabilitas", "Iterasi");
    print_separator('-', 80);
    
    for (int i = 0; i < num_experiments; i++) {
        AnalysisResult result = experiment_results[i];
        
        // UBAH: Hitung gain K berdasarkan increment 1.0
        double K = i * 1.0;
        
        const char* types[] = {"Overdamped", "Critical", "Underdamped"};
        const char* stability = result.is_stable ? "Stabil" : "Tidak Stabil";
        
        if (result.system_type == 2) { // Underdamped
            printf("%-5.1f %-12s %-12s %-15s %-10s %-8s\n",
                   K, "Kompleks", "Kompleks", types[result.system_type], 
                   stability, "N/A");
        } else {
            printf("%-5.1f %-12.6f %-12.6f %-15s %-10s %-8d\n",
                   K, result.root1.real, result.root2.real, 
                   types[result.system_type], stability, 
                   result.iterations1 + result.iterations2);
        }
    }
}

/**
 * Menyimpan hasil eksperimen ke file CSV
 * 
 * @param filename Nama file output
 */
void save_results_to_csv(const char* filename) {
    FILE* file = fopen(filename, "w");
    if (file == NULL) {
        printf("Error: Tidak dapat membuka file %s untuk ditulis.\n", filename);
        return;
    }
    
    // Header CSV
    fprintf(file, "K,Root1_Real,Root1_Imag,Root2_Real,Root2_Imag,");
    fprintf(file, "System_Type,Is_Stable,Iterations1,Iterations2,");
    fprintf(file, "Error1,Error2,Discriminant\n");
    
    // Data
    for (int i = 0; i < num_experiments; i++) {
        AnalysisResult result = experiment_results[i];
        double K = i * 1.0; // UBAH: increment 1.0 sesuai laporan
        
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
 */
void demonstrate_newton_raphson(void) {
    print_separator('=', 60);
    printf("DEMONSTRASI METODE NEWTON-RAPHSON\n");
    print_separator('=', 60);
    
    printf("Contoh: Mencari akar dari f(x) = x² - 4 = 0\n");
    printf("Akar analitik: x = ±2\n\n");
    
    // Setup sistem sederhana untuk demo: x² - 4 = 0
    // Kita akan mengubah parameter sehingga persamaan karakteristik menjadi s² - 4 = 0
    SystemParameters demo_params = {1.0, 0.0, -4.0, 0.0}; // m=1, c=0, k=-4, K=0
    
    NewtonRaphsonSolver solver;
    solver.tolerance = TOLERANCE;
    solver.max_iterations = MAX_ITERATIONS;
    solver.params = demo_params;
    
    // Cari akar positif
    int iterations;
    double error;
    bool converged;
    
    printf("Mencari akar positif dengan tebakan awal x₀ = 1.0:\n");
    double root_pos = newton_raphson_real(&solver, 1.0, &iterations, &error, &converged);
    printf("Hasil: x = %.6f\n", root_pos);
    printf("Iterasi: %d\n", iterations);
    printf("Error akhir: %.2e\n", error);
    printf("Status: %s\n\n", converged ? "Konvergen" : "Tidak konvergen");
    
    // Cari akar negatif
    printf("Mencari akar negatif dengan tebakan awal x₀ = -1.0:\n");
    double root_neg = newton_raphson_real(&solver, -1.0, &iterations, &error, &converged);
    printf("Hasil: x = %.6f\n", root_neg);
    printf("Iterasi: %d\n", iterations);
    printf("Error akhir: %.2e\n", error);
    printf("Status: %s\n", converged ? "Konvergen" : "Tidak konvergen");
}

/**
 * Menjalankan eksperimen komprehensif untuk berbagai nilai K
 */
void run_comprehensive_experiment(void) {
    print_separator('=', 60);
    printf("EKSPERIMEN KOMPREHENSIF\n");
    print_separator('=', 60);
    
    // PARAMETER SISTEM YANG BENAR SESUAI LAPORAN
    SystemParameters base_params = {1.0, 4.0, 1.0, 0.0}; // m=1, c=4, k=1, K awal
    
    // Reset counter eksperimen
    num_experiments = 0;
    
    printf("Parameter tetap: m=1.0 kg, c=4.0 N.s/m, k=1.0 N/m\n");
    printf("Menganalisis sistem untuk K = 0.0 hingga K = 10.0 (increment 1.0)\n");
    printf("Persamaan karakteristik: s^2 + 4s + (1+K) = 0\n");
    printf("Diskriminan = 16 - 4(1+K) = 12 - 4K\n\n");
    
    // Loop untuk berbagai nilai K (increment 1.0 sesuai laporan)
    for (double K = 0.0; K <= 10.0 && num_experiments < MAX_SYSTEMS; K += 1.0) {
        SystemParameters params = base_params;
        params.gain = K;
        
        // Hitung diskriminan untuk prediksi
        double discriminant = 16.0 - 4.0 * (1.0 + K);

        printf("K = %.1f: Diskriminan = %.1f → ", K, discriminant);
        if (discriminant > 0.01) printf("Overdamped\n");
        else if (fabs(discriminant) <= 0.01) printf("Critical Damped\n");
        else printf("Underdamped\n");
        
        // Analisis sistem
        AnalysisResult result = analyze_control_system(params);
        
        // Simpan hasil
        experiment_results[num_experiments] = result;
        num_experiments++;
        
        // Cetak hasil detail
        const char* types[] = {"Overdamped", "Critical", "Underdamped"};
        printf("Hasil: %s, %s", 
               types[result.system_type], 
               result.is_stable ? "Stabil" : "Tidak Stabil");
        
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
 * Analisis performa algoritma
 */
void performance_analysis(void) {
    print_separator('=', 60);
    printf("ANALISIS PERFORMA\n");
    print_separator('=', 60);
    
    if (num_experiments == 0) {
        printf("Tidak ada data eksperimen untuk analisis performa.\n");
        return;
    }
    
    // Statistik iterasi
    int total_iterations = 0;
    int max_iterations = 0;
    int min_iterations = MAX_ITERATIONS;
    int converged_count = 0;
    
    // Statistik error
    double total_error = 0.0;
    double max_error = 0.0;
    double min_error = 1.0;
    
    for (int i = 0; i < num_experiments; i++) {
        AnalysisResult result = experiment_results[i];
        
        if (result.system_type == 0) { // Hanya untuk overdamped yang menggunakan Newton-Raphson
            int iter = result.iterations1 + result.iterations2;
            double err = (result.error1 + result.error2) / 2.0;
            
            total_iterations += iter;
            if (iter > max_iterations) max_iterations = iter;
            if (iter < min_iterations) min_iterations = iter;
            
            total_error += err;
            if (err > max_error) max_error = err;
            if (err < min_error) min_error = err;
            
            if (result.converged1 && result.converged2) converged_count++;
        }
    }
    
    // Hitung rata-rata
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
    
    // Distribusi jenis sistem
    int overdamped = 0, critical = 0, underdamped = 0;
    int stable_count = 0;
    
    for (int i = 0; i < num_experiments; i++) {
        switch (experiment_results[i].system_type) {
            case 0: overdamped++; break;
            case 1: critical++; break;
            case 2: underdamped++; break;
        }
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
 * Mencetak garis pemisah
 * 
 * @param character Karakter untuk garis pemisah
 * @param length Panjang garis
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
 * Fungsi utama program
 */
int main(void) {
    // Tampilkan header
    print_header();
    
    // Menu interaktif
    int choice;
    bool running = true;
    
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
        
        if (scanf("%d", &choice) != 1) {
            printf("Input tidak valid. Silakan masukkan angka.\n");
            // Bersihkan buffer input
            int c;
            while ((c = getchar()) != '\n' && c != EOF);
            continue;
        }
        
        switch (choice) {
            case 1:
                demonstrate_newton_raphson();
                break;
                
            case 2: {
                // Input parameter sistem
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
                
                // Analisis sistem
                AnalysisResult result = analyze_control_system(params);
                print_analysis_result(params, result);
                validate_with_analytical(params, result);
                break;
            }
            
            case 3:
                run_comprehensive_experiment();
                break;
                
            case 4:
                print_experiment_summary();
                break;
                
            case 5:
                performance_analysis();
                break;
                
            case 6: {
                char filename[MAX_FILENAME];
                printf("Masukkan nama file CSV (contoh: hasil.csv): ");
                scanf("%s", filename);
                save_results_to_csv(filename);
                break;
            }
            
            case 7:
                printf("Terima kasih telah menggunakan program ini!\n");
                running = false;
                break;
                
            default:
                printf("Pilihan tidak valid. Silakan pilih 1-7.\n");
                break;
        }
        
        if (running) {
            printf("\nTekan Enter untuk melanjutkan...");
            getchar(); // Konsumsi newline dari scanf sebelumnya
            getchar(); // Tunggu Enter dari user
            system("clear || cls"); // Bersihkan layar (Linux/Windows)
        }
    }
    
    return 0;
}