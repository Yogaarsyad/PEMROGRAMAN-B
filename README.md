#  Implementasi Metode Newton-Raphson untuk Analisis Stabilitas Sistem Kontrol: Pendekatan Komputasi Numerik

## Group Number 3

## Group Members
- Daffa Hardhan - 2306161763
- Muhammad Bryan Farras - 2306230975
- Tri Yoga Arsyad - 2306161920

## Description
This project aims to implement the Newton-Raphson method in the C programming language as a tool for stability analysis of second-order control systems, especially in the mass-spring-damper model with variable controller gain. With this numerical approach, the roots of the system's characteristic equation can be calculated efficiently, allowing for evaluation of system stability based on the position of eigenvalues ​​in the complex plane. This program also provides a feature for validating numerical results with analytical solutions using the quadratic formula, as well as classifying system behavior (overdamped, critically damped, underdamped) based on discriminant values. This implementation is expected to be a practical reference for engineering students and practitioners in understanding and applying numerical methods to control system analysis.

## Method Summary
The method used is Newton-Raphson, which is an iterative algorithm to find the roots of nonlinear equations by utilizing function derivatives. In the context of a mass-spring-damper control system, the characteristic equation analyzed is in the form of a quadratic, namely:

s² + (c/m)s + (k + K)/m = 0

with parameters:

m = system mass
c = damping coefficient
k = spring constant
K = controller gain (varied)
Main steps:

Determine the roots of the characteristic equation using the Newton-Raphson method with an error tolerance of 1e-6 and a maximum of 100 iterations.

Classify the type of system based on the discriminant value (Δ): overdamped (Δ > 0), critically damped (Δ = 0), and underdamped (Δ < 0).

Evaluate the stability of the system based on the real part of the roots (stable if all roots have negative real parts).
Validates the numerical results with the analytical solution of the quadratic formula to ensure the accuracy and reliability of the method. The program also supports batch analysis for various controller gain values, and displays a summary of the results and convergence statistics.


### Program yang tepat (benar berda di branch Dapa)
