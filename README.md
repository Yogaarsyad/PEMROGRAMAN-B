# Fixed-Point Iteration Method – Solving cos(x) = x

## Group Number 3

## Group Members
- Daffa Hardhan - 2306161763
- Muhammad Bryan Farras - 2306230975
- Tri Yoga Arsyad - 2306161920


## Project Title
**Fixed-Point Iteration Method – Solving cos(x) = x**

## Description
This project demonstrates the implementation of the Fixed-Point Iteration method using the C programming language to solve the equation:

**cos(x) = x**

Since this nonlinear equation cannot be solved algebraically, we use a numerical iterative method to approximate the solution. The method involves transforming the equation into the form:
x = g(x) = cos(x)

Starting from an initial guess x₀ = 0.5, the program iteratively computes x₁ = cos(x₀), x₂ = cos(x₁), and so on, until the values converge (i.e., the difference between iterations is less than the defined error tolerance, EPSILON = 0.0001). A maximum of 100 iterations (MAX_ITER) is set to prevent infinite loops. 

**After 22 iterations, the value of x converges to approximately 0.739050, which is the solution to the equation cos(x) = x.**

## Method Summary
- Function:     g(x) = cos(x)
- Initial Guess:   x₀ = 0.5
- Error Tolerance:  EPSILON = 0.0001
- Maximum Iterations: 100

# Source Code (C)

```c
// FIXED-POINT ITERATION – GROUP 3
#include <stdio.h>
#include <math.h> // Required for cos().

#define EPSILON 0.0001  // Error tolerance.
#define MAX_ITER 100    // Maximum iterations.

int main() {
    float x0 = 0.5; // Initial guess.
    float x1;
    int iter = 0;

    printf("Fixed-Point Iteration Method: x = cos(x)\n");
    printf("Iteration\t\tx\n");

    do {
        x1 = cos(x0); // Compute next x using x(i+1) = cos(x(i)).
        printf("%d\t\t\t%.6f\n", iter + 1, x1);

        // Check if the result is within the error tolerance
        if (fabs(x1 - x0) < EPSILON) {
            break; // Stop if change is small enough.
        }

        x0 = x1; // Update for next iteration.
        iter++;

    } while (iter < MAX_ITER);

    printf("\nApproximate Root: %.6f\n", x1);
    return 0;
}

```

# Output Program:
![Output](https://github.com/user-attachments/assets/2712e396-c450-4869-80a4-522156a111cf)
