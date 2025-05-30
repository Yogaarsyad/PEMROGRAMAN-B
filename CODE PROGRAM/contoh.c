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
