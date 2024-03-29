#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Problem 1
// Modify repdigit.c (book reference) to show repeated digits

int main(int argc, char* argv[]){

    int digit, i, occurrences[10] = {0};
    int array[10] = {-1};
    long n;

    printf("Enter a number: ");
    scanf("%ld", &n);

    while (n > 0) {
        digit = n % 10;
        occurrences[digit]++;
        n /= 10;
    }

    printf("Digit:\t\t 0  1  2  3  4  5  6  7  8  9\n");
    printf("Occurrences:\t");

    for (i = 0; i < 10; i++) {
        printf("%2d ", occurrences[i]);
    }
    printf("\n");

    return 0;
}