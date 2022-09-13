#include <stdio.h>
#include <stdlib.h>

// Problem 6
// Display values of sizeof(type). Type(int, short, long, float, double, long double).



int main(int argc, char* argv[]){
                                                                // Size (bytes)
    printf("Size of int: %d\n", sizeof(int));                   // 4
    printf("Size of char: %d\n", sizeof(char));                 // 1
    printf("Size of short: %d\n", sizeof(short));               // 2
    printf("Size of long: %d\n", sizeof(long));                 // 4
    printf("Size of float: %d\n", sizeof(float));               // 4
    printf("Size of double: %d\n", sizeof(double));             // 8
    printf("Size of long double: %d\n", sizeof(long double));   // 12

    return 0;
}