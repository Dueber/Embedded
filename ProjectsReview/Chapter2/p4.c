#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Problem 4
// Write a program asking a user for a dollar and cents ammount. Add %5 tax to the amount and display.

int main(int argc, char* argv[]){

    float amount = 0.0;
    float tax = 0.05;

    printf("Enter dollar and cents amount: ");

    scanf("%f", &amount);

    amount = amount + (amount * tax);

    printf("%.2f\n", amount);

    return 0;
}