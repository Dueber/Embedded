#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int main(int argc, char* argv[]){

    int number, tens, ones;

    printf("Enter a two-digit number: ");
    scanf("%d", &number);

    ones = number / 10;
    tens = number % 10;

    printf("The reverse: %d%d", tens, ones);
    fflush(stdout);
    return 0;
}       