#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Problem 5
// User enters the value of x of a polynomial.

float doMath(int x){
    float answer;
    printf("Value of x: %d\n", x);
    answer = (3*(x*x*x*x*x) + 2*(x*x*x*x) - 5*(x*x*x) - (x*x) + 7*x - 6);
    return answer;
}

int main(int argc, char* argv[]){

    int x;
    float answer;

    printf("Enter a value of x: ");
    scanf("%d", &x);

    printf("Answer: %.2f", doMath(x));

    return 0;
}