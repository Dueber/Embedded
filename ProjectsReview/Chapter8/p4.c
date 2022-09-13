#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Problem 4
// ** Find string length with byte size! 

#define N 5

int main(int argc, char *argv[]) {

    int a[N], i, len;
    len = (sizeof(a) / sizeof(a[0]));

    printf("Enter %d numbers: ", N);
    for (i = 0; i < len; i++)
        scanf("%d", &a[i]);
    

    printf("Reverse order: ");
    for (i = len - 1; i >= 0; i--)
        printf(" %d", a[i]);
    
    printf("\n");

    return 0;
}