#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Problem 7
// User input integers for 5 x 5 array. Print sum of row and columns.

int main(int argc, char *argv[])
{
    int arr[5][5];
    
    int nums;
    int rowSum[5] = {0};
    int columnSum[5] = {0};

    // printf("Enter 25 numbers: ");
    printf("Enter 25 numbers: ");
    for(int i = 0; i < 5; i++){
        for(int j = 0; j < 5; j++){
            scanf("%d", &arr[i][j]);
        } 
    }

    for(int i = 0; i < 5; i++){
        printf("Row %d: ", i + 1);
        for(int j = 0; j < 5; j++){
            printf("%d ", arr[i][j]);
            rowSum[i] += arr[i][j];
            columnSum[j] += arr[i][j];
        }
        printf("\n");
    }

    printf("Row totals: ");
    for(int i = 0; i < 5; i++)
        printf("%d ", rowSum[i]);

    printf("\n");

    printf("Column totals: ");
    for (int i = 0; i < 5; i++)
        printf("%d ", columnSum[i]);

    return 0;
}