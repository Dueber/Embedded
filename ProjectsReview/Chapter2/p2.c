#include <stdio.h>
#include <stdlib.h>

// Problem 2
// Write a program that computes the volume of a sphere with a 10-meter radius, using the forumla v = 4/3 *pi *r^3

int main(int argc, char* argv[]){

    int v, r;

    r = 10;

    v = (4/3) * 3.14 * r * r * r;

    printf("Volume: %d", v);

    return 0;
}