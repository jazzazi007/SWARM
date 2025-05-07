#include <stdio.h>

int main() {
    FILE *fptr;
    fptr = fopen("data.csv", "w");
    if (fptr == NULL) {
        printf("File cannot be opened\n");
        return 1;
    }

    // Example data
    char name[] = "Student1";
    int mark1 = 90, mark2 = 95, mark3 = 88, mark4 = 92, mark5 = 91;

    // Write the header
    fprintf(fptr, "ST_ID, M1, M2, M3, M4, M5\n");

    // Write the data
    int i = 0;
    while(i < 20)
    {
        fprintf(fptr, "%s,%d,%d,%d,%d,%d\n", name, mark1+i, mark2+i, mark3+i, mark4+i, mark5+i);
        i++;
    }

    fclose(fptr);
    return 0;
}