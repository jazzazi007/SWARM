#include <stdio.h>

/*int main() {
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
}*/
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h> // For sleep function


int get_time_sec()
{
    struct timeval tv;
    struct timezone t_z;

    gettimeofday(&tv, NULL);
    return (tv.tv_sec);
}

int min2sec(float min)
{
    return min*60;
}
typedef struct time
{
    int prev_sec;
    int current_sec;
} time_s;


int main() {
    int time_sec = get_time_sec();
    printf("Current time in seconds since epoch: %d\n", time_sec);
    time_s t;
    t.prev_sec = get_time_sec();
    t.current_sec = get_time_sec();
    while (t.current_sec  - t.prev_sec <= min2sec(0.5))
    {
        t.current_sec = get_time_sec();
        printf("Current time in seconds since epoch: %d\n", t.current_sec - t.prev_sec);
        sleep(1); // Sleep for 1 second to simulate waiting
    }
    printf("1 minute has passed since %d seconds\n", t.prev_sec);
    return 0;
}