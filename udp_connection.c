/*
 * File: udp_connection.c
 * Author: Jazzazi 
 * 
 *          maaljazzazi22@eng.just.edu.jo
 * 
 * Description: This file contains the implementation of a MAVLink heartbeat sender and receiver.
 *              It sends heartbeat messages to a specified IP address and UDP port, and listens
 *              for incoming MAVLink messages.
 */
#include "../include/gnc.h"

atomic_int stop_flag = 0;

static int open_socket()
{
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket");
        return -1;
    }
    fprintf(stderr, "Socket opened \n");
    return sock;
}

int main(int ac, char **av) {
    thread_data_t thread_data;
    sockport sock;
    sts sts;
    mavlink_str mavlink_str;
    gains gains;
    pthread_t thread[4];
    memset(&thread_data, 0, sizeof(thread_data));  // Initialize memory to zero
    memset(&sock, 0, sizeof(sock));  // Initialize memory to zero
    memset(&sts, 0, sizeof(sts));  // Initialize memory to zero
    joy_s joy;
    joy.pitch = 1500;
    joy.roll = 1500;
    joy.throttle = 1500;
    joy.yaw = 1500;
    sts.mission_state = 0;     //here the mission starts to defined as 0, this means the mission is still not excuted, but there is data being recieved.
    sts.N_gain = 3.0;
    sts.last_los_angle = -10;
    sts.last_flight_path_angle = -2;
    sts.k_gain = 0.8;


    if (ac != 3) {
        fprintf(stderr, "Usage: %s <IP address> <UDP port> example: ./gnc.out 127.0.0.1 5555\n", av[0]);
        return -1;
    }
    sock.udp_port = atoi(av[2]);
    sock.ip_addr = av[1];
    printf("IP address: %s, UDP port: %d\n", sock.ip_addr, sock.udp_port);
    // Create socket
    sock.sockfd = open_socket();
    // Set up the server address (SITL's IP address and UDP port)

    memset(&sock.autopilot_addr, 0, sizeof(sock.autopilot_addr));
    sock.autopilot_addr.sin_family = AF_INET;
    sock.autopilot_addr.sin_port = htons(sock.udp_port); // Updated port
    sock.autopilot_addr.sin_addr.s_addr = inet_addr(sock.ip_addr); // Updated address
    if (bind(sock.sockfd, (struct sockaddr*)&sock.autopilot_addr, sizeof(sock.autopilot_addr)) < 0) {
        perror("bind");
        close(sock.sockfd);
        return -1;
    }
    struct timeval tv;
    tv.tv_sec = 5;  // 5 seconds timeout
    tv.tv_usec = 0;
    setsockopt(sock.sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    char test_buffer[1024];
    if (recvfrom(sock.sockfd, test_buffer, sizeof(test_buffer), 0, NULL, NULL) < 0) {
            perror("Failed to connect to UAV");
            close(sock.sockfd);
            return -1;
    }
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    setsockopt(sock.sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    printf("Socket bound\n");
    
    //prepare the thread data
    pthread_mutex_init(&thread_data.mutex, NULL);
    thread_data.sock = &sock;
    thread_data.joy = &joy;
    thread_data.sts = &sts;
    thread_data.mavlink_str = &mavlink_str;
    thread_data.gains = &gains;
    //create threads
    if (!pthread_create(&thread[0], NULL, readautopilot_thread, &thread_data))
        {
            printf("Read autopilot thread created successfully.\n");
            if (!pthread_create(&thread[1], NULL, sendautopilot_thread, &thread_data))
            {
                printf("Send autopilot thread created successfully.\n");
                if (!pthread_create(&thread[2], NULL, joystick_thread, &thread_data))
                {
                    printf("Joystick thread created successfully.\n");
                    if (!pthread_create(&thread[3], NULL, gnc_thread, &thread_data))
                        printf("GNC thread created successfully.\n");
                    else
                        {
                            perror("pthread_create");
                            return -1;
                        }
                }
                else
                    {
                        perror("pthread_create");
                        return -1;
                    }
            }
            else
                {
                    perror("pthread_create");
                    return -1;
                }
        }
    else
        {
            perror("pthread_create");
            return -1;
        }
    printf("Press Enter to stop the threads...\n");
    getchar();  // Wait for user input

    // Set stop flag to signal threads to stop
    stop_flag = 1;

    // Wait for threads to finish
    pthread_join(thread[0], NULL);
    pthread_join(thread[1], NULL);
    pthread_join(thread[2], NULL);
    pthread_join(thread[3], NULL);
    //destroy data in mutex
    pthread_mutex_destroy(&thread_data.mutex);

    printf("Tasks completed.\n");
    close(sock.sockfd);
    return 0;
}
