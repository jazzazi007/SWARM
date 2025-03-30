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
#include "../include/swarm.h"

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

static void connect_udp(sockport *sock, char **av)
{
    for (int i; i < 3; i++)
    {
        sock->udp_port[i] = atoi(av[i + 1]);
        sock->ip_addr[i] = "127.0.0.1";
        printf("IP address: %s, UDP port: %d\n", sock->ip_addr, sock->udp_port);
        // Create socket
        sock->sockfd[i] = open_socket();
        // Set up the server address (SITL's IP address and UDP port)
    
        memset(&sock->autopilot_addr[i], 0, sizeof(sock->autopilot_addr[i]));
        sock->autopilot_addr[i].sin_family = AF_INET;
        sock->autopilot_addr[i].sin_port = htons(sock->udp_port[i]); // Updated port
        sock->autopilot_addr[i].sin_addr.s_addr = inet_addr(sock->ip_addr[i]); // Updated address
        if (bind(sock->sockfd[i], (struct sockaddr*)&sock->autopilot_addr[i], sizeof(sock->autopilot_addr[i])) < 0) {
            perror("bind");
            close(sock->sockfd[i]);
            exit(1);
        }
        struct timeval tv;
        tv.tv_sec = 5;  // 5 seconds timeout
        tv.tv_usec = 0;
        setsockopt(sock->sockfd[i], SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        char test_buffer[1024];
        if (recvfrom(sock->sockfd[i], test_buffer, sizeof(test_buffer), 0, NULL, NULL) < 0) {
                perror("Failed to connect to UAV");
                close(sock->sockfd[i]);
                exit(1);
        }
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        setsockopt(sock->sockfd[i], SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        printf("Socket bound\n");   
    }
    exit(0);
}

int main(int ac, char **av) {
    sockport sock;
    sts sts;
    mavlink_str mavlink_str;
    gains gains;


    memset(&sock, 0, sizeof(sock));  // Initialize memory to zero
    memset(&sts, 0, sizeof(sts));  // Initialize memory to zero
    joy_s joy;
    joy.pitch = 1500;
    joy.roll = 1500;
    joy.throttle = 1500;
    joy.yaw = 1500;


    if (ac != 3) {
        fprintf(stderr, "Usage: %s <IP address> <UDP port> example: ./gnc.out 127.0.0.1 5555\n", av[0]);
        return -1;
    }
    // Initialize the socket
    connect_udp(&sock, av);
    while (1)
    {

    }
    printf("Tasks completed.\n");
    close(sock.sockfd);
    return 0;
}
