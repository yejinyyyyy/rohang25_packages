
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define RECV_BUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.144.25" // ZR10 Gimbal Default IP

int main() {
    int sockfd;
    int recv_len;
    struct sockaddr_in send_addr, recv_addr;
    unsigned char send_buf[] = {
        0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0xff, 0xa6, 0x3b, 0x11 
    }; // Gimbal -90
    unsigned char recv_buf[RECV_BUF_SIZE] = {0};

    // 1. Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        exit(1);
    }

    // 2. Set destination (ZR10)
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(SERVER_PORT);
    send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // 3. Send data
    printf("✅ Sending gimbal center command...\n");
    socklen_t addr_len = sizeof(struct sockaddr_in);
    if (sendto(sockfd, send_buf, sizeof(send_buf), 0,
               (struct sockaddr *)&send_addr, addr_len) < 0) {
        perror("sendto");
        close(sockfd);
        exit(1);
    }

    // 4. Receive response
    recv_len = recvfrom(sockfd, recv_buf, RECV_BUF_SIZE, 0,
                        (struct sockaddr *)&recv_addr, &addr_len);
    if (recv_len < 0) {
        perror("recvfrom");
        close(sockfd);
        exit(1);
    }

    printf("✅ Received HEX data (%d bytes): ", recv_len);
    for (int i = 0; i < recv_len; i++) {
        printf("%02x ", recv_buf[i]);
    }
    printf("\n");

    // 5. Cleanup
    close(sockfd);
    return 0;
}
