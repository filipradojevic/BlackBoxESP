#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib") // Visual Studio, ne obavezno u GCC/MSYS

#define UDP_PORT 1002
#define BUFFER_SIZE 1024

int main() {
    WSADATA wsa;
    SOCKET sock;
    struct sockaddr_in server_addr, client_addr;
    char buffer[BUFFER_SIZE];
    int recv_len;
    int client_len = sizeof(client_addr);

    // 1. Inicijalizacija WinSock-a
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        printf("Greška pri inicijalizaciji WinSock-a: %d\n", WSAGetLastError());
        return 1;
    }

    // 2. Kreiranje UDP socketa
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        printf("Greška pri kreiranju socketa: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // 3. Bind na lokalnu IP i port
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        printf("Bind failed: %d\n", WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    printf("UDP echo server pokrenut na portu %d\n", UDP_PORT);

    // 4. Glavna petlja
    while (1) {
        recv_len = recvfrom(sock, buffer, BUFFER_SIZE, 0,
                            (struct sockaddr*)&client_addr, &client_len);
        if (recv_len == SOCKET_ERROR) {
            printf("recvfrom() failed: %d\n", WSAGetLastError());
            break;
        }

        // Echo nazad
        sendto(sock, buffer, recv_len, 0,
               (struct sockaddr*)&client_addr, client_len);
    }

    // Cleanup
    closesocket(sock);
    WSACleanup();
    return 0;
}
