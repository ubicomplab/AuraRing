#include "comms.h"
#include <stdio.h>
#include <ws2tcpip.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 512	//Max length of buffer
#define PORT 8888	//The port on which to listen for incoming data
#define PORT_OUT 9884	//The port on which to listen for incoming data


int init_winsock() {
	WSADATA wsa;


	//Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code: %d", WSAGetLastError());
		return -1;
	}
	printf("Initialised.\n");
	return 0;
}

int init_socket(SOCKET& s)
{
	struct sockaddr_in server;
	

	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket: %d", WSAGetLastError());
		return -1;
	}
	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	//Bind
	if (::bind(s, (struct sockaddr *)& server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d\n", WSAGetLastError());
		return -1;
	}
	puts("Bind done");

	return 0;
}

int init_out_socket(SOCKET& s)
{


	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket: %d", WSAGetLastError());
		return -1;
	}
	printf("Socket created.\n");

	return 0;
}

int receive_from_socket(SOCKET s, char* buf)
{
	int slen, recv_len;
	struct sockaddr_in si_other;
	slen = sizeof(si_other);

	//printf("Waiting for data...\n");
	fflush(stdout);

	//clear the buffer by filling null, it might have previously received data
	memset(buf, 0, BUFLEN);

	//try to receive some data, this is a blocking call
	if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) & si_other, &slen)) == SOCKET_ERROR)
	{
		printf(" recvfrom() failed with error code : %d\n", WSAGetLastError());
		return -1;
	}

	//print details of the client/peer and the data received
	//char ipbuf[INET_ADDRSTRLEN];
	//inet_ntop(AF_INET, &si_other.sin_addr, ipbuf, sizeof(ipbuf));
	//printf(" Received packet from %s:%d\n", ipbuf, ntohs(si_other.sin_port));
	//printf(" Data: %s\n", buf);


	return recv_len;
}

int get_sensor_data_from_socket(SOCKET s, sensor_data& data)
{
	char buf[BUFLEN];
	int size = receive_from_socket(s, buf);
	if (size < 0) {
		return -1;
	}

	for (int i = 0; i < 9; i++) {
		data[i] = *((float*)(buf) + i);
	}
	
	return 0;
}

int send_solution_to_socket(SOCKET s, const double* buffer, int len_bytes) {


	struct sockaddr_in si_other;
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT_OUT);
	inet_pton(AF_INET, "127.0.0.1", &si_other.sin_addr.S_un.S_addr);
	
	if (sendto(s, (char*)buffer, len_bytes, 0, (struct sockaddr*)& si_other, sizeof(si_other)) == SOCKET_ERROR)
	{
		printf("sendto() failed with error code : %d\n", WSAGetLastError());
		return -1;
	}
	return 0;
}

void close_socket(SOCKET s)
{
	closesocket(s);
	WSACleanup();

	return;
}