/********************************************************
Autho:Song Jinbo
Data:2017-04-05
Description:1:This is just a test for socket programming
	    2:It doesn't get the guidance's data.
*********************************************************/
#include <netinet/in.h>    // for sockaddr_in
#include <unistd.h>
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <string.h>        // for bzero
#include <arpa/inet.h>
 
#define BUFFER_SIZE 1024
#define FILE_NAME_MAX_SIZE 512

int main()
{
	int client_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (client_socket < 0)
	{
		printf("Create Socket Failed!\n");
		exit(1);
	}

	struct sockaddr_in server_addr;
	bzero(&server_addr, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr("192.168.1.142");
	server_addr.sin_port = htons(6001);
	
	if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
	{
		printf("Can Not Connect!\n");
		exit(1);
	}

	char recvBuffer[100];
	char sendBuffer[100];

	recv(client_socket, recvBuffer, 100, 0);

	printf("%s\n", recvBuffer);

	sprintf(sendBuffer, "This is Client!\n");
	send(client_socket, sendBuffer, 100, 0);

	close(client_socket);

	return 0;
}
