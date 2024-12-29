//assignment_3_2
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

#define LOCAL_PORT 54321
#define MAX 80

// server: ./inet_server
// client: ./inet_client 192.168.86.222 test_file.txt testtest.txt

extern int errno;
extern void broken_pipe_handler();

int main(int argc, char const *argv[])
{
    int client_sock, len;
    struct hostent *hp;
    struct sockaddr_in client_sockaddr;
    struct linger opt;
    int sockarg;
    char buff[MAX];

    int byte_count=0;
    int bytesReceived = 0;
    char recvBuff[256];
    memset(recvBuff, '0', sizeof(recvBuff));

    if(argc < 4)
    {
        printf("Usage: inet_client <server hostname> <filename (host)> <new_filename (client)>\n");
        exit(-1);
    }
    //parse args for filenames and host addr
    char *new_filename = strdup(argv[3]);
    char *server_filename = strdup(argv[2]);
    bzero(buff, sizeof(buff));
    strncpy(buff, server_filename, sizeof(buff) - 1);
    server_filename[sizeof(buff) - 1] = '\0';

    if((hp = gethostbyname(argv[1])) == NULL)
    {
        fprintf(stderr, "Error: %s unknown host.\n", argv[1]);
        exit(-1);
    }
    //get socket
    if((client_sock=socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("client: socket");
        exit(-1);
    }
    //set addr info
    client_sockaddr.sin_family = AF_INET;
    client_sockaddr.sin_port = htons(LOCAL_PORT);
    bcopy(hp->h_addr, &client_sockaddr.sin_addr, hp->h_length);

    /* discard undelivered data on closed socket */ 
    opt.l_onoff = 1;
    opt.l_linger = 0;

    sockarg = 1;

    setsockopt(client_sock, SOL_SOCKET, SO_LINGER, (char*) &opt, sizeof(opt));

    setsockopt(client_sock, SOL_SOCKET, SO_REUSEADDR, (char *)&sockarg, sizeof(int));
    //connect
    if(connect(client_sock, (struct sockaddr*)&client_sockaddr,
        sizeof(client_sockaddr)) < 0) 
    {
        perror("client: connect");
        exit(-1);
    }
    printf("ConnectDone: %s\n", argv[1]);
    signal(SIGPIPE, broken_pipe_handler);
    //send file to download
    write(client_sock, buff, sizeof(buff));

    /* Create file where data will be stored */
    FILE *fp;
    fp = fopen(new_filename, "ab"); 
    if(NULL == fp)
    {
        fprintf(stderr, "%s", "error opening file\n");    
        exit(3);
    }

    /* Receive data in chunks of 256 bytes */
    while((bytesReceived = read(client_sock, recvBuff, 256)) > 0)
    {
        fwrite(recvBuff, 1,bytesReceived,fp);
        byte_count +=bytesReceived;
    }

        printf("FileWritten: %d bytes\n", byte_count);   
    return 0;
}
//broken conn handler
void broken_pipe_handler()
{
  printf("\nbroken pipe signal received\n");
}