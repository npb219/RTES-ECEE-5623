//assignment_3_2
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <signal.h>

#define DEFAULT_PORT 54321
#define MAX 80 //max buff size

extern int errno;
extern void int_handler();
extern void broken_pipe_handler();
extern void serve_clients();

int byte_count = 0 ;
static int server_sock, client_sock, fromlen;
static struct sockaddr_in server_sockaddr, client_sockaddr;
char sendBuff[1025]; //working buffer for send file

int main(void)
{
    char hostname[64];
    struct hostent *hp;
    struct linger opt;
    int sockarg;

    //get hostname
    gethostname(hostname, sizeof(hostname));

    if((hp = (struct hostent*) gethostbyname(hostname)) == NULL)
    {
        fprintf(stderr, "Error: %s host unknown.\n", hostname);
        exit(-1);
    }

    //get socket
    if((server_sock=socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("Server: socket");
        exit(-1);
    }

    //printf("Socket retrieve success\n");

    memset(&server_sockaddr, '0', sizeof(server_sockaddr));
    memset(sendBuff, '0', sizeof(sendBuff));
    //addr info
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_sockaddr.sin_port = htons(DEFAULT_PORT);

    /* Bind address to the socket */
    if(bind(server_sock, (struct sockaddr *) &server_sockaddr, sizeof(server_sockaddr)) < 0) 
    {
        perror("Server: bind");
        exit(-1);
    }

    /* turn on zero linger time so that undelivered data is discarded when
     socket is closed
    */
    opt.l_onoff = 1;
    opt.l_linger = 0;

    sockarg = 1;

    setsockopt(server_sock, SOL_SOCKET, SO_LINGER, (char*) &opt, sizeof(opt));
    setsockopt(client_sock, SOL_SOCKET, SO_REUSEADDR, (char *)&sockarg, sizeof(int));
    signal(SIGINT, int_handler);
    signal(SIGPIPE, broken_pipe_handler);

    serve_clients();

    return 0;
}   

/* Listen and accept loop function */
void serve_clients()
{
    char buff[MAX];
    /* Read data from file and send it */
    while(1)
    {

        /* Listen on the socket */
        if(listen(server_sock, 5) < 0)
        {
        perror("Server: listen");
        exit(-1);
        }

        /* Accept connections */
        if((client_sock=accept(server_sock, 
                            (struct sockaddr *)&client_sockaddr,
                            &fromlen)) < 0) 
        {
        perror("Server: accept");
        exit(-1);
        }
        //connection open now
        
         
        bzero(buff, MAX); 
        // read the message from client and copy it in buffer 
        read(client_sock, buff, sizeof(buff)); 
        // print file to download
        printf("requested file: %s\n ", buff); 

        /* Open the file that we wish to transfer */
        FILE *fp = fopen(buff,"rb");
        if(fp==NULL)
        {
            fprintf(stderr, "%s", "File open error\n");    
            exit(-1); 
        }   

    
        /* read file in chunks of 256 bytes */
        unsigned char buff[256]={0};
        int nread;
        byte_count = 0;
        
        while( 1 )
        {
            //read
            nread = fread( buff,1,256,fp );
            if( nread < 1 )
                break;

            byte_count +=nread;

            //printf("Bytes read %d \n", nread);        
            buff[nread] = '\0';
            printf("Buffer contents: %s\n", buff);
            //write contents
            write(client_sock, buff, nread);

            //check for end
            if (nread < 256)
            {
                printf("TransferDone: %d bytes\n" ,byte_count);
                if (feof(fp))
                {
                    printf("End of file\n");
                }

                if (ferror(fp))
                {
                    printf("Error reading\n");
                    break;
                }
            }
        }

        shutdown(client_sock, SHUT_WR); // Ensure all data is flushed
        close(client_sock);
        sleep(1);

    } /* end for ever */

}

/* Close sockets after a Ctrl-C interrupt */

void int_handler()
{
  char ch;

  printf("Enter y to close sockets or n to keep open:");
  scanf("%c", &ch);

  if(ch == 'y')
  {
    printf("\nsockets are being closed\n");
    close(client_sock);
    close(server_sock);
  }

  printf("Server: Shutting down ...\n");

  exit(0);

}


void broken_pipe_handler()
{
  char ch;

  printf("Enter y to continue serving clients or n to halt:");
  scanf("%c", &ch);

  if(ch == 'y')
  {
    printf("\nwill continue serving clients\n");
    serve_clients();
  }

  else
  {
    printf("Server: Shutting down ...\n");
    exit(0);
  }

}

