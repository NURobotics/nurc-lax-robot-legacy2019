#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <iostream>
//#include <netinet/in.h>
// arpa/inet.h might work on both MAC and Linux OSes
#include <arpa/inet.h>

#define PORT 8080 // NOTE: Might need to redefine if its already in use
#define MSG_LEN 1024 // Buffer size

using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

// TODO: have a predefined port number instead of passing through argc
int main(int argc, char *argv[])
{
    // sockfd is listening for different connections
    // newsockfd is for handling currect connection
    int sockfd, newsockfd;
    socklen_t clilen;
    int opt = 1; // enables the option

    // buffer is used to read messages coming from client
    char buffer[MSG_LEN];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    // create a socket
    // socket(int domain, int type, int protocol)
    sockfd =  socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");

    // add socket option so it reuses the same address.
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    // clear address structure. Prepping serv address
    bzero((char *) &serv_addr, sizeof(serv_addr));

    /* setup the host_addr structure for use in bind call */
    // server byte order
    serv_addr.sin_family = AF_INET;
    // automatically be filled with current host's IP address
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    // convert short integer value for port must be converted into network byte order
    serv_addr.sin_port = htons(PORT);

    // bind(int fd, struct sockaddr *local_addr, socklen_t addr_length)
    // bind() passes file descriptor, the address structure,
    // and the length of the address structure
    // This bind() call will bind the socket to the current IP address on port, PORT
    if (bind(sockfd,
                (struct sockaddr *) &serv_addr,
                sizeof(serv_addr)) < 0)
        error("ERROR on binding");

    // This listen() call tells the socket to listen to the incoming connections.
    // The listen() function places all incoming connection into a backlog queue
    // until accept() call accepts the connection.
    // Here, we set the maximum size for the backlog queue to 5.
    listen(sockfd,5); // TODO: look at the necessary queue size. What happens if queue is full?

    // The accept() call actually accepts an incoming connection
    clilen = sizeof(cli_addr);

    // This accept() function will write the connecting client's address info
    // into the the address structure and the size of that structure is clilen.
    // The accept() returns a new socket file descriptor for the accepted connection.
    // So, the original socket file descriptor can continue to be used
    // for accepting new connections while the new socker file descriptor is used for
    // communicating with the connected client.
    newsockfd = accept(sockfd,
            (struct sockaddr *) &cli_addr,
            &clilen
            );
    if (newsockfd < 0)
        error("ERROR on accept");

    printf("server: got connection from %s port %d\n",
            inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));

    // This send() function sends the 13 bytes of the string to the new socket
    send(newsockfd, "Hello, world!\n", 13, 0);

    // continuously waits for message from socket until connection is closed by either
    // client or server
    while(1) {
        // clears buffer. Could also use memset
        bzero(buffer, MSG_LEN);
        // TODO: read the buffer information. loop if incoming message is larger
        // than the buffer size
        n = read(newsockfd, buffer, MSG_LEN);

        if (n < 0) error("ERROR reading from socket");

        if (!strcmp(buffer, "exit")) {
            cout << "Client has quit the session" << endl;
            break;
        }

        cout << "Client: " << buffer << endl;
        cout << "Input your message >";

        string data;
        getline(cin, data);
        bzero(buffer, MSG_LEN);
        strcpy(buffer, data.c_str());
        if(data == "exit") {
            //notify client that server has closed the connection
            send(newsockfd, (char*)&buffer, strlen(buffer), 0);
            break;
        }
    }


    // TODO: close client socket connection only when explicitly told or the web
    // app is closed
    close(newsockfd);
    // TODO: close listening fd when closing/turning off the server
    close(sockfd);
    return 0;
};
