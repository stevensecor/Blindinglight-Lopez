#include "udplog.h"

#define LOGGING_ENABLED 1

#include <sockLib.h>
#include <netinet/in.h>
#include <inetLib.h>
#include <stdio.h>
#include <string.h>
#include "Synchronized.h"

const int PORT = 1140;// competition legal (see FMS Whitepaper)
const char* DS_IP = "10.15.11.5";
const char* CODER_IP = "10.15.11.53";
const char* TARGET_IP = CODER_IP;

bool ready = false;
int sock_fd = 0;
struct sockaddr_in saddr;
SEM_ID semaphore = 0;

const int BUFLEN = 1024;
char buf[BUFLEN];

void UDPLog::log(const char *fmt, ...) {
#if LOGGING_ENABLED
	if (!ready) {
		printf("UDP Logger not initialized\n");
		return;
	}

	{
		Synchronized sync(semaphore);

		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, BUFLEN, fmt, args);
		va_end(args);

		// we don't care if it worked or not (target present)
		sendto(sock_fd, buf, strlen(buf), 0, (sockaddr *) &saddr, sizeof(saddr));
	}
#endif
}

void UDPLog::setup() {
	semaphore = semMCreate(SEM_Q_PRIORITY);

	memset(&saddr, 0, sizeof(saddr));
	saddr.sin_family = AF_INET;
	saddr.sin_port = htons(PORT);

	if (inet_aton((char*) TARGET_IP, &saddr.sin_addr) == ERROR) {
		printf("UDPLogger: Bad inet_aton\n");
		return;
	}
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock_fd == ERROR) {
		printf("UDPLogger: Bad socket\n");
		// cleanup
		semFlush(semaphore);
		return;
	}

	ready = true;
}

void UDPLog::destroy() {
	semFlush(semaphore);

	shutdown(sock_fd, SHUT_RDWR);

	ready = false;
}
