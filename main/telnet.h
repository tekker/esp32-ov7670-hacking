
struct telnetUserData_t;
typedef struct telnetUserData telnetUserData_t;

struct telnetUserData {
	int sockfd;
};

//void telnet_esp32_listenForClients(void (*callbackParam)(uint8_t *buffer, size_t size, telnetUserData_t *telnetClient));
void telnet_esp32_listenForClients(void (*dataCallbackParam)(uint8_t *buffer, size_t size, telnetUserData_t *telnetClient)
								 ,void (*connectCallbackParam)(bool connectOrDisconnect, telnetUserData_t *telnetClient));
void telnet_esp32_sendData(uint8_t *buffer, size_t size);
int telnet_esp32_vprintf(const char *fmt, va_list va);
