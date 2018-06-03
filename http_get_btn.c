/* http_get - Retrieves a web page over HTTP GET.
 *
 * See http_get_ssl for a TLS-enabled version.
 *
 * This sample code is in the public domain.,
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <unistd.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ssid_config.h"

#define WEB_SERVER "192.168.1.105"
#define WEB_PORT 8000
#define WEB_PATH "/"

/* pin config */
const int gpio = 0;   /* gpio 0 usually has "PROGRAM" button attached */
const int active = 0; /* active == 0 for active low */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;
int flag = 0;
int successes = 0, failures = 0;


void buttonPollTask(void *pvParameters)
{
    printf("Polling for button press on gpio %d...\r\n", gpio);
    while(1) {
        while(gpio_read(gpio) != active)
        {
            taskYIELD();
        }
        printf("Polled for button press at %dms\r\n", xTaskGetTickCount()*portTICK_PERIOD_MS);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void gpio_intr_handler(uint8_t gpio_num);

static QueueHandle_t tsqueue;

void gpio_intr_handler(uint8_t gpio_num)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}



void http_get_task()
{
        printf("HTTP get task starting...\r\n");
        const struct addrinfo hints = {
            .ai_family = AF_UNSPEC,
            .ai_socktype = SOCK_STREAM,
        };
        struct addrinfo *res;

        printf("Running DNS lookup for %s...\r\n", WEB_SERVER);
        int err = getaddrinfo(WEB_SERVER, "8000", &hints, &res);

        if (err != 0 || res == NULL) {
            printf("DNS lookup failed err=%d res=%p\r\n", err, res);
            if(res)
                freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            //failures++;
            //continue;
        }

#if LWIP_IPV6
        {
            struct netif *netif = sdk_system_get_netif(0);
            int i;
            for (i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
                printf("  ip6 %d state %x\n", i, netif_ip6_addr_state(netif, i));
                if (!ip6_addr_isinvalid(netif_ip6_addr_state(netif, i)))
                    printf("  ip6 addr %d = %s\n", i, ip6addr_ntoa(netif_ip6_addr(netif, i)));
            }
        }
#endif

        struct sockaddr *sa = res->ai_addr;
        if (sa->sa_family == AF_INET) {
            printf("DNS lookup succeeded. IP=%s\r\n", inet_ntoa(((struct sockaddr_in *)sa)->sin_addr));
        }
#if LWIP_IPV6
        if (sa->sa_family == AF_INET6) {
            printf("DNS lookup succeeded. IP=%s\r\n", inet6_ntoa(((struct sockaddr_in6 *)sa)->sin6_addr));
        }
#endif

        int s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            printf("... Failed to allocate socket.\r\n");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            failures++;
            //continue;
        }

        printf("... allocated socket\r\n");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            close(s);
            freeaddrinfo(res);
            printf("... socket connect failed.\r\n");
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            failures++;
            //continue;
        }

        printf("... connected\r\n");
        freeaddrinfo(res);

        const char *req =
            "GET "WEB_PATH" HTTP/1.1\r\n"
            "Host: "WEB_SERVER"\r\n"
            "User-Agent: esp-open-rtos/0.1 esp8266\r\n"
            "Connection: close\r\n"
            "\r\n";
        if (write(s, req, strlen(req)) < 0) {
            printf("... socket send failed\r\n");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            failures++;
            //continue;
        }
        printf("... socket send success\r\n");

        static char recv_buf[128];
        int r;
        do {
            bzero(recv_buf, 128);
            r = read(s, recv_buf, 127);
            if(r > 0) {
                printf("%s", recv_buf);
            }
        } while(r > 0);

        printf("... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        if(r != 0)
            failures++;
        else
            successes++;
        close(s);
        printf("successes = %d failures = %d\r\n", successes, failures);
        for(int countdown = 10; countdown >= 0; countdown--) {
            printf("%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("\r\nStarting again!\r\n");
}

/* This task configures the GPIO interrupt and uses it to tell
   when the button is pressed.

   The interrupt handler communicates the exact button press time to
   the task via a queue.

   This is a better example of how to wait for button input!
*/
void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", gpio);
    QueueHandle_t *tsqueue = (QueueHandle_t *)pvParameters;
    gpio_set_interrupt(gpio, int_type, gpio_intr_handler);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_PERIOD_MS;
        printf("Button interrupt not occured\r\n");

        if(last < button_ts-200) {
            printf("Button interrupt fired at %dms\r\n", button_ts);
            flag = 1;
            http_get_task();
            last = button_ts;
        }
    }
}



void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    printf("ssid: %s\n",WIFI_SSID);
    printf("pass: %s\n",WIFI_PASS);
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    gpio_enable(gpio, GPIO_INPUT);

    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 384, &tsqueue, 2, NULL);
    //xTaskCreate(&http_get_task, "get_task", 384, NULL, 2, NULL);
}

