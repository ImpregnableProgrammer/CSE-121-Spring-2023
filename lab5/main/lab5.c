/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

#include "esp_tls.h" // For verifying server identity over TLS: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/protocols/esp_tls.html#tls-server-verification
#include "esp_crt_bundle.h" // For CA certificate bundle access included with esp-idf: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/protocols/esp_crt_bundle.html

#include "sensor.h"

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "Rohan-iPhone-14"
#define EXAMPLE_ESP_WIFI_PASS      "rohanPhone"
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
    
static const char *TAG = "lab5_wifi_http";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// Initialize connection to server and return socket fd
// Returns -1 for errors
// This is partly taken from the example here: https://github.com/espressif/esp-idf/blob/54576b7528b182256c027de86eb605a172bc2821/examples/protocols/http_request/main/http_request_example_main.c
int init_connection(const char *SERVER, const char *PORT) {
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s;

    int err = getaddrinfo(SERVER, PORT, &hints, &res);

    if(err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
        return -1;
    }

    /* Code to print the resolved IP.

        Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
    addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
    ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

    s = socket(res->ai_family, res->ai_socktype, 0);
    if(s < 0) {
        ESP_LOGE(TAG, "... Failed to allocate socket.");
        freeaddrinfo(res);
        return -1;
    }
    ESP_LOGI(TAG, "... allocated socket");

    if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
        close(s);
        freeaddrinfo(res);
        return -1;
    }

    ESP_LOGI(TAG, "... connected");
    freeaddrinfo(res);

    return s;
}

// Send an HTTP GET request to the server given
// and write the response into the `buf` given
static void http_get_request(const char *SERVER, const char *PORT, const char *REQUEST, char* buf, const size_t buf_len)
{   
    // Create socket connection to server
    int s = init_connection(SERVER, PORT);
    if (s < 0) {
        ESP_LOGE(TAG, "socket connection failed");
        return;
    }

    // Write GET request to server
    if (write(s, REQUEST, strlen(REQUEST)) < 0) {
        ESP_LOGE(TAG, "... socket send failed");
        close(s);
        return;
    }
    ESP_LOGI(TAG, "... socket send success");

    // Set receving timeout for socket
    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
            sizeof(receiving_timeout)) < 0) {
        ESP_LOGE(TAG, "... failed to set socket receiving timeout");
        close(s);
        return;
    }
    ESP_LOGI(TAG, "... set socket receiving timeout success");

    // Write HTTP response to buffer given
    // discarding the rest of the response if the buffer isn't large enough
    int r = read(s, buf, buf_len - 1);
    if (r < 0) {
        ESP_LOGE(TAG, "error reading response from socket");
        close(s);
        return;
    }

    ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
    close(s);
}

// Send an HTTP POST request to the server given
// and write the response to stdout
static void http_post_request(const char *SERVER, const char *PORT, const char *REQUEST)
{
    int s = init_connection(SERVER, PORT);
    if (s < 0) {
        ESP_LOGE(TAG, "socket connection failed");
        return;
    }

    // Send POST request to server
    if (write(s, REQUEST, strlen(REQUEST)) < 0) {
        ESP_LOGE(TAG, "... socket send failed");
        close(s);
        return;
    }
    ESP_LOGI(TAG, "... socket send success");

    // Set timeout for receving server response
    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
            sizeof(receiving_timeout)) < 0) {
        ESP_LOGE(TAG, "... failed to set socket receiving timeout");
        close(s);
        return;
    }
    ESP_LOGI(TAG, "... set socket receiving timeout success");

    // Write HTTP POST response to STDOUT
    ESP_LOGI(TAG, "...writing HTTP POST response");
    char buf[512];
    int r;
    do {
        bzero(buf, sizeof(buf));
        r = read(s, buf, sizeof(buf)-1);
        for (int i = 0; i < r; i++) {
            putchar(buf[i]);
        }
    } while(r > 0);

    ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
    close(s);
}

// This performs an HTTPS GET request using the CA certificate bundle supplied with esp-idf
// to verify the server identity over TLS for the URL given
// This is taken and modified from the example here: https://github.com/espressif/esp-idf/blob/54576b7528b182256c027de86eb605a172bc2821/examples/protocols/https_request/main/https_request_example_main.c#L257
// The response is written into the buffer given
static void https_get_request(const char* URL, const char *REQUEST, char* buf, size_t buf_len)
{   
    ESP_LOGI(TAG, "https_request using crt bundle");
    esp_tls_cfg_t cfg = {
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_tls_t *tls = esp_tls_init();
    if (!tls) {
        ESP_LOGE(TAG, "Failed to allocate esp_tls handle!");
        return;
    }

    // Establish TLS connection with server
    if (esp_tls_conn_http_new_sync(URL, &cfg, tls) == 1) {
        ESP_LOGI(TAG, "Connection established...");
    } else {
        ESP_LOGE(TAG, "Connection failed...");
        esp_tls_conn_destroy(tls);
    }

    // Send request to server
    size_t written_bytes = 0;
    do {
        int ret = esp_tls_conn_write(tls,
                                 REQUEST + written_bytes,
                                 strlen(REQUEST) - written_bytes);
        if (ret >= 0) {
            ESP_LOGI(TAG, "%d bytes written", ret);
            written_bytes += ret;
        } else if (ret != ESP_TLS_ERR_SSL_WANT_READ && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(TAG, "esp_tls_conn_write  returned: [0x%02X](%s)", ret, esp_err_to_name(ret));
            esp_tls_conn_destroy(tls);
        }
    } while (written_bytes < strlen(REQUEST));

    // Read response from server into the buffer given
    // discarding the part of the response exceeding the buffer length
    ESP_LOGI(TAG, "Reading HTTP response...");
    int ret = esp_tls_conn_read(tls, (char *)buf, buf_len - 1);
    if (ret < 0) {
        ESP_LOGE(TAG, "esp_tls_conn_read returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
        return;
    }

    /*do {
        len = buf_len - 1;
        memset(buf, 0x00, buf_len);
        ret = esp_tls_conn_read(tls, (char *)buf, len);

        if (ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ) {
            continue;
        } else if (ret < 0) {
            ESP_LOGE(TAG, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
            break;
        } else if (ret == 0) {
            ESP_LOGI(TAG, "connection closed");
            break;
        }

        ESP_LOGD(TAG, "%d bytes read", ret);
        // Print response directly to stdout as it is read
        for (int i = 0; i < len; i++) {
            putchar(buf[i]);
        }
        putchar('\n'); // JSON output doesn't have a newline at end
    } while (1); */
}

static void https_request_task(void *pvParameters) 
{
    const char
        *PHONE_SERVER = "Rohan-iPhone-14.local",
        *PHONE_PORT = "8080",
        *LOCATION_REQUEST = "GET /root/location HTTP/1.0";

    char LOCATION[16], WTTR[256], WTTR_REQUEST[100], CONTENT[256 + 64], POST_REQUEST[256 + 128];

    // Get location from phone and create wttr.in request
    http_get_request(PHONE_SERVER, PHONE_PORT, LOCATION_REQUEST, LOCATION, sizeof(LOCATION));
    snprintf(WTTR_REQUEST, sizeof(WTTR_REQUEST), "GET /%s?0&Q&A&T HTTP/1.1\n"
        "Host: www.wttr.in:443\n"
        "User-Agent: esp-idf/1.0 esp32\n\n", LOCATION);

    int temperature = 0, humidity = 0;
    while (1) {
        https_get_request("https://www.wttr.in/", WTTR_REQUEST, WTTR, sizeof(WTTR));
        read_temperature_and_humidity(&temperature, &humidity);
        snprintf(CONTENT, sizeof(CONTENT), "%s\nSensor temp: %d\nSensor humidity: %d\n", WTTR, temperature, humidity);
        snprintf(POST_REQUEST, sizeof(POST_REQUEST), "POST / HTTP/1.0\nContent-Length: %d\n\n%s", strlen(CONTENT), CONTENT);
        http_post_request(PHONE_SERVER, PHONE_PORT, POST_REQUEST);
        ESP_LOGI(TAG, "Done!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    i2c_master_init();
    xTaskCreate(&https_request_task, "https request", 4096, NULL, 1, NULL); // "1" is task priority level
}
