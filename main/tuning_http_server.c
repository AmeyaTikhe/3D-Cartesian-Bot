#include "tuning_http_server.h"
#include "cJSON.h"
#include "esp_http_server.h"

#define MAX_COORDINATES 100  // Maximum number of coordinates to store
static int coordinates[MAX_COORDINATES][2];  // 2D array to store coordinates (x, y)
static int coord_index = 0;  // Index to keep track of the current position in the array


static const char *TAG = "websocket_server";

static comms_val_t comms_val = { .x_coord = 0, .y_coord = 0, .z_coord = 0, .val_changed = false, .esp_ack = false  };

static QueueHandle_t client_queue;
const static int client_queue_size = 10;

static void initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set(MDNS_HOST_NAME);
    mdns_instance_name_set(MDNS_INSTANCE);

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}};

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

//websocket callback function

void websocket_callback(uint8_t num, WEBSOCKET_TYPE_t type, char *msg, uint64_t len)
{
    switch (type)
    {
    case WEBSOCKET_CONNECT:
        ESP_LOGI(TAG, "client %i connected!", num);
        break;
    case WEBSOCKET_DISCONNECT_EXTERNAL:
        ESP_LOGI(TAG, "client %i sent a disconnect message", num);
        break;
    case WEBSOCKET_DISCONNECT_INTERNAL:
        ESP_LOGI(TAG, "client %i was disconnected", num);
        break;
    case WEBSOCKET_DISCONNECT_ERROR:
        ESP_LOGI(TAG, "client %i was disconnected due to an error", num);
        break;
    case WEBSOCKET_TEXT:
        if (len)
        { 
            // Print received message for debugging
            ESP_LOGI(TAG, "Received message: %.*s", (int)len, msg);

            // Parse coordinates in format "x1,y1;x2,y2;x3,y3"
            char *message_copy = malloc(len + 1); // +1 for null terminator
            if (message_copy == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for message copy");
                break;
            }
            
            // Copy the message to our buffer and ensure null-termination
            memcpy(message_copy, msg, len);
            message_copy[len] = '\0';
            
            char *saveptr1, *saveptr2;
            char *pair = strtok_r(message_copy, ";", &saveptr1); // Split by semicolon
            
            // Reset coordinate index before processing new batch
            coord_index = 0;
            
            // Process each coordinate pair (x,y)
            while (pair != NULL && coord_index < MAX_COORDINATES) {
                // Split pair by comma to get x and y
                char *x_str = strtok_r(pair, ",", &saveptr2);
                if (x_str != NULL) {
                    char *y_str = strtok_r(NULL, ",", &saveptr2);
                    if (y_str != NULL) {
                        int x = atoi(x_str);
                        int y = atoi(y_str);
                        
                        // Store the coordinates
                        coordinates[coord_index][0] = x;
                        coordinates[coord_index][1] = y;
                        coord_index++;
                        
                        ESP_LOGI(TAG, "Parsed coordinate pair: x=%d, y=%d", x, y);
                        
                        // Save the most recent coordinate to comms_val (if needed)
                        comms_val.x_coord = x;
                        comms_val.y_coord = y;
                        comms_val.val_changed = true;
                    }
                }
                
                // Get next coordinate pair
                pair = strtok_r(NULL, ";", &saveptr1);
            }
            
            // Send acknowledgment back
            char ack_msg[64];
            snprintf(ack_msg, sizeof(ack_msg), "{\"status\":\"received\",\"count\":%d}", coord_index);
            ws_server_send_text_all(ack_msg, strlen(ack_msg));
            
            ESP_LOGI(TAG, "Processed %d coordinate pairs", coord_index);
            
            // Free allocated memory
            free(message_copy);
        }
        break;
    case WEBSOCKET_BIN:
        ESP_LOGI(TAG, "client %i sent binary message of size %llu:\n%s", num, (unsigned long long)len, msg);
        break;
    case WEBSOCKET_PING:
        ESP_LOGI(TAG, "client %i pinged us with message of size %llu:\n%s", num, (unsigned long long)len, msg);
        break;
    case WEBSOCKET_PONG:
        ESP_LOGI(TAG, "client %i responded to the ping", num);
        break;
    }
}

static void http_server(struct netconn *conn)
{
    const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";

    struct netbuf *inbuf;
    static char *buf;
    static uint16_t buflen;
    static err_t err;

    // default page
    extern const uint8_t root_html_start[] asm("_binary_index_html_start");
    extern const uint8_t root_html_end[] asm("_binary_index_html_end");
    const uint32_t root_html_len = root_html_end - root_html_start;

    netconn_set_recvtimeout(conn, 10000); // allow a connection timeout of 1 second
    ESP_LOGI(TAG, "reading from client...");
    err = netconn_recv(conn, &inbuf);
    ESP_LOGI(TAG, "read from client");

    if (err == ERR_OK)
    {
        netbuf_data(inbuf, (void **)&buf, &buflen);

        if (buf && buflen > 0)
        {
            // Print full request for debugging
            ESP_LOGI(TAG, "Incoming request:\n%.*s", buflen, buf);

            if (strstr(buf, "Upgrade: websocket"))
            {
                // WebSocket upgrade request
                ESP_LOGI(TAG, "WebSocket upgrade requested.");
                ws_server_add_client(conn, buf, buflen, "/", websocket_callback);
                netbuf_delete(inbuf);
            }
            else if (strstr(buf, "GET /"))
            {
                // Serve the default page
                ESP_LOGI(TAG, "Serving root HTML page.");
                netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
                netconn_write(conn, root_html_start, root_html_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
            else
            {
                ESP_LOGI(TAG, "Unknown request:\n%.*s", buflen, buf);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
        }
        else
        {
            ESP_LOGI(TAG, "Unknown request (empty buffer?)");
            netconn_close(conn);
            netconn_delete(conn);
            netbuf_delete(inbuf);
        }
    }
    else
    {
        ESP_LOGI(TAG, "error on read, closing connection");
        netconn_close(conn);
        netconn_delete(conn);
        if (inbuf)
            netbuf_delete(inbuf);
    } 
}

static void server_task(void *pvParameters)
{
    // const static char *TAG = "server_task";
    struct netconn *conn, *newconn;
    static err_t err;
    client_queue = xQueueCreate(client_queue_size, sizeof(struct netconn *));

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    ESP_LOGI(TAG, "server listening");
    do
    {
        err = netconn_accept(conn, &newconn);
        ESP_LOGI(TAG, "new client");
        if (err == ERR_OK)
        {
            xQueueSendToBack(client_queue, &newconn, portMAX_DELAY);
            // http_serve(newconn);
        }
        vTaskDelay(10);
    } while (err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
    ESP_LOGE(TAG, "task ending, rebooting board");
    esp_restart();
}

// receives clients from queue, handles them
static void server_handle_task(void *pvParameters)
{
    // const static char *TAG = "server_handle_task";
    struct netconn *conn;
    ESP_LOGI(TAG, "task starting");
    for (;;)
    {
        xQueueReceive(client_queue, &conn, portMAX_DELAY);
        if (!conn)
            continue;
        http_server(conn);
        vTaskDelay(10);
    }
    vTaskDelete(NULL);
}

comms_val_t read_comms()
{
    return comms_val;
}

void reset_val_changed_coms(){
    comms_val.val_changed = false;
}

void start_websocket_server()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(MDNS_HOST_NAME);

    connect_to_wifi();

    ws_server_start();
    xTaskCreate(&server_task, "server_task", 3000, NULL, 9, NULL);
    xTaskCreate(&server_handle_task, "server_handle_task", 4000, NULL, 6, NULL);
}