#include "tuning_http_server.h"
#include "cJSON.h"
#include "esp_http_server.h"
// #include <inttypes.h>

// #define MIN(a,b) (((a)<(b))?(a):(b))
// #define MAX(a,b) (((a)>(b))?(a):(b))

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
            
            // Parse coordinates format from index.html: "Cx,y"
            if (msg[0] == 'C' && len > 1)
            {
                // Find the comma that separates x and y values
                char *comma = strchr(&msg[1], ',');
                if (comma != NULL)
                {
                    // Extract x coordinate
                    *comma = '\0'; // Temporarily replace comma with null terminator
                    int x = atoi(&msg[1]);
                    
                    // Extract y coordinate
                    int y = atoi(comma + 1);
                    
                    // Restore comma
                    *comma = ',';
                    
                    // Save to comms_val
                    comms_val.x_coord = x;
                    comms_val.y_coord = y;
                    comms_val.val_changed = true;
                    
                    // Add to coordinates array
                    if (coord_index < MAX_COORDINATES)
                    {
                        coordinates[coord_index][0] = x;
                        coordinates[coord_index][1] = y;
                        coord_index++;
                    }
                    
                    ESP_LOGI(TAG, "Saved coordinates: x=%d, y=%d", x, y);
                    
                    // Send acknowledgment back if needed
                    const char *ack_msg = "{\"status\":\"received\"}";
                    ws_server_send_text_all((char *)ack_msg, strlen(ack_msg));
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid coordinate format: %s", msg);
                }
            }
            // Handle other message types if needed
            else
            {
                ESP_LOGI(TAG, "Unhandled message: %.*s", (int)len, msg);
            }    
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

    netconn_set_recvtimeout(conn, 1000); // allow a connection timeout of 1 second
    ESP_LOGI(TAG, "reading from client...");
    err = netconn_recv(conn, &inbuf);
    ESP_LOGI(TAG, "read from client");
    if (err == ERR_OK)
    {
        netbuf_data(inbuf, (void **)&buf, &buflen);
        if (buf)
        {
            // default page
            if (strstr(buf, "GET / ") && !strstr(buf, "Upgrade: websocket"))
            {
                ESP_LOGI(TAG, "Sending /");
                netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
                netconn_write(conn, root_html_start, root_html_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }

            // default page websocket
            else if (strstr(buf, "GET / ") && strstr(buf, "Upgrade: websocket"))
            {
                ESP_LOGI(TAG, "Requesting websocket on /");
                ws_server_add_client(conn, buf, buflen, "/", websocket_callback);
                netbuf_delete(inbuf);
            }

            else
            {
                ESP_LOGI(TAG, "Unknown request");
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
        }
        else
        {
            ESP_LOGI(TAG, "Unknown request (empty?...)");
            netconn_close(conn);
            netconn_delete(conn);
            netbuf_delete(inbuf);
        }
    }
    else
    { // if err==ERR_OK
        ESP_LOGI(TAG, "error on read, closing connection");
        netconn_close(conn);
        netconn_delete(conn);
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

// esp_err_t coords_post_handler(httpd_req_t *req)
// {
//     static char content[256];  // Buffer to hold incoming JSON
//     int total_len = req->content_len;
//     int received = 0;

//     while (received < total_len) {
//         int ret = httpd_req_recv(req, content + received, total_len - received);
//         if (ret <= 0) {
//             return ESP_FAIL;
//         }
//         received += ret;
//     }
//     content[received] = '\0'; // Null-terminate for string handling

//     ESP_LOGI(TAG, "Raw coordinate data received: %s", content);

//     // Send the raw JSON string to your own function in stepper_motor_ws_v2.c
//     handle_raw_coordinates(content); // you will implement this

//     httpd_resp_sendstr(req, "Coordinates received");
//     return ESP_OK;
// }

void start_websocket_server()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(MDNS_HOST_NAME);

    connect_to_wifi();

    // ESP_ERROR_CHECK(init_fs());
    ws_server_start();
    xTaskCreate(&server_task, "server_task", 3000, NULL, 9, NULL);
    xTaskCreate(&server_handle_task, "server_handle_task", 4000, NULL, 6, NULL);
}