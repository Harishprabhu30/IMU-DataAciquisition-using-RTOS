#pragma once

#include <freertos/event_groups.h>

#define WIFI_CONNECTED_BIT BIT0

extern EventGroupHandle_t wifi_event_group;

void wifi_task_init(void); // Initialize Wi-Fi task