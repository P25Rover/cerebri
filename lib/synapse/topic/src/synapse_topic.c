/*
 * Copyright (c) 2023 CogniPilot Foundation <cogni@cognipilot.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <math.h>
#include <stdio.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

LOG_MODULE_REGISTER(zros_topic);

#include "synapse_shell_print.h"
#include "synapse_topic_list.h"

#define TOPIC_DICTIONARY()                                                     \
    (actuators, &topic_actuators, "actuators"),                                \
    (imu, &topic_imu, "imu"),                                                  \
    (joy, &topic_joy, "joy"),                                                  \
    (led_array, &topic_led_array, "led_array"),                                \
    (status, &topic_status, "status")

int topic_count_hz(const struct shell* sh, struct zros_topic* topic, void* msg, snprint_t* echo)
{
    struct zros_sub sub;
    struct zros_node node;
    zros_node_init(&node, "sub hz");
    zros_sub_init(&sub, &node, topic, msg, 1000);
    struct k_poll_event events[1] = {
        *zros_sub_get_event(&sub),
    };

    int64_t ticks_start = k_uptime_ticks();
    int64_t elapsed_ticks = 0;
    const int64_t ticks_sample = 5 * CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int64_t ticks_remaining = ticks_sample;

    static const int max_msg = 11;
    int64_t msg_tick[max_msg];
    double sample_sec[max_msg - 1];

    int msg_count = 0;

    while (ticks_remaining > 0.1 * CONFIG_SYS_CLOCK_TICKS_PER_SEC && msg_count < max_msg) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events),
            K_MSEC(1e3 * ticks_remaining / CONFIG_SYS_CLOCK_TICKS_PER_SEC));
        if (rc != 0) {
            char name[20];
            zros_topic_get_name(topic, name, sizeof(name));
            LOG_WRN("%s not published.", name);
        }

        elapsed_ticks = k_uptime_ticks() - ticks_start;
        ticks_remaining = ticks_sample - elapsed_ticks;

        if (zros_sub_update_available(&sub)) {
            rc = zros_sub_update(&sub);
            if (rc == 0) {
                msg_tick[msg_count] = k_uptime_ticks();
                msg_count++;
            } else {
                LOG_ERR("sub update failed");
            }
        }
    }

    double mean = 0;
    double min = 0;
    double max = 0;
    shell_print(sh, "sample   delta");
    for (int i = 0; i < msg_count - 1; i++) {
        sample_sec[i] = (float)(msg_tick[i + 1] - msg_tick[i]) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        if (i == 0) {
            min = sample_sec[i];
            max = sample_sec[i];
        } else {
            if (sample_sec[i] < min) {
                min = sample_sec[i];
            } else if (sample_sec[i] > max) {
                max = sample_sec[i];
            }
        }
        mean += sample_sec[i];
        shell_print(sh, "  %d  %10.6fs", i, sample_sec[i]);
    }
    mean /= (msg_count - 1);

    double std = 0;
    for (int i = 0; i < msg_count - 1; i++) {
        double v = sample_sec[i] - mean;
        std += v * v;
    }
    std = sqrt(std / (msg_count - 1));

    zros_sub_fini(&sub);
    zros_node_fini(&node);

    shell_print(sh, "average rate: %8.3f Hz\n"
                    "min: %10.6fs, max: %10.6fs, std: %10.6fs, window: %d",
        1.0 / mean, min, max, std, msg_count);
    return ZROS_OK;
}

int topic_echo(const struct shell* sh, struct zros_topic* topic, void* msg, snprint_t* echo)
{
    static char buf[2048] = {};
    struct zros_sub sub;
    struct zros_node node;
    zros_node_init(&node, "sub hz");
    zros_sub_init(&sub, &node, topic, msg, 1000);
    char name[20] = {};
    struct k_poll_event events[1] = {
        *zros_sub_get_event(&sub),
    };
    float sample_period = 2.0;
    int rc = 0;
    rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(sample_period * 1e3));
    zros_topic_get_name(topic, name, sizeof(name));
    if (rc != 0) {
        LOG_WRN("%s not published.", name);
        return -1;
    } else {
        if (!zros_sub_update_available(&sub)) {
            LOG_WRN("%s no update available.", name);
            return -1;
        } else {
            zros_sub_update(&sub);
            echo(buf, sizeof(buf), msg);
            printf("%s", buf);
            memset(buf, 0, sizeof(buf));
        }
    }
    zros_sub_fini(&sub);
    zros_node_fini(&node);
    return ZROS_OK;
}

typedef int msg_handler_t(const struct shell* sh, struct zros_topic* topic, void* msg, snprint_t* echo);

int handle_msg(const struct shell* sh, struct zros_topic* topic, msg_handler_t* handler)
{
    if (topic == &topic_actuators) {
        synapse_msgs_Actuators msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_actuators);
    } else if (topic == &topic_status) {
        synapse_msgs_Status msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_status);
    } else if (topic == &topic_imu) {
        synapse_msgs_Imu msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_imu);
    } else if (topic == &topic_joy) {
        synapse_msgs_Joy msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_joy);
    } else if (topic == &topic_led_array) {
        synapse_msgs_LEDArray msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_ledarray);
    } else {
        char name[20];
        zros_topic_get_name(topic, name, sizeof(name));
        shell_print(sh, "%s not handled", name);
    }
    return ZROS_OK;
}

static int cmd_zros_topic_hz(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct zros_topic* topic = (struct zros_topic*)data;
    return handle_msg(sh, topic, &topic_count_hz);
}

static int cmd_zros_topic_echo(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct zros_topic* topic = (struct zros_topic*)data;
    return handle_msg(sh, topic, &topic_echo);
}

void topic_print_iterator(const struct zros_topic* topic, void* data)
{
    const struct shell* sh = (const struct shell*)data;
    char name[20];
    zros_topic_get_name(topic, name, sizeof(name));
    shell_print(sh, "%s", name);
}

static int cmd_zros_topic_list(const struct shell* sh,
    size_t argc, char** argv)
{
    zros_broker_iterate_topic(topic_print_iterator, (void*)sh);
    return ZROS_OK;
}

void pub_print_iterator(const struct zros_pub* pub, void* data)
{
    const struct shell* sh = (const struct shell*)data;
    char name[30];
    zros_node_get_name(pub->_node, name, sizeof(name));
    shell_print(sh, "\t%s", name);
}

void sub_print_iterator(const struct zros_sub* sub, void* data)
{
    const struct shell* sh = (const struct shell*)data;
    char name[30];
    zros_node_get_name(sub->_node, name, sizeof(name));
    shell_print(sh, "\t%s", name);
}

static int cmd_zros_topic_info(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct zros_topic* topic = (struct zros_topic*)data;
    shell_print(sh, "pubs");
    zros_topic_iterate_pub(topic, pub_print_iterator, (void*)sh);
    shell_print(sh, "subs");
    zros_topic_iterate_sub(topic, sub_print_iterator, (void*)sh);
    return ZROS_OK;
}

void node_print_iterator(const struct zros_node* node, void* data)
{
    const struct shell* sh = (const struct shell*)data;
    char name[30];
    zros_node_get_name(node, name, sizeof(name));
    shell_print(sh, "%s", name);
}

static int cmd_zros_node_list(const struct shell* sh,
    size_t argc, char** argv)
{
    zros_broker_iterate_nodes(node_print_iterator, (void*)sh);
    return ZROS_OK;
}

// level 2 (topic echo/hz/list)
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_echo, cmd_zros_topic_echo, TOPIC_DICTIONARY());
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_hz, cmd_zros_topic_hz, TOPIC_DICTIONARY());
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_info, cmd_zros_topic_info, TOPIC_DICTIONARY());

SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros_topic,
    SHELL_CMD(echo, &sub_zros_topic_echo, "Echo topic.", NULL),
    SHELL_CMD(hz, &sub_zros_topic_hz, "Check topic pub rate.", NULL),
    SHELL_CMD(info, &sub_zros_topic_info, "Topic pubs and subs.", NULL),
    SHELL_CMD(list, NULL, "List topics.", cmd_zros_topic_list),
    SHELL_SUBCMD_SET_END);

// level 2 (node list)
SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros_node,
    SHELL_CMD(list, NULL, "List nodes.", cmd_zros_node_list),
    SHELL_SUBCMD_SET_END);

// level 1 (topic/node)
SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros,
    SHELL_CMD(topic, &sub_zros_topic, "Topic commands.", NULL),
    SHELL_CMD(node, &sub_zros_node, "Node commands.", NULL),
    SHELL_SUBCMD_SET_END);

// level 0 (zros)
SHELL_CMD_REGISTER(zros, &sub_zros, "ZROS Commands", NULL);

/* vi: ts=4 sw=4 et: */
