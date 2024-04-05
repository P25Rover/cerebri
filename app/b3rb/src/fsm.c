/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0

	Ce node ZROS est relié à l'interface de contrôle via le topic "JOY" (pour "JOYSTICK"). 
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4
#define STATE_ANY -1

LOG_MODULE_REGISTER(b3rb_fsm, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

static void transition(
    void* state,
    bool request,
    const char* request_name,
    int pre,
    uint8_t post,
    char* buf,
    size_t n,
    int* request_seq,
    bool* request_rejected,
    size_t n_guards,
    ...)
{
    uint8_t* state_int = (uint8_t*)state;

    // not requested
    if (!request) {
        return;
    }

    // null transition
    if (*state_int == post) {
        return;
    }

    // LOG_INF("%s pre: %d, post: %d, state: %d", request_name, pre, post, *state_int);
    // pre state required and not matched
    if (pre >= 0 && *state_int != pre) {
        return;
    }

    // new valid request
    (*request_seq)++;

    // check guards
    va_list ap;
    va_start(ap, n_guards);
    for (size_t i = 0; i < n_guards; i++) {
        int guard = va_arg(ap, int);
        const char* guard_txt = va_arg(ap, const char*);
        if (guard) {
            snprintf(buf, n, "deny %s: %s", request_name, guard_txt);
            LOG_WRN("%s", buf);
            *request_rejected = true;
            return;
        }
    }
    va_end(ap);
    snprintf(buf, n, "accept %s", request_name);
    LOG_INF("%s", buf);
    *state_int = post;
    *request_rejected = false;
    return;
}

typedef struct status_input_s {
    bool request_arm;
    bool request_disarm;
    bool request_manual;
    bool request_auto;
    bool request_cmd_vel;
    bool request_calibration;
    bool safe;
    bool mode_set;
    bool fuel_low;
    bool fuel_critical;
} status_input_t;

typedef struct _context {
    struct zros_node node;
    synapse_msgs_Joy joy;
    synapse_msgs_Status status;
    status_input_t status_input;
    struct zros_sub sub_joy;
    struct zros_pub pub_status;
} context;

static context g_ctx = {
    .node = {},
    .joy = synapse_msgs_Joy_init_default,
    .status = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default,
        },
        .arming = synapse_msgs_Status_Arming_ARMING_DISARMED,
        .fuel = synapse_msgs_Status_Fuel_FUEL_UNKNOWN,
        .fuel_percentage = 0,
        .joy = synapse_msgs_Status_Joy_JOY_UNKNOWN,
        .mode = synapse_msgs_Status_Mode_MODE_UNKNOWN,
        .power = 0.0,
        .status_message = "",
    },
    .sub_joy = {},
    .pub_status = {},
};

static void b3rb_fsm_init(context* ctx)
{
    zros_node_init(&ctx->node, "b3rb_fsm");
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_pub_init(&ctx->pub_status, &ctx->node, &topic_status, &ctx->status);
}

static void fsm_compute_input(status_input_t* input, const context* ctx)
{
    input->request_arm = ctx->joy.buttons[JOY_BUTTON_ARM] == 1;
    input->request_disarm = ctx->joy.buttons[JOY_BUTTON_DISARM] == 1;
    input->request_manual = ctx->joy.buttons[JOY_BUTTON_MANUAL] == 1;
    input->request_auto = ctx->joy.buttons[JOY_BUTTON_AUTO] == 1;
    input->request_cmd_vel = ctx->joy.buttons[JOY_BUTTON_CMD_VEL] == 1;
    input->request_calibration = ctx->joy.buttons[JOY_BUTTON_CALIBRATION] == 1;
    input->mode_set = ctx->status.mode != synapse_msgs_Status_Mode_MODE_UNKNOWN;
}

static void fsm_update(synapse_msgs_Status* status, const status_input_t* input)
{
    // arm transition
    transition(
        &status->arming, // state
        input->request_arm, // request
        "request arm", // label
        synapse_msgs_Status_Arming_ARMING_DISARMED, // pre
        synapse_msgs_Status_Arming_ARMING_ARMED, // post
        status->status_message, sizeof(status->status_message), // status
        &status->request_seq, &status->request_rejected, // request
        // guards
        5,
        status->mode == synapse_msgs_Status_Mode_MODE_UNKNOWN, "mode not set",
        status->mode == synapse_msgs_Status_Mode_MODE_CALIBRATION, "mode calibration",
        input->safe, "safety on",
        input->fuel_critical, "fuel_critical",
        input->fuel_low, "fuel_low");

    // disarm transitions
    transition(
        &status->arming, // state
        input->request_disarm, // request
        "request disarm", // label
        synapse_msgs_Status_Arming_ARMING_ARMED, // pre
        synapse_msgs_Status_Arming_ARMING_DISARMED, // post
        status->status_message, sizeof(status->status_message), // status
        &status->request_seq, &status->request_rejected, // request
        0); // guards

    // mode transitions
    transition(
        &status->mode, // state
        input->request_manual, // request
        "request mode manual", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_MANUAL, // post
        status->status_message, sizeof(status->status_message), // status
        &status->request_seq, &status->request_rejected, // request
        0); // guards

    transition(
        &status->mode, // state
        input->request_cmd_vel, // request
        "request mode cmd_vel", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_CMD_VEL, // post
        status->status_message, sizeof(status->status_message), // status
        &status->request_seq, &status->request_rejected, // request
        0); // guards
        
    transition(
        &status->mode, // state
        input->request_auto, // request
        "request mode auto", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_AUTO, // post
        status->status_message, sizeof(status->status_message), // status
        &status->request_seq, &status->request_rejected, // request
        0); // guards

    transition(
        &status->mode, // state
        input->request_calibration, // request
        "request mode calibration", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_CALIBRATION, // post
        status->status_message, sizeof(status->status_message), // status
        &status->request_seq, &status->request_rejected, // request
        // guards
        1,
        status->arming == synapse_msgs_Status_Arming_ARMING_ARMED, "disarm required");

    // set timestamp
    stamp_header(&status->header, k_uptime_ticks());
    status->header.seq++;
}

static void status_add_extra_info(synapse_msgs_Status* status, status_input_t* input)
{
    if (input->fuel_critical) {
        status->fuel = synapse_msgs_Status_Fuel_FUEL_CRITICAL;
    } else if (input->fuel_low) {
        status->fuel = synapse_msgs_Status_Fuel_FUEL_LOW;
    } else {
        status->fuel = synapse_msgs_Status_Fuel_FUEL_NOMINAL;
    }
}

static void b3rb_fsm_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("initializing b3rb_fsm");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    b3rb_fsm_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_joy),
    };

    int64_t joy_last_ticks = k_uptime_ticks();
    int64_t joy_loss_ticks = 1.0 * CONFIG_SYS_CLOCK_TICKS_PER_SEC;

    while (true) {

        // current ticks
        int64_t now_ticks = k_uptime_ticks();

        // wait for input event, publish at 1 Hz regardless
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("fsm joy/battery polling timeout");
        }

        if (zros_sub_update_available(&ctx->sub_joy)) {
            zros_sub_update(&ctx->sub_joy);
            if (ctx->status.joy == synapse_msgs_Status_Joy_JOY_LOSS) {
                LOG_WRN("joy regained");
            }
            joy_last_ticks = now_ticks;
            ctx->status.joy = synapse_msgs_Status_Joy_JOY_NOMINAL;
        }

        // check for joy loss
        if (ctx->status.joy != synapse_msgs_Status_Joy_JOY_LOSS
            && now_ticks - joy_last_ticks > joy_loss_ticks) {
            LOG_WRN("joy loss");
            ctx->status.joy = synapse_msgs_Status_Joy_JOY_LOSS;
        }

        // perform processing
        fsm_compute_input(&ctx->status_input, ctx);
        fsm_update(&ctx->status, &ctx->status_input);
        status_add_extra_info(&ctx->status, &ctx->status_input);
        zros_pub_update(&ctx->pub_status);
    }
}

K_THREAD_DEFINE(b3rb_fsm, MY_STACK_SIZE,
    b3rb_fsm_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
