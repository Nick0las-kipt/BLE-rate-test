/* main.c - Application main entry point */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>

#include <strings.h>
#include <stdlib.h>
#include <stdio.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <../subsys/bluetooth/host/keys.h>

#include <shell/shell.h>

#include "bt.h"
#include "lna.h"

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(app);

void main(void)
{
    lna_init();
    bt_init();
}

static int cmd_notify(const struct shell *shell, size_t argc, char **argv)
{
    if (argc >= 2){
        if ((0 == strncasecmp(argv[1],"on",3)) || (0 == strncasecmp(argv[1],"1",3))) {
            shell_print(shell, "Notifications enabled, delay %dms", debug_notify_delay_get());
            debug_notify_enable_set(1);
            return 0;
        } else if ((0 == strncasecmp(argv[1],"off",3)) || (0 == strncasecmp(argv[1],"0",3))){
            shell_print(shell, "Notifications disabled");
            debug_notify_enable_set(0);
            return 0;
        }
    }

    shell_print(shell, "Usage: %s <on|1|off|0>", (argc) ? argv[0] : "notify");

    return 0;
}

#define INTERVAL_DELAY_MIN 1
#define INTERVAL_DELAY_MAX 10000

static int cmd_interval(const struct shell *shell, size_t argc, char **argv)
{
    if (argc >= 2){
        int new_delay = atoi(argv[1]);
        if ((new_delay >= INTERVAL_DELAY_MIN) && (new_delay <= INTERVAL_DELAY_MAX)){
            debug_notify_delay_set(new_delay);
            shell_print(shell, "Interval set to %dms", new_delay);
            shell_print(shell, "Notifications: %s", debug_notify_enable_get() ? "On" : "Off");
            return 0;
        } else {
            shell_print(shell, "Bad delay: \"%s\"", argv[1]);
        }
    }

    shell_print(shell, "Usage: %s <delay> (%d..%d)", (argc) ? argv[0] : "interval", INTERVAL_DELAY_MIN, INTERVAL_DELAY_MAX);

    return 0;
}

static int cmd_datasize(const struct shell *shell, size_t argc, char **argv)
{
    if (argc >= 2){
        int new_sample_size = atoi(argv[1]);
        if ((new_sample_size > 0) && (new_sample_size <= MAX_ATTR_SIZE)){
            set_size_to_notify(new_sample_size);
            shell_print(shell, "Data sample size set to %d bytes", new_sample_size);
            shell_print(shell, "Notifications: %s", debug_notify_enable_get() ? "On" : "Off");
            return 0;
        } else {
            shell_print(shell, "Bad data sample size: \"%s\"", argv[1]);
        }
    }

    shell_print(shell, "Usage: %s <data sample size in bytes> (1 .. %d)", (argc) ? argv[0] : "datasize", MAX_ATTR_SIZE);

    return 0;
}
//int bt_conn_le_param_update(struct bt_conn *conn, const struct bt_le_conn_param *param)

static int cmd_btparam(const struct shell *shell, size_t argc, char **argv)
{
    int minint = 0;
    int maxint = 0;
    int latency = CONFIG_BT_PERIPHERAL_PREF_SLAVE_LATENCY;
    int timeout = CONFIG_BT_PERIPHERAL_PREF_TIMEOUT;


    if (argc >= 3){
        minint  = ((atoi(argv[1])+1) * 4)/5;
        maxint  = ((atoi(argv[2])+1) * 4)/5;
        if (argc > 3) latency = atoi(argv[3]);
        if (argc > 4) timeout = (atoi(argv[4])+5) / 10;
        if (minint < 6){
            shell_print(shell, "min int too short");
            return 0;            
        }
        if (maxint < minint){
            shell_print(shell, "max int too short");
            return 0;
        }
        if (latency < 0){
            shell_print(shell, "latency too short");
            return 0;
        }
        if (10 * timeout < 16 * (1 + latency) * maxint){
            shell_print(shell, "timeout too short");
            return 0;
        }
        if (timeout >= 3200){
            shell_print(shell, "timeout too long");
            return 0;
        }
        struct bt_conn * conn;
        if ((conn = conn_get())) {
            int rv = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(minint, maxint, latency, timeout));
            
            shell_print(shell, "Update connection parameters. Result=%d",rv);
            
            return 0;
        } else {
            shell_print(shell, "No active connection, nothing to update");
            return 0;
        }
    }

    shell_print(shell, "Usage: %s <min int> <max int> [latency] [timeout]. All times in ms", (argc) ? argv[0] : "btparam");

    return 0;
}

static int cmd_unpair(const struct shell *shell, size_t argc, char **argv)
{
    int rv = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
    shell_print(shell, "Unpairing. Result: %d", rv);
    return 0;
}

static void bt_keys_print(struct bt_keys *keys, void *data){
    shell_print(data, "Keys for %02x:%02x:%02x:%02x:%02x:%02x (type %x) ", 
        keys->addr.a.val[0], keys->addr.a.val[1], keys->addr.a.val[2],
        keys->addr.a.val[3], keys->addr.a.val[4], keys->addr.a.val[5],
        keys->addr.type );
    shell_print(data, "  id=0x%x, size=0x%x, flags=0x%x, keys=0x%04x", keys->id, keys->enc_size, keys->flags, keys->keys);
    shell_print(data, "  irk=%02x%02x%02x%02x-%02x%02x%02x%02x-%02x%02x%02x%02x-%02x%02x%02x%02x",
        keys->irk.val[0], keys->irk.val[1], keys->irk.val[2], keys->irk.val[3], 
        keys->irk.val[4], keys->irk.val[5], keys->irk.val[6], keys->irk.val[7], 
        keys->irk.val[8], keys->irk.val[9], keys->irk.val[10],keys->irk.val[11], 
        keys->irk.val[12],keys->irk.val[13],keys->irk.val[14],keys->irk.val[15]); 
    shell_print(data, "  irk addr=%02x:%02x:%02x:%02x:%02x:%02x",
        keys->irk.rpa.val[0], keys->irk.rpa.val[1], keys->irk.rpa.val[2], 
        keys->irk.rpa.val[3], keys->irk.rpa.val[4], keys->irk.rpa.val[5]);
#if defined(CONFIG_BT_KEYS_OVERWRITE_OLDEST)
    shell_print(data, "  cntr=%d", keys->aging_counter);
#endif    
}

static int cmd_pairinfo(const struct shell *shell, size_t argc, char **argv)
{
    bt_keys_foreach(0xffff, bt_keys_print, (void*) shell);
    return 0;
}

static int cmd_confirm(const struct shell *shell, size_t argc, char **argv)
{
    int rv = bt_confirm_paring();
    shell_print(shell, "Confirm result: %d", rv);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_rate,
    SHELL_CMD(notify,   NULL, "Enable/disable simulated notify", cmd_notify),
    SHELL_CMD(interval, NULL, "Configure notification interval", cmd_interval),
    SHELL_CMD(datasize, NULL, "Configure notification data size", cmd_datasize),
    SHELL_CMD(btparam,  NULL, "Configure BT connection parameters", cmd_btparam),
    SHELL_CMD(unpair,   NULL, "Unpair all devices", cmd_unpair),
    SHELL_CMD(pairinfo, NULL, "Print paring info", cmd_pairinfo),
    SHELL_CMD(confirm,  NULL, "Confirm paring", cmd_confirm),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(rate, &sub_rate, "Rate test commands", NULL);