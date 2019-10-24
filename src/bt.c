/* BT management code */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/byteorder.h>
#include <errno.h>
#include <zephyr.h>

#include <strings.h>
#include <stdlib.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <settings/settings.h>
#include <../subsys/bluetooth/host/keys.h>

#include "bt.h"

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(bt);


#define RATE_DATA_SERVICE_INITIALIZER                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                                         0xBC, 0x9A, 0x78, 0x56, 0x00, 0x11, 0x34, 0x12

#define RATE_DATA_INITIALIZER                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                                         0xBC, 0x9A, 0x78, 0x56, 0x01, 0x11, 0x34, 0x12

static struct bt_uuid_128 rate_data_data_service_uuid = BT_UUID_INIT_128(RATE_DATA_SERVICE_INITIALIZER);
static struct bt_uuid_128 rate_data_uuid =              BT_UUID_INIT_128(RATE_DATA_INITIALIZER);


static uint8_t rate_data_frame_value[MAX_ATTR_SIZE] = {
};


appl_attr_info_t appl_local_attrs[] = {
    {
        .size = sizeof(rate_data_frame_value),
        .data_ptr = &rate_data_frame_value,
    },
};

#define IS_ATTR_LOCAL(arg) (((uint32_t)arg >= (uint32_t)&appl_local_attrs) && \
    ((uint32_t)arg < ((uint32_t)&appl_local_attrs) + sizeof(appl_local_attrs)))

static uint16_t appl_id_from_uuid(const struct bt_uuid * uuid){
    if (BT_UUID_TYPE_128 != uuid->type) return 0;
    uint8_t * pval = (uint8_t*) &(((struct bt_uuid_128 *) uuid)->val);
    return (pval[13] << 8) | (pval[12]);
}

static appl_attr_info_t * appl_attr_get(uint16_t new_appl_id){
    if (!(new_appl_id)) return NULL;
    for (int i = 0; i < sizeof(appl_local_attrs)/sizeof(appl_local_attrs[0]); ++i){
        if (new_appl_id == appl_local_attrs[i].appl_id) return &appl_local_attrs[i];
    }
    return NULL;
}

appl_attr_info_t * get_attr_by_appl_id(uint16_t appl_id){
    if (!(appl_id)) return NULL;
    for (int i = 0; i < sizeof(appl_local_attrs)/sizeof(appl_local_attrs[0]); ++i){
        if (appl_id == appl_local_attrs[i].appl_id) return &appl_local_attrs[i];
    }
    return NULL;
}

static u16_t find_static_handle_for_attr(const struct bt_gatt_attr *attr)
{
    u16_t handle = 1;

    Z_STRUCT_SECTION_FOREACH(bt_gatt_service_static, static_svc) {
        for (int i = 0; i < static_svc->attr_count; i++, handle++) {
            if (attr == &static_svc->attrs[i]) {
                return handle;
            }
        }
    }
    return 0;
}

static const struct bt_gatt_attr *find_static_attr_by_handle(uint16_t f_handle)
{
    if (f_handle < 1) return NULL;
    u16_t handle = 1;

        Z_STRUCT_SECTION_FOREACH(bt_gatt_service_static, static_svc) {
            /* Skip ahead if start is not within service handles */
            if (handle + static_svc->attr_count < f_handle) {
                handle += static_svc->attr_count;
                continue;
            }
            
            int i = f_handle - handle;
            return &static_svc->attrs[i];
        }
    return NULL;
}

static ssize_t read_def(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, u16_t len, u16_t offset)
{
    if (!IS_ATTR_LOCAL(attr->user_data)) return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);

    const appl_attr_info_t * info = attr->user_data;
    const char *value = info->data_ptr;
    const uint16_t size = info->size;

    LOG_INF("Read attr local ID 0x%4x ofs=0x%x len=0x%x", info->appl_id, offset, len);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, size);
}

static ssize_t write_def(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, u16_t len, u16_t offset,
             u8_t flags)
{
    if (!IS_ATTR_LOCAL(attr->user_data)) return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
    
    const appl_attr_info_t * info = attr->user_data;
    char *value = info->data_ptr;
    const uint16_t size = info->size;

    if (offset + len > size) {
        LOG_ERR("Too long write to attr local ID 0x%4x len=%d ofs=%d)",info->appl_id, len, offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    
    LOG_INF("Write to attr local ID 0x%4x len=%d ofs=%d",info->appl_id, len, offset);
    
    memcpy(value + offset, buf, len);
    if ((size - offset - len) > 0) memset(value + offset + len, 0, size - offset - len );

    return len;
}


static void ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    const struct bt_gatt_attr * attr1 = find_static_attr_by_handle(attr->handle - 1);
    LOG_WRN("Configuration for %s: [%s%s]",
        (attr1) ? bt_uuid_str(attr1->uuid) : "<unknown attr>",
         (BT_GATT_CCC_NOTIFY == value) ? "Notify":"",
         (BT_GATT_CCC_INDICATE == value) ? "Indicate":"");
}

// Rate 
BT_GATT_SERVICE_DEFINE(rate_data_data_service,
    BT_GATT_PRIMARY_SERVICE(&rate_data_data_service_uuid),
    BT_GATT_CHARACTERISTIC(&rate_data_uuid.uuid,
                   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY ,
                   BT_GATT_PERM_READ,
                   read_def, NULL, &appl_local_attrs[0]),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x12, 0x18),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, RATE_DATA_SERVICE_INITIALIZER),
};

static struct bt_conn * g_conn;

static void connected(struct bt_conn *conn, u8_t err)
{
    if (err) {
        LOG_INF("Connection failed (err 0x%02x)", err);
    } else {
        g_conn = conn;
        LOG_INF("Connected");
    }
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
    g_conn = NULL;
    LOG_INF("Disconnected (reason 0x%02x)", reason);
}

static void param_updated(struct bt_conn *conn, u16_t interval, u16_t latency, u16_t timeout){
        int mtu = bt_gatt_get_mtu(conn);

        int ibp = (5 * interval) / 4;
        int iap = (125*interval) % 100;

        int lbp = (5 * latency) / 4;
        int lap = (125*latency) % 100;
    
        LOG_WRN("Connection params updated, MTU = %u, interval=%d.%02dms, latency=%d.%02dms, timeout = %dms", mtu,
            ibp, iap, lbp, lap, 10 * timeout);
}

static void le_security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err){
        LOG_WRN("Connection security changed: level=%d error=%d", level, err);   
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = param_updated,
    .security_changed = le_security_changed,
};

static void bt_ready(int err)
{
    if (err) {
        LOG_INF("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_INF("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_ERR("Passkey for %s: %06u", log_strdup(addr), passkey);
}

static struct bt_conn *conn_to_confirm = NULL;

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", log_strdup(addr));

    conn_to_confirm = NULL;
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing complete: %s reult: %s", log_strdup(addr), bonded ? "Bonded":"Not bonded");

    conn_to_confirm = NULL;
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing failed: bt%s result: %d", log_strdup(addr), reason);

    conn_to_confirm = NULL;
}

static void auth_pairing_confirm(struct bt_conn *conn){
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing confirm request: %s", log_strdup(addr));

    conn_to_confirm = conn;
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
//    .passkey_display = NULL,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
    .pairing_complete = auth_pairing_complete,
    .pairing_failed = auth_pairing_failed,
//    .pairing_confirm = auth_pairing_confirm,
};

int bt_confirm_paring(void){
    return (conn_to_confirm) ? bt_conn_auth_pairing_confirm(conn_to_confirm) : -ENOTCONN;
}

static void attrs_post_init(){
    Z_STRUCT_SECTION_FOREACH(bt_gatt_service_static, static_svc) {
        for (int i = 0; i < static_svc->attr_count; i++) {
            const struct bt_gatt_attr * attr_ptr = &static_svc->attrs[i];
            
            if (IS_ATTR_LOCAL(attr_ptr->user_data)){
                appl_attr_info_t * info = (appl_attr_info_t*) attr_ptr->user_data;
                info->attr_ptr = attr_ptr;
                uint16_t handle = find_static_handle_for_attr(attr_ptr);
                const struct bt_gatt_attr * attr1 = find_static_attr_by_handle(handle - 1);
            
                if ((attr1) && !bt_uuid_cmp(attr1->uuid, BT_UUID_GATT_CHRC)) {
                    struct bt_gatt_chrc *chrc = attr1->user_data;
                    info->properties = chrc->properties;
                }                
                if (0 == info->appl_id){
                    uint16_t new_appl_id = appl_id_from_uuid(attr_ptr->uuid);
                    if (!(appl_attr_get(new_appl_id))) info->appl_id = new_appl_id;
                }
                LOG_DBG("UUID %s known as ID 0x%x, Properties=0x%x handle=%d size=%d",
                    bt_uuid_str(attr_ptr->uuid), info->appl_id, info->properties, handle, info->size);
            }
        }
    }
}

void bt_init(void)
{
    int err = 0;
    attrs_post_init();

    err = bt_enable(bt_ready);
    if (err) {
        LOG_INF("Bluetooth init failed (err %d)", err);
        return;
    }
   
    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);
}

struct bt_conn * conn_get(void){
    return g_conn;
}

typedef struct notify_msg_s {
    const struct bt_gatt_attr *attr_ptr;
    uint16_t seq_num;
    uint16_t size;
    uint16_t flags;
    uint8_t data[MAX_ATTR_SIZE];
} notify_msg_t;

K_MEM_POOL_DEFINE(notify_pool,BT_MEM_BLOCK_SIZE, BT_MEM_BLOCK_SIZE, 8, 4);

K_MSGQ_DEFINE(notify_msgq, sizeof(struct k_mem_block), 4, 4);
K_MSGQ_DEFINE(indicate_msgq, sizeof(struct k_mem_block), 4, 4);

struct k_poll_signal notify_signal = K_POLL_SIGNAL_INITIALIZER(notify_signal);

int queue_put(const struct bt_gatt_attr *attr_ptr, uint16_t seq_num, uint16_t size,
              uint16_t flags, const void * pdata, struct k_msgq * queue)
{
    struct k_mem_block block;
    if (size > MAX_ATTR_SIZE){
        return -ENOMEM;
    }
    if (0 == k_mem_pool_alloc(&notify_pool, &block, sizeof(notify_msg_t), K_NO_WAIT)){
        notify_msg_t * msg = (notify_msg_t *)block.data;
        msg->attr_ptr = attr_ptr;
        msg->seq_num = seq_num;
        msg->size = size;
        msg->flags = flags;
        memcpy(&(msg->data[0]), pdata, size);
        if (0 == k_msgq_put(queue, &block, K_NO_WAIT)){
            k_poll_signal_raise(&notify_signal, 0);
            return 0;
        } else {
            k_mem_pool_free(&block);
            return -ENOMEM;
        }
    } else {
        return -ENOMEM;
    }
}

int queue_indicate(const struct bt_gatt_attr *attr_ptr, uint16_t seq_num, uint16_t size,
              uint16_t flags, const void * pdata)
{
    const struct bt_gatt_attr * attr1 = find_static_attr_by_handle(find_static_handle_for_attr(attr_ptr) - 1);

    if ((attr1) && !bt_uuid_cmp(attr1->uuid, BT_UUID_GATT_CHRC)) {
        struct bt_gatt_chrc *chrc = attr1->user_data;
        if (!(chrc->properties & BT_GATT_CHRC_INDICATE)) {
            return -EINVAL;
        }
    }
    return queue_put(attr_ptr, seq_num, size, flags, pdata, &indicate_msgq);
}


int queue_notify(const struct bt_gatt_attr *attr_ptr, uint16_t seq_num, uint16_t size,
              uint16_t flags, const void * pdata)
{
    const struct bt_gatt_attr * attr1 = find_static_attr_by_handle(find_static_handle_for_attr(attr_ptr) - 1);

    if ((attr1) && !bt_uuid_cmp(attr1->uuid, BT_UUID_GATT_CHRC)) {
        struct bt_gatt_chrc *chrc = attr1->user_data;
        if (!(chrc->properties & BT_GATT_CHRC_NOTIFY)) {
            return -EINVAL;
        }
    }
    return queue_put(attr_ptr, seq_num, size, flags, pdata, &notify_msgq);
}

static int ind_err;
static int ind_bsy;

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, u8_t err);

static struct k_poll_event notify_events[1] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                 K_POLL_MODE_NOTIFY_ONLY,
                                 &notify_signal),
    };

void notify_thread(void *arg0, void *arg1, void *arg2){
    (void) arg0;
    (void) arg1;
    (void) arg2;
    struct k_mem_block block;
    while (1){
        k_poll(notify_events, 1, K_FOREVER);
        notify_events[0].signal->signaled = 0;
        notify_events[0].state = K_POLL_STATE_NOT_READY;
        while (0 == k_msgq_get(&indicate_msgq, &block, K_NO_WAIT)){
            notify_msg_t * msg = (notify_msg_t *)block.data;
            uint32_t before = k_cycle_get_32();
            static struct bt_gatt_indicate_params ind_params;
            
            ind_params.attr = msg->attr_ptr;
            ind_params.func = indicate_cb;
            ind_params.data = &msg->data[0];
            ind_params.len = msg->size;
            
            ind_bsy = 1;
            
            int res = bt_gatt_indicate(NULL, &ind_params);

            if (res) {
                ind_bsy = 0;
            } else {
                while (ind_bsy) k_sleep(K_MSEC(1000));
                res = ind_err;
            }

            uint32_t after = k_cycle_get_32();
            int ms = (1000U * (after - before)) / sys_clock_hw_cycles_per_sec();
            LOG_INF("Send indication 0x%02x, (%dms)", msg->seq_num, ms);
            k_mem_pool_free(&block);
        }
        if (0 == k_msgq_get(&notify_msgq, &block, K_NO_WAIT)){
            notify_msg_t * msg = (notify_msg_t *)block.data;
            uint32_t before = k_cycle_get_32();
            int res = bt_gatt_notify(NULL, msg->attr_ptr, &msg->data[0], msg->size);

            uint32_t after = k_cycle_get_32();
            int ms = (1000U * (after - before)) / sys_clock_hw_cycles_per_sec();
            LOG_INF("Send notification 0x%02x, (%dms) rv=%d", msg->seq_num, ms, res);
            k_mem_pool_free(&block);
        }
    }
}

K_THREAD_DEFINE(notify_tid, 1024,
                notify_thread, NULL, NULL, NULL,
                NOTIFY_PRIORITY, 0, K_NO_WAIT);

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, u8_t err)
{
    ind_err = -err;
    ind_bsy = 0;
    k_wakeup(notify_tid);
}

static int frame_num = 0;

static const int tdigs = 6;
int sample_frame_randomize(int osize){
    if (osize > sizeof(rate_data_frame_value)) osize = sizeof(rate_data_frame_value);
    uint32_t us100_elapsed = (k_cycle_get_32() * 10000LL) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    for (uint32_t * pdata = (uint32_t *) &rate_data_frame_value[0]; pdata < &rate_data_frame_value[osize]; ++pdata){
        *pdata = sys_rand32_get();
    }    
    for (int i = tdigs - 1; i >= 0; --i) {
        int shift = (((tdigs-1) & 1) == (i & 1)) ? 0 : 4;
        if (shift){
            rate_data_frame_value[i>>1] |= (us100_elapsed %10) << shift;
        }else {
            rate_data_frame_value[i>>1] = us100_elapsed % 10;
        }
        us100_elapsed /=10;
    }
    frame_num = (1 + frame_num) % 100;
    int num_ofs = (1 + tdigs) >> 1;
    rate_data_frame_value[num_ofs++] = 0;
    rate_data_frame_value[num_ofs++] = frame_num % 10 + ((frame_num /10) << 4);
    rate_data_frame_value[num_ofs] = 0;
    if (osize >= (num_ofs << 1)){
        rate_data_frame_value[osize-num_ofs-1] = 0;
        memcpy(&rate_data_frame_value[osize-num_ofs],&rate_data_frame_value[0], num_ofs);
    }
    return frame_num;
}

static int sample_size_to_notify = sizeof(rate_data_frame_value);

void set_size_to_notify(int size){
    sample_size_to_notify = size;
}

void sample_frame_notify_dbg(int size_to_notify){
    // Debug notification;
    int num = sample_frame_randomize(size_to_notify);
    if (0 == size_to_notify) size_to_notify = sizeof(rate_data_frame_value);
    int res = queue_notify(appl_local_attrs[0].attr_ptr, 0xff00 + num, size_to_notify, 0, &rate_data_frame_value);
    if (res){
        LOG_ERR("Send DBG notification 0x%02x, failed (%d)", num, res);
    } else {
        LOG_INF("Send DBG notification 0x%02x", num);
    }
}

static uint8_t notify_enable = 0;
static int simulate_loop_delay = 33;

void simulate_thread(void *arg0, void *arg1, void *arg2){
    (void) arg0;
    (void) arg1;
    (void) arg2;
    int ms = 0;
    while (1){
            int time_to_sleep = simulate_loop_delay - ms;
            if (time_to_sleep < 1) time_to_sleep = 1;
            if (notify_enable) LOG_INF("Elapsed: %d, Sleep: %d", ms, time_to_sleep);
            k_sleep(K_MSEC(time_to_sleep));
            uint32_t before = k_cycle_get_32();
            if (notify_enable) {
                sample_frame_notify_dbg(sample_size_to_notify);
            }
            k_yield();
            k_yield();

            uint32_t after = k_cycle_get_32();
            ms = (1000U * (after - before)) / sys_clock_hw_cycles_per_sec();
    }
}

K_THREAD_DEFINE(simulate_tid, 1024,
                simulate_thread, NULL, NULL, NULL,
                CONFIG_NUM_PREEMPT_PRIORITIES - 1, 0, K_NO_WAIT);

int debug_notify_enable_get(void){
    return notify_enable;
}

void debug_notify_enable_set(int enable){
    notify_enable = enable;
}

int debug_notify_delay_get(void){
    return simulate_loop_delay;
}

void debug_notify_delay_set(int delay){
    simulate_loop_delay = delay;
}
