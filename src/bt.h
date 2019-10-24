#ifndef _WASP_BT_H
#define _WASP_BT_H

#ifdef __cplusplus
extern "C" {
#endif
//  Recommend attr size <= 182 bytes to be compatible with Iphone
#define MAX_ATTR_SIZE     200

#define BT_MEM_BLOCK_SIZE 256

#define NOTIFY_PRIORITY 0


typedef struct appl_attr_info_s {
    uint16_t appl_id;
    uint16_t size;
    void *data_ptr;
    const struct bt_gatt_attr *attr_ptr;
    uint8_t properties;
} appl_attr_info_t;

appl_attr_info_t * get_attr_by_appl_id(uint16_t appl_id);

int bt_confirm_paring(void);

void bt_init(void);
struct bt_conn * conn_get(void);

int queue_notify(const struct bt_gatt_attr *attr_ptr, uint16_t seq_num, uint16_t size, uint16_t flags, const void * pdata);
int queue_indicate(const struct bt_gatt_attr *attr_ptr, uint16_t seq_num, uint16_t size, uint16_t flags, const void * pdata);

void set_size_to_notify(int size);
void sample_frame_notify_dbg(int size_to_notify);

int debug_notify_enable_get(void);
void debug_notify_enable_set( int enable);
int debug_notify_delay_get(void);
void debug_notify_delay_set(int delay);

#ifdef __cplusplus
}
#endif

#endif