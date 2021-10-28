/*
 *    RoboPeak USB LCD Display Linux Driver
 *
 *    Copyright (C) 2009 - 2013 RoboPeak Team
 *    This file is licensed under the GPL. See LICENSE in the package.
 *
 *    http://www.robopeak.net
 *
 *    Author Shikai Chen
 *   -----------------------------------------------------------------
 *    USB Driver Implementations
 */

#include "inc/common.h"
#include "inc/usbhandlers.h"
#include "inc/fbhandlers.h"
#include "inc/touchhandlers.h"
#include"inc/tiny_jpeg.h"


#define DL_ALIGN_UP(x, a) ALIGN(x, a)
#define DL_ALIGN_DOWN(x, a) ALIGN(x-(a-1), a)

static const struct usb_device_id id_table[] = {
    {
        .idVendor    = RP_DISP_USB_VENDOR_ID,
        .idProduct   = RP_DISP_USB_PRODUCT_ID,
        .match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,
    },
    { },
};

static atomic_t _devlist_count = ATOMIC_INIT(0);
static LIST_HEAD(rpusbdisp_list);
static DEFINE_MUTEX(_mutex_usbdevlist);
static DECLARE_WAIT_QUEUE_HEAD(_usblist_waitqueue);

#if 0
static struct task_struct * _usb_status_polling_task;
static int _kthread_usb_status_poller_proc(void *data);
#endif

static volatile int _working_flag = 0;


#define RPUSBDISP_STATUS_BUFFER_SIZE   32


struct rpusbdisp_disp_ticket_bundle {
    int  ticket_count;
    struct list_head   ticket_list;

};

struct rpusbdisp_disp_ticket {
    struct urb                      *  transfer_urb;
    struct list_head                   ticket_list_node;
    struct rpusbdisp_dev            *  binded_dev;

};


struct rpusbdisp_disp_ticket_pool {
    struct list_head                   list;
    spinlock_t                         oplock;
    size_t                             disp_urb_count;
    size_t                             packet_size_factor;//packets number
    int                                availiable_count;
    wait_queue_head_t                  wait_queue;
    struct delayed_work                completion_work;

};

struct rpusbdisp_dev {
    // timing and sync
    struct list_head                   dev_list_node;
    int                                dev_id;
    struct mutex                       op_locker;
    __u8                               is_alive;

    // usb device info
    struct usb_device               *  udev;
    struct usb_interface            *  interface;


    // status package related
    __u8                               status_in_buffer[RPUSBDISP_STATUS_BUFFER_SIZE]; // data buffer for the status IN endpoint
    size_t                             status_in_buffer_recvsize;



    __u8                               status_in_ep_addr;
    wait_queue_head_t                  status_wait_queue;
    struct urb                      *  urb_status_query;
    int                                urb_status_fail_count;


    // display data related
    __u8                               disp_out_ep_addr;
    size_t                             disp_out_ep_max_size;

    struct rpusbdisp_disp_ticket_pool  disp_tickets_pool;



    void                            *  fb_handle;
    void                            *  touch_handle;

    __u16                              device_fwver;
    uint8_t							msg[RP_DISP_DEFAULT_HEIGHT*RP_DISP_DEFAULT_WIDTH*4];
    size_t 							msg_pos;
#define JPEG_MAX_SIZE (RP_DISP_DEFAULT_HEIGHT*RP_DISP_DEFAULT_WIDTH*3)
    uint8_t sg_buf[JPEG_MAX_SIZE];
    stream_mgr_t smgr;
	int jpg_quality;
};


#if 0

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver lcd_class = {
    .name =         "rp_usbdisp%d",
    .fops =         &disp_fops,
    .minor_base =   RPUSBDISP_MINOR,
};

#endif


static int _return_disp_tickets(struct rpusbdisp_dev * dev,  struct  rpusbdisp_disp_ticket * ticket);


struct device * rpusbdisp_usb_get_devicehandle(struct rpusbdisp_dev *dev) {
    return &dev->udev->dev;
}

void rpusbdisp_usb_set_fbhandle(struct rpusbdisp_dev * dev, void * fbhandle)
{
    dev->fb_handle = fbhandle;
}

void * rpusbdisp_usb_get_fbhandle(struct rpusbdisp_dev * dev)
{
    return dev->fb_handle;
}

void rpusbdisp_usb_set_touchhandle(struct rpusbdisp_dev * dev, void * touch_handle)
{
    dev->touch_handle = touch_handle;
}

void * rpusbdisp_usb_get_touchhandle(struct rpusbdisp_dev * dev)
{
    return dev->touch_handle;
}



static void _status_start_querying(struct rpusbdisp_dev * dev);



static void _add_usbdev_to_list(struct rpusbdisp_dev * dev)
{
    mutex_lock(&_mutex_usbdevlist);
    list_add(&dev->dev_list_node, &rpusbdisp_list);
    atomic_inc(&_devlist_count);
    mutex_unlock(&_mutex_usbdevlist);

    wake_up(&_usblist_waitqueue);

}


static void _del_usbdev_from_list(struct rpusbdisp_dev * dev)
{
    mutex_lock(&_mutex_usbdevlist);
    list_del(&dev->dev_list_node);
    atomic_dec(&_devlist_count);
    mutex_unlock(&_mutex_usbdevlist);
}


static void _on_parse_status_packet(struct rpusbdisp_dev *dev)
{
    rpusbdisp_status_packet_header_t * header = (rpusbdisp_status_packet_header_t *)dev->status_in_buffer;

    if(header->packet_type == RPUSBDISP_STATUS_TYPE_NORMAL) {
        // only supports the normal status packet currently
        rpusbdisp_status_normal_packet_t * normalpacket = (rpusbdisp_status_normal_packet_t *)header;



        if(normalpacket->display_status & RPUSBDISP_DISPLAY_STATUS_DIRTY_FLAG) {
            fbhandler_set_unsync_flag(dev);
            schedule_delayed_work(&dev->disp_tickets_pool.completion_work, 0);
        }


        touchhandler_send_ts_event(dev, le32_to_cpu(normalpacket->touch_x), le32_to_cpu(normalpacket->touch_y),  normalpacket->touch_status == RPUSBDISP_TOUCH_STATUS_PRESSED ? 1 : 0);
//        printk("X:%d Y:%d S:%d\n", (int)le32_to_cpu(normalpacket->touch_x), (int)le32_to_cpu(normalpacket->touch_y), normalpacket->touch_status);
    }
}

static void _on_display_transfer_finished_delaywork(struct work_struct *work)
{
    struct rpusbdisp_dev * dev = container_of(work, struct rpusbdisp_dev,
                                 disp_tickets_pool.completion_work.work);


    if(!dev->is_alive) return;
    fbhandler_on_all_transfer_done(dev);
}

static void _on_display_transfer_finished(struct urb *urb)
{
    struct rpusbdisp_disp_ticket * ticket = (struct rpusbdisp_disp_ticket *)urb->context;
    struct rpusbdisp_dev         * dev = ticket->binded_dev;

    int                            all_finished = 0;
    /* sync/async unlink faults aren't errors */
    if(urb->status) {

        if(dev->is_alive) {
            // set unsync flag
            fbhandler_set_unsync_flag(dev);
        }
        err("transmission failed for urb %p, error code %x\n", urb, urb->status);

    }



    urb->transfer_buffer_length = dev->disp_tickets_pool.packet_size_factor * dev->disp_out_ep_max_size; // reset buffer size

    // insert to the available queue
    all_finished = _return_disp_tickets(dev, ticket);

    if(all_finished && !urb->status) {
        schedule_delayed_work(&dev->disp_tickets_pool.completion_work, 0);
    }

}

static void _on_status_query_finished(struct urb *urb)
{
    struct rpusbdisp_dev *dev = urb->context;

    if(!dev->is_alive) {
        return;
    }

    // check the status...
    switch(urb->status) {
    case 0:
        // succeed
        // store the actual transfer size
        dev->status_in_buffer_recvsize = urb->actual_length;


        _on_parse_status_packet(dev);
        // notify the waiters..
        wake_up(&dev->status_wait_queue);
        break;
    case -EPIPE:
        //  usb_clear_halt(dev->udev, usb_rcvintpipe(dev->udev, dev->status_in_ep_addr));
    default:
        //++dev->urb_status_fail_count;
        dev->urb_status_fail_count = RPUSBDISP_STATUS_QUERY_RETRY_COUNT;
    }

    if(dev->urb_status_fail_count < RPUSBDISP_STATUS_QUERY_RETRY_COUNT) {
        _status_start_querying(dev);
    }
}



static void _status_start_querying(struct rpusbdisp_dev * dev)
{
    unsigned int pipe;
    int status;
    struct usb_host_endpoint *ep;


    if(!dev->is_alive) {
        // the device is dead, abort
        return;
    }

    if(dev->urb_status_fail_count >= RPUSBDISP_STATUS_QUERY_RETRY_COUNT) {
        // max retry count has reached, return
        return;
    }
#if 1
    //pipe = usb_rcvintpipe(dev->udev, dev->status_in_ep_addr);
    pipe = usb_rcvbulkpipe(dev->udev, dev->status_in_ep_addr);
    ep = usb_pipe_endpoint(dev->udev, pipe);

    if(!ep) return;


    usb_fill_int_urb(dev->urb_status_query, dev->udev, pipe, dev->status_in_buffer, RPUSBDISP_STATUS_BUFFER_SIZE,
                     _on_status_query_finished, dev,
                     ep->desc.bInterval);

    //submit it
    status = usb_submit_urb(dev->urb_status_query, GFP_ATOMIC);
    if(status) {
        if(status == -EPIPE) {
            usb_clear_halt(dev->udev, pipe);
        }

        ++ dev->urb_status_fail_count;
    }
#else

    return ;
#endif
}


static int _sell_disp_tickets(struct rpusbdisp_dev * dev, struct rpusbdisp_disp_ticket_bundle * bundle, size_t required_count)
{
    unsigned long irq_flags;
    int ans = 0;
    struct list_head *node;
    // do not sell tickets when the device has been closed
    if(!dev->is_alive) return 0;
    if(required_count == 0) {
        printk("required for zero ?!\n");
        return 0;
    }
    spin_lock_irqsave(&dev->disp_tickets_pool.oplock, irq_flags);
    do {
        if(required_count > dev->disp_tickets_pool.availiable_count) {
            // no enough tickets availiable
            break;
        }

        dev->disp_tickets_pool.availiable_count -= required_count;

        bundle->ticket_count = required_count;
        ans = bundle->ticket_count;

        INIT_LIST_HEAD(&bundle->ticket_list);
        while(required_count--) {
            node = dev->disp_tickets_pool.list.next;
            list_del_init(node);
            list_add_tail(node, &bundle->ticket_list);
        }
    } while(0);
    spin_unlock_irqrestore(&dev->disp_tickets_pool.oplock, irq_flags);
    return ans;
}


static int _return_disp_tickets(struct rpusbdisp_dev * dev,  struct  rpusbdisp_disp_ticket * ticket)
{
    int all_finished = 0;
    unsigned long  irq_flags;
    // insert to the available queue

    spin_lock_irqsave(&dev->disp_tickets_pool.oplock, irq_flags);
    list_add_tail(&ticket->ticket_list_node, &dev->disp_tickets_pool.list);

    if(++dev->disp_tickets_pool.availiable_count == dev->disp_tickets_pool.disp_urb_count) {
        all_finished = 1;
    }

    spin_unlock_irqrestore(&dev->disp_tickets_pool.oplock, irq_flags);


    wake_up(&dev->disp_tickets_pool.wait_queue);
    return all_finished;
}


int rpusbdisp_usb_try_copy_area(struct rpusbdisp_dev * dev, int sx, int sy, int dx, int dy, int width, int height)
{

    struct  rpusbdisp_disp_ticket_bundle bundle;
    struct  rpusbdisp_disp_ticket * ticket;
    rpusbdisp_disp_copyarea_packet_t * cmd_copyarea;
    // only one ticket is enough
    if(!_sell_disp_tickets(dev, &bundle, 1)) {
        // tickets is inadequate, try next time
        return 0;
    }

    BUG_ON(sizeof(rpusbdisp_disp_copyarea_packet_t) > dev->disp_out_ep_max_size);

    ticket = list_entry(bundle.ticket_list.next, struct  rpusbdisp_disp_ticket, ticket_list_node);

    cmd_copyarea = (rpusbdisp_disp_copyarea_packet_t *)ticket->transfer_urb->transfer_buffer;
    cmd_copyarea->header.cmd_flag = RPUSBDISP_DISPCMD_COPY_AREA | RPUSBDISP_CMD_FLAG_START;

    cmd_copyarea->sx = cpu_to_le16(sx);
    cmd_copyarea->sy = cpu_to_le16(sy);
    cmd_copyarea->dx = cpu_to_le16(dx);
    cmd_copyarea->dy = cpu_to_le16(dy);

    cmd_copyarea->width = cpu_to_le16(width);
    cmd_copyarea->height = cpu_to_le16(height);

    //add one more byte to bypass usbdisp 1.03 fw bug
    ticket->transfer_urb->transfer_buffer_length = sizeof(rpusbdisp_disp_copyarea_packet_t) + 1;

    if(usb_submit_urb(ticket->transfer_urb, GFP_KERNEL)) {
        // submit failure,
        _on_display_transfer_finished(ticket->transfer_urb);
        return 0;
    }

    return 1;

}

int rpusbdisp_usb_try_draw_rect(struct rpusbdisp_dev * dev, int x, int y, int right, int bottom,  pixel_type_t color, int operation)
{
    rpusbdisp_disp_fillrect_packet_t * cmd_fillrect;
    struct  rpusbdisp_disp_ticket_bundle bundle;
    struct  rpusbdisp_disp_ticket * ticket;

    // only one ticket is enough
    if(!_sell_disp_tickets(dev, &bundle, 1)) {
        // tickets is inadequate, try next time
        return 0;
    }

    BUG_ON(sizeof(rpusbdisp_disp_fillrect_packet_t) > dev->disp_out_ep_max_size);

    ticket = list_entry(bundle.ticket_list.next, struct  rpusbdisp_disp_ticket, ticket_list_node);

    cmd_fillrect = (rpusbdisp_disp_fillrect_packet_t *)ticket->transfer_urb->transfer_buffer;

    cmd_fillrect->header.cmd_flag = RPUSBDISP_DISPCMD_RECT | RPUSBDISP_CMD_FLAG_START;

    cmd_fillrect->left = cpu_to_le16(x);
    cmd_fillrect->top = cpu_to_le16(y);
    cmd_fillrect->right = cpu_to_le16(right);
    cmd_fillrect->bottom = cpu_to_le16(bottom);

    cmd_fillrect->color_565 = cpu_to_le16(color);
    cmd_fillrect->operation = operation;

    ticket->transfer_urb->transfer_buffer_length = sizeof(rpusbdisp_disp_fillrect_packet_t);

    if(usb_submit_urb(ticket->transfer_urb, GFP_KERNEL)) {
        // submit failure,
        _on_display_transfer_finished(ticket->transfer_urb);
        return 0;
    }

    return 1;

}





// context used by the bitblt display packet encoder
struct bitblt_encoding_context_t {
    struct  rpusbdisp_disp_ticket_bundle bundle;
    struct  rpusbdisp_disp_ticket * ticket;
    struct  list_head * current_node;

    size_t  encoded_pos;
    size_t  packet_pos;
    _u8     * urbbuffer ;

} ;

static uint16_t gfid = 0;


static int _bitblt_encode_command_header(uint8_t * msg, int x, int y, int right, int bottom,  uint8_t op_flg)
{
    usbdisp_disp_bitblt_packet_t * bitblt_header ;


    // encoding the command header...

    bitblt_header = (usbdisp_disp_bitblt_packet_t *)msg;
    bitblt_header->header.cmd_flag = op_flg;
    bitblt_header->header.cmd_flag |= USBDISP_CMD_FLAG_START;

    bitblt_header->padding = 0;
    bitblt_header->x = cpu_to_le16(x);
    bitblt_header->y = cpu_to_le16(y);
    bitblt_header->width = cpu_to_le16(right + 1 - x);
    bitblt_header->height = cpu_to_le16(bottom + 1 - y);
    bitblt_header->operation = 0;


    return sizeof(usbdisp_disp_bitblt_packet_t);

}

static int _bitblt_encode_command_header_total_bytes(uint8_t * msg, int total_bytes)
{
    usbdisp_disp_bitblt_packet_t * bitblt_header ;
    bitblt_header = (usbdisp_disp_bitblt_packet_t *)msg;
    bitblt_header->total_bytes = total_bytes;
    return sizeof(usbdisp_disp_bitblt_packet_t);
}

static void _bitblt_encoder_cleanup(struct bitblt_encoding_context_t * ctx, struct rpusbdisp_dev * dev)
{
    struct  rpusbdisp_disp_ticket * ticket;

    // return unused tickets
    while(ctx->current_node != &ctx->bundle.ticket_list) {

        ticket = list_entry(ctx->current_node, struct  rpusbdisp_disp_ticket, ticket_list_node);
        ctx->current_node = ctx->current_node->next;

        _return_disp_tickets(dev, ticket);
    }

}

static int _bitblt_encoder_flush(struct bitblt_encoding_context_t * ctx, struct rpusbdisp_dev * dev)
{


    // submit the final ticket

    size_t transfer_size = ctx->packet_pos * dev->disp_out_ep_max_size + ctx->encoded_pos;

    if(transfer_size) {
        ctx->ticket->transfer_urb->transfer_buffer_length = transfer_size;

        ctx->current_node = ctx->current_node->next;
        if(usb_submit_urb(ctx->ticket->transfer_urb, GFP_KERNEL)) {
            // submit failure,

            _on_display_transfer_finished(ctx->ticket->transfer_urb);
            LOGW("submit urb NG\n");
            return -1; //abort
        }
    }


    _bitblt_encoder_cleanup(ctx, dev);

    return 0;
}


static int _bitblt_encode_n_transfer_data(struct bitblt_encoding_context_t * ctx, struct rpusbdisp_dev * dev, const void * data, size_t count)
{
    const _u8 * payload_in_bytes = (const _u8 *)data;

    while(count) {
        // fill the buffer as much as possible...
        size_t buffer_avail_length = dev->disp_out_ep_max_size - ctx->encoded_pos;
        size_t size_to_copy = count > buffer_avail_length ? buffer_avail_length : count;

        memcpy(ctx->urbbuffer + ctx->encoded_pos, payload_in_bytes, size_to_copy);
        payload_in_bytes += size_to_copy;
        ctx->encoded_pos += size_to_copy;


        count -= size_to_copy;

        if(buffer_avail_length == size_to_copy) {
            // current transfer block is full
            ctx->urbbuffer += dev->disp_out_ep_max_size;


            if(++ctx->packet_pos >= dev->disp_tickets_pool.packet_size_factor) {
                // current urb is full, send the ticket
                ctx->current_node = ctx->current_node->next;
                if(usb_submit_urb(ctx->ticket->transfer_urb, GFP_KERNEL)) {
                    // submit failure,

                    _on_display_transfer_finished(ctx->ticket->transfer_urb);
                    LOGW("submit usr NG\n");
                    return -1; //abort
                }


                // a new ticket
                ctx->packet_pos = 0;

                BUG_ON(ctx->current_node == &ctx->bundle.ticket_list);

                ctx->ticket = list_entry(ctx->current_node, struct  rpusbdisp_disp_ticket, ticket_list_node);
                ctx->urbbuffer = (_u8 *)ctx->ticket->transfer_urb->transfer_buffer;

            }

            // encoding the header
            *ctx->urbbuffer = 0;
            ((usbdisp_disp_packet_header_t *)ctx->urbbuffer)->cmd_flag = RPUSBDISP_DISPCMD_BITBLT;
            ctx->encoded_pos = sizeof(usbdisp_disp_packet_header_t);

        }
    }

    return 0;
}





int encoder_ctx_init(struct bitblt_encoding_context_t * ctx, struct rpusbdisp_dev * dev, int msg_size)
{

    int    required_tickets_count, effective_payload_per_packet_size, packet_count;
    int max_retry_tickets = 3;

    // do not transmit zero size image
    if(!msg_size) return -1;

    effective_payload_per_packet_size = dev->disp_out_ep_max_size - sizeof(usbdisp_disp_packet_header_t);
    packet_count = (msg_size + effective_payload_per_packet_size - 1) / effective_payload_per_packet_size;
    required_tickets_count = (packet_count + dev->disp_tickets_pool.packet_size_factor - 1) / dev->disp_tickets_pool.packet_size_factor;

    if(required_tickets_count > RPUSBDISP_MAX_TRANSFER_TICKETS_COUNT) {
        err("required_tickets_count (%d)>RPUSBDISP_MAX_TRANSFER_TICKETS_COUNT(%d)\n", required_tickets_count, RPUSBDISP_MAX_TRANSFER_TICKETS_COUNT);
        // BUG_ON(1);
        return -1;
    }

    {
retry:

    if(!_sell_disp_tickets(dev, &ctx->bundle, required_tickets_count)) {
        // tickets is inadequate, try next time
            if(max_retry_tickets) {
                msleep(10);
                max_retry_tickets--;
                goto retry;
            }
            LOGW("%s %d tickets is inadequate\n", __FUNCTION__, required_tickets_count);
            return -1;
    }
    }
    // init the context...
    ctx->packet_pos = 0;
    ctx->current_node = ctx->bundle.ticket_list.next;
    ctx->ticket = list_entry(ctx->current_node, struct	rpusbdisp_disp_ticket, ticket_list_node);
    ctx->urbbuffer = (_u8 *)ctx->ticket->transfer_urb->transfer_buffer;
    ctx->encoded_pos = 0;

    return 0;

}



int rpusbdisp_usb_send_msg(struct rpusbdisp_dev * dev, const uint8_t * msg_data, int msg_size)
{
    struct bitblt_encoding_context_t encoder_ctx;


    if(encoder_ctx_init(&encoder_ctx, dev, msg_size) < 0) return -1;


    if(_bitblt_encode_n_transfer_data(&encoder_ctx, dev, msg_data, msg_size) < 0) {
        // abort the operation...

        _bitblt_encoder_cleanup(&encoder_ctx, dev);
        return -2;
    }


    return _bitblt_encoder_flush(&encoder_ctx, dev);


}
long get_os_us(void)
{
    struct timespec ts;
    ts = current_kernel_time();
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}
long get_system_us(void)
{
    return get_os_us();
}
long gta, gtb;
#define FPS_STAT_MAX 6
typedef struct {
    long tb[FPS_STAT_MAX];
    int cur;
    long last_fps;
} fps_mgr_t;
fps_mgr_t fps_mgr = {
    .cur = 0,
    .last_fps = -1,
};
long get_fps(void)
{
    fps_mgr_t * mgr = &fps_mgr;
    if(mgr->cur < FPS_STAT_MAX)//we ignore first loop and also ignore rollback case due to a long period
        return mgr->last_fps;//if <0 ,please ignore it
    else {
        long a = mgr->tb[(mgr->cur-1)%FPS_STAT_MAX];
        long b = mgr->tb[(mgr->cur)%FPS_STAT_MAX];
        long fps = (a - b) / (FPS_STAT_MAX - 1);
        fps = (1000000 * 10) / fps; //x10 for int
        mgr->last_fps = fps;
        return fps;
    }
}
void put_fps_data(long t)
{
    fps_mgr_t * mgr = &fps_mgr;
    mgr->tb[mgr->cur%FPS_STAT_MAX] = t;
    mgr->cur++;//cur ptr to next
}
int rpusbdisp_usb_try_send_jpeg_image(struct rpusbdisp_dev * dev, const pixel_type_t * framebuffer, int x, int y, int right, int bottom, int line_width, int clear_dirty)
{
    int last_copied_x, last_copied_y;
    int ret = 0;
    int pos = 0;
    uint8_t * pix_msg;
	stream_mgr_t m_mgr;
    stream_mgr_t * mgr = &m_mgr;
    uint16_t total_bytes = 0;
	int msg_pos=0;
	uint8_t * msg=dev->msg;
	uint8_t * urb_msg=dev->sg_buf;
	int jpg_quality=dev->jpg_quality;
	int target_quaility_size=12*1024;


    // estimate how many tickets are needed
    size_t image_size = (right - x + 1) * (bottom - y + 1) * (RP_DISP_DEFAULT_PIXEL_BITS / 8);

    // do not transmit zero size image
    if(!image_size) return -1;


    msg_pos = _bitblt_encode_command_header(msg, x, y, right, bottom, USBDISP_CMD_BITBLT_JPEG);

    pix_msg = &msg[msg_pos];
    // locate to the begining...
    framebuffer += (y * line_width + x);

    for(last_copied_y = y; last_copied_y <= bottom; ++last_copied_y) {

        for(last_copied_x = x; last_copied_x <= right; ++last_copied_x) {

#ifdef PIXEL_32BIT
            pixel_type_t pix = *framebuffer;
            uint8_t r, g, b;
            b = pix & 0xff;
            g = (pix >> 8) & 0xff;
            r = (pix >> 16) & 0xff;

            *pix_msg++ = r;
            *pix_msg++ = g;
            *pix_msg++ = b;
            msg_pos += 3;

#else
            pixel_type_t pix = *framebuffer;
            uint8_t r, g, b;
            b = (pix << 3) & 0xf8;
            g = (pix >> 3) & 0xfc;
            r = (pix >> 8) & 0xf8;
            *pix_msg++ = r;
            *pix_msg++ = g;
            *pix_msg++ = b;
            msg_pos += 3;

#endif


            ++framebuffer;

        }
        framebuffer += line_width - right - 1 + x;
    }
	long fps=get_fps();
	int low_quality=-1;;
	int high_quality=-1;
	redo:
    {
        //ok we use jpeg transfer data

        mgr->data =urb_msg;
        mgr->max = JPEG_MAX_SIZE;
        mgr->dp = 0;
        if(!tje_encode_to_ctx(mgr, (right - x + 1), (bottom - y + 1), 3, &msg[sizeof(usbdisp_disp_bitblt_packet_t)], jpg_quality)) {
            LOG("Could not encode JPEG\n");
        }
    }
	LOG("jpg: total:%d %d\n",  mgr->dp, jpg_quality);
	if(fps>=60)//we think this may video case
	{
	#define ABS(x) ((x) < 0 ? -(x) : (x))

		if(ABS(mgr->dp - target_quaility_size)<2*1024)
					goto next;
		if(high_quality>0 && low_quality>0)
			goto next;
		if(mgr->dp > target_quaility_size) {
			high_quality=jpg_quality;
			if(jpg_quality>=2){
				jpg_quality--;
				goto redo;
    }
	   	}else {
	   		low_quality=jpg_quality;
				if(jpg_quality<=5) {
						jpg_quality++;
						goto redo;
					}
	   	}		   
	}
	else {
			jpg_quality=5;
		}
	next:
	dev->jpg_quality=jpg_quality;
    memcpy(&msg[sizeof(usbdisp_disp_bitblt_packet_t)], mgr->data, mgr->dp);

    total_bytes = sizeof(usbdisp_disp_bitblt_packet_t) + mgr->dp;
    _bitblt_encode_command_header_total_bytes(msg, total_bytes);
    //
    ret = rpusbdisp_usb_send_msg(dev, dev->msg, total_bytes);
    if(ret >= 0) {
        put_fps_data(get_system_us());
	}
    LOG("jpg:%d %d %d %d %d total:%d (%d) fid:%d fps:%d(x10)\n", ret, x, y, right, bottom, total_bytes, dev->msg_pos, gfid, get_fps());
    	
    return ret;


}

pixel_type_t lcd_color_test_pattern[RP_DISP_DEFAULT_HEIGHT*RP_DISP_DEFAULT_WIDTH] = {0};

#define TEST_COLOR_RED 0xff
#define TEST_COLOR_GREEN 0xff00
#define TEST_COLOR_BLUE 0xff0000

void fill_color(pixel_type_t *buf, int len, pixel_type_t color)
{
    int i = 0;
    for(i = 0; i <= len; i++) {
        buf[i] = color;
    }
    //dump_memory_bytes("-",buf,16);
}


void fill_color_bar(pixel_type_t *buf, int len)
{

    fill_color(buf, len / 3, TEST_COLOR_RED);
    fill_color(&buf[len/3], len / 3, TEST_COLOR_GREEN);
    fill_color(&buf[(len*2)/3], len / 3, TEST_COLOR_BLUE);

}

#define rgb565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

int rpusbdisp_usb_try_send_image(struct rpusbdisp_dev * dev, const pixel_type_t * framebuffer, int x, int y, int right, int bottom, int line_width, int clear_dirty)
{
    int    last_copied_x, last_copied_y;
    int pos = 0;
    uint16_t * pix_msg;
    _u32 total_bytes = 0;



    // estimate how many tickets are needed
    const size_t image_size = (right - x + 1) * (bottom - y + 1) * (RP_DISP_DEFAULT_PIXEL_BITS / 8);

    // do not transmit zero size image
    if(!image_size) return 1;

#define JPEG_TRANSFER_SIZE (10*1024)
    if(image_size > JPEG_TRANSFER_SIZE) { //ok we use jpeg transfer data
#if 1
        return   rpusbdisp_usb_try_send_jpeg_image(dev, framebuffer, x, y, right, bottom, line_width, clear_dirty);
#else
        fill_color_bar(lcd_color_test_pattern, RP_DISP_DEFAULT_HEIGHT * RP_DISP_DEFAULT_WIDTH);
        return rpusbdisp_usb_try_send_jpeg_image(dev, lcd_color_test_pattern, x, y, right, bottom, line_width, clear_dirty);

#endif
    }


    dev->msg_pos = _bitblt_encode_command_header(dev->msg, x, y, right, bottom,  RPUSBDISP_DISPCMD_BITBLT);
    pix_msg = (uint16_t *)&dev->msg[dev->msg_pos];
    // locate to the begining...
    framebuffer += (y * line_width + x);

    for(last_copied_y = y; last_copied_y <= bottom; ++last_copied_y) {

        for(last_copied_x = x; last_copied_x <= right; ++last_copied_x) {

#ifdef PIXEL_32BIT
            pixel_type_t pix = *framebuffer;
            uint8_t r, g, b;
            b = pix & 0xff;
            g = (pix >> 8) & 0xff;
            r = (pix >> 16) & 0xff;
            uint16_t current_pixel_le = rgb565(r, g, b);
            current_pixel_le = (current_pixel_le >> 8) | (current_pixel_le << 8);
            *pix_msg = current_pixel_le;
            pix_msg++;
            dev->msg_pos += sizeof(uint16_t);

#else
            pixel_type_t current_pixel_le = cpu_to_le16(*framebuffer);
			current_pixel_le = (current_pixel_le >> 8) | (current_pixel_le << 8);
            *pix_msg = current_pixel_le;
            pix_msg++;
            dev->msg_pos += sizeof(pixel_type_t);

#endif


            ++framebuffer;

        }
        framebuffer += line_width - right - 1 + x;
    }

    total_bytes = dev->msg_pos;
    _bitblt_encode_command_header_total_bytes(dev->msg, total_bytes);
    put_fps_data(get_system_us());
    LOG("%s %d %d %d %d total_bytes:%d (%d) fps:%d(x10)\n", __FUNCTION__, x, y, right, bottom, total_bytes, dev->msg_pos, get_fps());
    return rpusbdisp_usb_send_msg(dev, dev->msg, dev->msg_pos);


}


static void _on_release_disp_tickets_pool(struct rpusbdisp_dev * dev)
{
    unsigned long irq_flags;
    struct rpusbdisp_disp_ticket * ticket;
    struct list_head *node;
    int tickets_count = dev->disp_tickets_pool.disp_urb_count;
    DEFINE_WAIT(wait);

    dev_info(&dev->interface->dev, "waiting for all tickets to be finished...\n");



    while(tickets_count) {

        spin_lock_irqsave(&dev->disp_tickets_pool.oplock, irq_flags);
        if(dev->disp_tickets_pool.availiable_count) {
            --dev->disp_tickets_pool.availiable_count;
        } else {
            spin_unlock_irqrestore(&dev->disp_tickets_pool.oplock, irq_flags);
            // sleep_on_timeout(&dev->disp_tickets_pool.wait_queue, 2*HZ);

            prepare_to_wait(&dev->disp_tickets_pool.wait_queue, &wait, TASK_UNINTERRUPTIBLE);
            schedule_timeout(2 * HZ);
            finish_wait(&dev->disp_tickets_pool.wait_queue, &wait);
            continue;
        }
        node = dev->disp_tickets_pool.list.next;
        list_del_init(node);
        spin_unlock_irqrestore(&dev->disp_tickets_pool.oplock, irq_flags);

        ticket = list_entry(node, struct rpusbdisp_disp_ticket, ticket_list_node);

        usb_free_coherent(ticket->transfer_urb->dev, RPUSBDISP_MAX_TRANSFER_SIZE,
                          ticket->transfer_urb->transfer_buffer, ticket->transfer_urb->transfer_dma);

        usb_free_urb(ticket->transfer_urb);
        kfree(ticket);
        --tickets_count;
    }

}

static int _on_alloc_disp_tickets_pool(struct rpusbdisp_dev * dev)
{
    struct  rpusbdisp_disp_ticket * newborn;
    int     actual_allocated = 0;
    _u8   * transfer_buffer;
    size_t  packet_size_factor;
    size_t  ticket_logic_size;

    packet_size_factor = (RPUSBDISP_MAX_TRANSFER_SIZE / dev->disp_out_ep_max_size) ;
    ticket_logic_size = packet_size_factor * dev->disp_out_ep_max_size ;


    spin_lock_init(&dev->disp_tickets_pool.oplock);
    INIT_LIST_HEAD(&dev->disp_tickets_pool.list);

    while(actual_allocated < RPUSBDISP_MAX_TRANSFER_TICKETS_COUNT) {
        newborn = kzalloc(sizeof(struct rpusbdisp_disp_ticket), GFP_KERNEL);
        if(!newborn) {
            break;
        }

        newborn->transfer_urb = usb_alloc_urb(0, GFP_KERNEL);
        if(!newborn->transfer_urb) {
            kfree(newborn);
            break;
        }



        transfer_buffer = usb_alloc_coherent(dev->udev, RPUSBDISP_MAX_TRANSFER_SIZE, GFP_KERNEL, &newborn->transfer_urb->transfer_dma);
        if(!transfer_buffer) {
            usb_free_urb(newborn->transfer_urb);
            kfree(newborn);
            break;
        }


        // setup urb
        usb_fill_bulk_urb(newborn->transfer_urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->disp_out_ep_addr),
                          transfer_buffer, ticket_logic_size, _on_display_transfer_finished, newborn);
        newborn->transfer_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;


        newborn->binded_dev = dev;

        dev_info(&dev->interface->dev, "allocated ticket %p with urb %p\n", newborn, newborn->transfer_urb);


        list_add_tail(&newborn->ticket_list_node, &dev->disp_tickets_pool.list);

        ++actual_allocated;

    }

    INIT_DELAYED_WORK(&dev->disp_tickets_pool.completion_work, _on_display_transfer_finished_delaywork);

    init_waitqueue_head(&dev->disp_tickets_pool.wait_queue);
    dev->disp_tickets_pool.disp_urb_count = actual_allocated;
    dev->disp_tickets_pool.availiable_count = actual_allocated;
    dev->disp_tickets_pool.packet_size_factor = packet_size_factor;
    dev_info(&dev->interface->dev, "allocated %d urb tickets for transfering display data. %lu size each\n", actual_allocated, ticket_logic_size);

    return actual_allocated ? 0 : -ENOMEM;
};

static int _on_new_usb_device(struct rpusbdisp_dev * dev)
{
    // the rp-usb-display device has been verified
    mutex_init(&dev->op_locker);
    init_waitqueue_head(&dev->status_wait_queue);


    dev->urb_status_query = usb_alloc_urb(0, GFP_KERNEL);
    if(! dev->urb_status_query) {
        dev_info(&dev->interface->dev, "Cannot allocate status query urbs.\n");
        goto status_urb_alloc_fail;
    }

    if(_on_alloc_disp_tickets_pool(dev) < 0) {
        dev_info(&dev->interface->dev, "Cannot allocate the display tickets pool.\n");
        goto disp_tickets_alloc_fail;
    }


    _add_usbdev_to_list(dev);
    dev->dev_id = rpusbdisp_usb_get_device_count();
    dev->is_alive = 1;
    dev->device_fwver = le16_to_cpu(dev->udev->descriptor.bcdDevice);

    dev_info(&dev->interface->dev, "RP USB Display found (#%d), Firmware Version: %d.%02d, S/N: %s\n",
             dev->dev_id,
             (dev->device_fwver >> 8),
             (dev->device_fwver & 0xFF),
             dev->udev->serial ? dev->udev->serial : "(unknown)");


    // start status querying...
    _status_start_querying(dev);

	register_fb_handlers();
    fbhandler_on_new_device(dev);
    touchhandler_on_new_device(dev);


    // force all the image to be flush
    fbhandler_set_unsync_flag(dev);
    schedule_delayed_work(&dev->disp_tickets_pool.completion_work, 0);
    return 0;

disp_tickets_alloc_fail:
    usb_free_urb(dev->urb_status_query);

status_urb_alloc_fail:
    return -ENOMEM;
}


static void _on_del_usb_device(struct rpusbdisp_dev * dev)
{

    mutex_lock(&dev->op_locker);
    dev->is_alive = 0;
    mutex_unlock(&dev->op_locker);

    touchhandler_on_remove_device(dev);
    fbhandler_on_remove_device(dev);
	unregister_fb_handlers();

    // kill all pending urbs
    usb_kill_urb(dev->urb_status_query);
    cancel_delayed_work_sync(&dev->disp_tickets_pool.completion_work);




    _del_usbdev_from_list(dev);



    _on_release_disp_tickets_pool(dev);

    usb_free_urb(dev->urb_status_query);
    dev->urb_status_query = NULL;

    dev_info(&dev->interface->dev, "RP USB Display (#%d) now disconnected\n", dev->dev_id);
}


int rpusbdisp_usb_get_device_count(void)
{
    return atomic_read(&_devlist_count);
}

static int rpusbdisp_probe(struct usb_interface *interface, const struct usb_device_id *id)
{

    struct rpusbdisp_dev *dev = NULL;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    size_t buffer_size;
    int i;
    int retval = -ENOMEM;
    printk("in %s\n", __FUNCTION__);
    /* allocate memory for our device state and initialize it */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if(dev == NULL) {
        err("Out of memory");
        goto error;
    }


    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

    if(le16_to_cpu(dev->udev->descriptor.bcdDevice) > 0x0200) {
        dev_warn(&interface->dev, "The device you used may requires a newer driver version to work.\n");
    }

    // check for endpoints
    iface_desc = interface->cur_altsetting;

    for(i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
        endpoint = &iface_desc->endpoint[i].desc;

        if(!dev->status_in_ep_addr &&
           usb_endpoint_is_bulk_in(endpoint)) {
            //usb_endpoint_is_int_in(endpoint)) {
            LOG("the status input endpoint has been found\n");
            buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
            dev->status_in_ep_addr = endpoint->bEndpointAddress;

        }

        if(!dev->disp_out_ep_addr &&
           usb_endpoint_is_bulk_out(endpoint) && endpoint->wMaxPacketSize) {
            LOG("endpoint for video output has been found\n");
            dev->disp_out_ep_addr = endpoint->bEndpointAddress;
            dev->disp_out_ep_max_size = le16_to_cpu(endpoint->wMaxPacketSize);

        }
    }

    if(!(dev->status_in_ep_addr && dev->disp_out_ep_addr)) {
        err("Could not find the required endpoints\n");
        goto error;
    }


    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    // add the device to the list
    if(_on_new_usb_device(dev)) {
        goto error;
    }


	dev->jpg_quality=5;
#if 0

    /* we can register the device now, as it is ready */
    retval = usb_register_dev(interface, &rpusbdisp_class);
    if(retval) {
        /* something prevented us from registering this driver */
        err("Not able to get a minor for this device.");
        usb_set_intfdata(interface, NULL);
        goto error;
    }



    /* let the user know what node this device is now attached to */
    dev_info(&interface->dev, "The RP USB Display device now attached to RP_USBDISP-%d\n",
             interface->minor);
#endif

    return 0;

error:
    if(dev) {
        kfree(dev);
    }
    return retval;
}


#if 0
static void lcd_draw_down(struct usb_lcd *dev)
{
    int time;

    time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
    if(!time)
        usb_kill_anchored_urbs(&dev->submitted);
}
#endif

static int rpusbdisp_suspend(struct usb_interface *intf, pm_message_t message)
{
    struct usb_lcd *dev = usb_get_intfdata(intf);

    if(!dev)
        return 0;
    // not implemented yet
    return 0;
}


static int rpusbdisp_resume(struct usb_interface *intf)
{
    // not implemented yet
    return 0;
}

static void rpusbdisp_disconnect(struct usb_interface *interface)
{
    struct rpusbdisp_dev *dev;

    dev = usb_get_intfdata(interface);
    usb_set_intfdata(interface, NULL);
    _on_del_usb_device(dev);

    kfree(dev);
}


static struct usb_driver usbdisp_driver = {
    .name       =  RP_DISP_DRIVER_NAME,
    .probe      =  rpusbdisp_probe,
    .disconnect =  rpusbdisp_disconnect,
    .suspend    =  rpusbdisp_suspend,
    .resume     =  rpusbdisp_resume,
    .id_table   =  id_table,
    .supports_autosuspend = 0,
};

int __init register_usb_handlers(void)
{
    // create the status polling task

    _working_flag = 1;
#if 0
    _usb_status_polling_task = kthread_run(_kthread_usb_status_poller_proc, NULL, "rpusbdisp_worker%d", 0);
    if(IS_ERR(_usb_status_polling_task)) {
        err("Cannot create the kernel worker thread!\n");
        return -ENOMEM;
    }
#endif
    return usb_register(&usbdisp_driver);
}


void unregister_usb_handlers(void)
{

    // cancel the worker thread
    _working_flag = 0;
    wake_up(&_usblist_waitqueue);
#if 0
    if(!IS_ERR(_usb_status_polling_task)) {
        kthread_stop(_usb_status_polling_task);
    }
#endif

    usb_deregister(&usbdisp_driver);
}


#if 0

static int _kthread_usb_status_poller_proc(void *data)
{
    struct list_head *pos, *q;

    while(_working_flag) {

        if(rpusbdisp_usb_get_device_count() < 1) {
            // nothing to do , sleep...
            sleep_on(&_usblist_waitqueue);
            continue;
        }


        // send usb msg to query the status

        list_for_each_safe(pos, q, &rpusbdisp_list) {
            struct rpusbdisp_dev *current_dev;
            current_dev = list_entry(pos, struct rpusbdisp_dev, dev_list_node);


            // send urbs

        }


    }

    return 0;
}

#endif

