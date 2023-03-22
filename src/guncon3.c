// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Namco GunCon 2 USB light gun
 * Copyright (C) 2019-2021 beardypig <beardypig@protonmail.com>
 *
 * Based largely on the PXRC driver by Marcus Folkesson <marcus.folkesson@gmail.com>
 *
 */
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/types.h>

static unsigned long debug = 0;
module_param(debug, ulong, 0444);
MODULE_PARM_DESC(debug, "Enable Debugging");

#define NAMCO_VENDOR_ID    0x0b9a
#define GUNCON3_PRODUCT_ID 0x0800


//data[10] = A3_Stick B3_Stick 0          0           0               0  0  0
//data[11] = C1       0       Trigger   Out_range   one_reference   B1 B2 0
//data[12] = 0        0       0         0           C2              A1 A2 0

/*
     input_report_key(dev, BTN_TRIGGER, (data[1] & 0x20));
    input_report_key(dev, BTN_0, (data[0] & 0x04));    // A
    input_report_key(dev, BTN_1, (data[0] & 0x02));
    input_report_key(dev, BTN_2, (data[1] & 0x04));    // B
    input_report_key(dev, BTN_3, (data[1] & 0x02));
    input_report_key(dev, BTN_4, (data[1] & 0x80));    // C
    input_report_key(dev, BTN_5, (data[0] & 0x08));
    input_report_key(dev, BTN_6, (data[2] & 0x80));    // Joystick buttons
    input_report_key(dev, BTN_7, (data[2] & 0x40));
    */
#define GUNCON3_TRIGGER             0x00002000
#define GUNCON3_BTN_A1              0x00000004
#define GUNCON3_BTN_A2              0x00000002
#define GUNCON3_BTN_A3              0x00800000
#define GUNCON3_BTN_B1              0x00000400
#define GUNCON3_BTN_B2              0x00000200
#define GUNCON3_BTN_B3              0x00400000
#define GUNCON3_BTN_C1              0x00008000
#define GUNCON3_BTN_C2              0x00000008
#define GUNCON3_BTN_OUT_RANGE       0x00000800
#define GUNCON3_BTN_ONE_REFERENCE   0x00000100

#define GUNCON3_BTN_SELECT  GUNCON3_BTN_C1
#define GUNCON3_BTN_START   GUNCON3_BTN_C2
#define GUNCON3_BTN_A       GUNCON3_BTN_B2
#define GUNCON3_BTN_B       GUNCON3_BTN_B1
#define GUNCON3_BTN_C       GUNCON3_BTN_A2


// default calibration, can be updated with evdev-joystick
#define X_MIN -32768
#define X_MAX 32767
#define Y_MIN -32768
#define Y_MAX 32767

struct guncon3 {
    struct input_dev *input_device;
    struct usb_interface *intf;
    struct urb *urb;
    struct mutex pm_mutex;
    bool is_open;
    char phys[64];
    bool is_recalibrate;
    unsigned char key[8];
};

struct gc_mode {
    unsigned short a;
    unsigned char b;
    unsigned char c;
    unsigned char d;
    unsigned char mode;
};

const unsigned char key[8]={0x01,0x12,0x6f,0x32,0x24,0x60,0x17,0x21};
static const unsigned char KEY_TABLE[320] = {
        0x75, 0xC3, 0x10, 0x31, 0xB5, 0xD3, 0x69, 0x84, 0x89, 0xBA, 0xD6, 0x89, 0xBD, 0x70, 0x19, 0x8E, 0x58, 0xA8,
        0x3D, 0x9B, 0x5D, 0xF0, 0x49, 0xE8, 0xAD, 0x9D, 0x7A, 0x0D, 0x7E, 0x24, 0xDA, 0xFC, 0x0D, 0x14, 0xC5, 0x23,
        0x91, 0x11, 0xF5, 0xC0, 0x4B, 0xCD, 0x44, 0x1C, 0xC5, 0x21, 0xDF, 0x61, 0x54, 0xED, 0xA2, 0x81, 0xB7, 0xE5,
        0x74, 0x94, 0xB0, 0x47, 0xEE, 0xF1, 0xA5, 0xBB, 0x21, 0xC8, 0x91, 0xFD, 0x4C, 0x8B, 0x20, 0xC1, 0x7C, 0x09, 0x58,
        0x14, 0xF6, 0x00, 0x52, 0x55, 0xBF, 0x41, 0x75, 0xC0, 0x13, 0x30, 0xB5, 0xD0, 0x69, 0x85, 0x89, 0xBB, 0xD6, 0x88,
        0xBC, 0x73, 0x18, 0x8D, 0x58, 0xAB, 0x3D, 0x98, 0x5C, 0xF2, 0x48, 0xE9, 0xAC, 0x9F, 0x7A, 0x0C, 0x7C, 0x25, 0xD8,
        0xFF, 0xDC, 0x7D, 0x08, 0xDB, 0xBC, 0x18, 0x8C, 0x1D, 0xD6, 0x3C, 0x35, 0xE1, 0x2C, 0x14, 0x8E, 0x64, 0x83, 0x39,
        0xB0, 0xE4, 0x4E, 0xF7, 0x51, 0x7B, 0xA8, 0x13, 0xAC, 0xE9, 0x43, 0xC0, 0x08, 0x25, 0x0E, 0x15, 0xC4, 0x20, 0x93,
        0x13, 0xF5, 0xC3, 0x48, 0xCC, 0x47, 0x1C, 0xC5, 0x20, 0xDE, 0x60, 0x55, 0xEE, 0xA0, 0x40, 0xB4, 0xE7, 0x74,
        0x95, 0xB0, 0x46, 0xEC, 0xF0, 0xA5, 0xB8, 0x23, 0xC8, 0x04, 0x06, 0xFC, 0x28, 0xCB, 0xF8, 0x17, 0x2C, 0x25, 0x1C,
        0xCB, 0x18, 0xE3, 0x6C, 0x80, 0x85, 0xDD, 0x7E, 0x09, 0xD9, 0xBC, 0x19, 0x8F, 0x1D, 0xD4, 0x3D, 0x37, 0xE1, 0x2F,
        0x15, 0x8D, 0x64, 0x06, 0x04, 0xFD, 0x29, 0xCF, 0xFA, 0x14, 0x2E, 0x25, 0x1F, 0xC9, 0x18, 0xE3, 0x6D, 0x81, 0x84,
        0x80, 0x3B, 0xB1, 0xE5, 0x4D, 0xF7, 0x51, 0x78, 0xA9, 0x13, 0xAD, 0xE9, 0x80, 0xC1, 0x0B, 0x25, 0x93, 0xFC,
        0x4D, 0x89, 0x23, 0xC2, 0x7C, 0x0B, 0x59, 0x15, 0xF6, 0x01, 0x50, 0x55, 0xBF, 0x81, 0x75, 0xC3, 0x10, 0x31, 0xB5,
        0xD3, 0x69, 0x84, 0x89, 0xBA, 0xD6, 0x89, 0xBD, 0x70, 0x19, 0x8E, 0x58, 0xA8, 0x3D, 0x9B, 0x5D, 0xF0, 0x49,
        0xE8, 0xAD, 0x9D, 0x7A, 0x0D, 0x7E, 0x24, 0xDA, 0xFC, 0x0D, 0x14, 0xC5, 0x23, 0x91, 0x11, 0xF5, 0xC0, 0x4B, 0xCD,
        0x44, 0x1C, 0xC5, 0x21, 0xDF, 0x61, 0x54, 0xED, 0xA2, 0x81, 0xB7, 0xE5, 0x74, 0x94, 0xB0, 0x47, 0xEE, 0xF1,
        0xA5, 0xBB, 0x21, 0xC8
};



static int guncon3_decode(unsigned char *data,unsigned char *data_decoded) {
    int x, y, key_index;
    int bkey, keyr, byte;
    int a_sum,b_sum;
    int key_offset;

    b_sum = data[13] ^ data[12];
        b_sum = b_sum + data[11] + data[10] - data[9] - data[8];
        b_sum = b_sum ^ data[7];
        b_sum = b_sum & 0xFF;

        a_sum = data[6] ^ b_sum;
        a_sum = a_sum - data[5] - data[4];
        a_sum = a_sum ^ data[3];
        a_sum = a_sum + data[2] + data[1] - data[0];
        a_sum = a_sum & 0xFF;

    if (a_sum != key[7]) {
            if (debug)
                printk(KERN_ERR "checksum mismatch: %02x %02x\n", a_sum, key[7]);
            return -1;
        }
    key_offset = key[1] ^ key[2];
        key_offset = key_offset - key[3] - key[4];
        key_offset = key_offset ^ key[5];
        key_offset = key_offset + key[6] - key[7];
        key_offset = key_offset ^ data[14];
        key_offset = key_offset + 0x26;
        key_offset = key_offset & 0xFF;

    key_index = 4;

    //byte E is part of the key offset
    // byte D is ignored, possibly a padding byte - make the checksum workout
    for (x = 12; x >= 0; x--) {
            byte = data[x];
            for (y = 4; y > 1; y--) { // loop 3 times
                key_offset--;

                bkey = KEY_TABLE[key_offset + 0x41];
                keyr = key[key_index];
                if (--key_index == 0)
                    key_index = 7;

                if ((bkey & 3) == 0)
                    byte =(byte - bkey) - keyr;
                else if ((bkey & 3) == 1)
                    byte = ((byte + bkey) + keyr);
                else
                    byte = ((byte ^ bkey) ^ keyr);
            }
            data_decoded[x] = byte;
    }
      /*      if (debug)
        {    printk(KERN_ERR "%x %x %x %x %x %x %x %x %x %x %x %x %x",
                           data_decoded[0],data_decoded[1],data_decoded[2],data_decoded[3],data_decoded[3],
                           data_decoded[5],data_decoded[6],data_decoded[7],data_decoded[8],data_decoded[9],
                           data_decoded[10],data_decoded[11],data_decoded[12]);
        }
    */
    return 0;
}

static void guncon3_usb_irq(struct urb *urb) {
    struct guncon3 *guncon3 = urb->context;
    unsigned char *data = urb->transfer_buffer;
    unsigned char data_decoded[15];
    int error;
    unsigned long buttons;
    int16_t x, y;
    unsigned char hat0_x = 0;
    unsigned char hat0_y = 0;
    unsigned char hat1_x = 0;
    unsigned char hat1_y = 0;
    unsigned short _X_MIN, _X_MAX;
    int status;
    switch (urb->status) {
        case 0:
            /* success */
            break;
        case -ETIME:
            /* this urb is timing out */
            dev_dbg(&guncon3->intf->dev,
                    "%s - urb timed out - was the device unplugged?\n",
                    __func__);
            return;
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
        case -EPIPE:
            /* this urb is terminated, clean up */
            dev_dbg(&guncon3->intf->dev, "%s - urb shutting down with status: %d\n",
                    __func__, urb->status);
            return;
        default:
            dev_dbg(&guncon3->intf->dev, "%s - nonzero urb status received: %d\n",
                    __func__, urb->status);
            goto exit;
    }

    if (urb->actual_length == 15) {
        //decode
        status = guncon3_decode(data,data_decoded);
        if(status<0)
        {
            return;
        }
        

        /* Aiming */
        //modif guncon3
        x= (((u_int16_t)data_decoded[4]) << (8)) | data_decoded[3];
        y= (((u_int16_t)data_decoded[6]) << (8)) | data_decoded[5];
        y *= (-1);
        //stick A
        hat0_y = data_decoded[10];
        hat0_x = data_decoded[9];
       
        //stick B
        hat1_x=data_decoded[11];
        hat1_y=data_decoded[12];
       
        input_report_abs(guncon3->input_device, ABS_X, x);
        input_report_abs(guncon3->input_device, ABS_Y, y);


        /* Buttons */
        buttons = data_decoded[2];
        buttons <<=8;
        buttons += data_decoded[1];
        buttons <<=8;
        buttons += data_decoded[0];
        
        //printk(KERN_ERR "Button:%08x",buttons&(~GUNCON3_BTN_OUT_RANGE)&(~GUNCON3_BTN_ONE_REFERENCE));
        input_report_abs(guncon3->input_device, ABS_RX, hat0_x);
        input_report_abs(guncon3->input_device, ABS_RY, hat0_y);
        input_report_abs(guncon3->input_device, ABS_HAT0X, hat1_x);
        input_report_abs(guncon3->input_device, ABS_HAT0Y, hat1_y);

        // main buttons
        input_report_key(guncon3->input_device, BTN_LEFT, buttons & GUNCON3_TRIGGER); 
        input_report_key(guncon3->input_device, BTN_MIDDLE, buttons & GUNCON3_BTN_A1);
        input_report_key(guncon3->input_device, BTN_RIGHT, buttons & GUNCON3_BTN_A2); 
        input_report_key(guncon3->input_device, BTN_WEST, buttons & GUNCON3_BTN_A3);
        input_report_key(guncon3->input_device, BTN_Z, buttons & GUNCON3_BTN_B1); 
        input_report_key(guncon3->input_device, BTN_TL, buttons & GUNCON3_BTN_B2); 
        input_report_key(guncon3->input_device, BTN_TR, buttons & GUNCON3_BTN_B3);
        input_report_key(guncon3->input_device, BTN_SELECT, buttons & GUNCON3_BTN_C1); 
        input_report_key(guncon3->input_device, BTN_START, buttons & GUNCON3_BTN_C2); 
        //input_report_key(guncon3->input_device, BTN_TR2, buttons & GUNCON3_BTN_ONE_REFERENCE); //OUT        
        input_report_key(guncon3->input_device, BTN_TL2, buttons & GUNCON3_BTN_OUT_RANGE); //OUT
        
        input_sync(guncon3->input_device);
    }
    else
    {
        if (debug)
                    printk(KERN_ERR "Size input != 15");
    }

exit:
    /* Resubmit to fetch new fresh URBs */
    error = usb_submit_urb(urb, GFP_ATOMIC);
    if (error && error != -EPERM)
        dev_err(&guncon3->intf->dev,
                "%s - usb_submit_urb failed with result: %d",
                __func__, error);
}

static int guncon3_open(struct input_dev *input) {
    unsigned char *gmode;
    int status;
    struct guncon3 *guncon3 = input_get_drvdata(input);
    struct usb_device *usb_dev = interface_to_usbdev(guncon3->intf);
    int retval;
    int actual_length;
    mutex_lock(&guncon3->pm_mutex);

    gmode = kzalloc(6, GFP_KERNEL);
    if (!gmode)
        return -ENOMEM;

    /* set the key */
    guncon3->key[0] = 0x01;
    guncon3->key[1] = 0x12;
    guncon3->key[2] = 0x6F;
    guncon3->key[3] = 0x32;
    guncon3->key[4] = 0x24;
    guncon3->key[5] = 0x60;
    guncon3->key[6] = 0x17;
    guncon3->key[7] = 0x21;
    status=usb_interrupt_msg(usb_dev, usb_sndintpipe(usb_dev, 2),
                    guncon3->key, 8, &actual_length, 100000);
    if(status<0)
    {
       printk(KERN_ERR "key no sended");
    }
    else
    {
       printk(KERN_ERR "key sended!!!");
    }
    kfree(gmode);

    retval = usb_submit_urb(guncon3->urb, GFP_KERNEL);
    if (retval) {
        dev_err(&guncon3->intf->dev,
                "%s - usb_submit_urb failed, error: %d\n",
                __func__, retval);
        retval = -EIO;
        goto out;
    }

    guncon3->is_open = true;

out:
    mutex_unlock(&guncon3->pm_mutex);
    return retval;
}

static void guncon3_close(struct input_dev *input) {
    struct guncon3 *guncon3 = input_get_drvdata(input);
    mutex_lock(&guncon3->pm_mutex);
    usb_kill_urb(guncon3->urb);
    guncon3->is_open = false;
    mutex_unlock(&guncon3->pm_mutex);
}

static void guncon3_free_urb(void *context) {
    struct guncon3 *guncon3 = context;

    usb_free_urb(guncon3->urb);
}

static int guncon3_probe(struct usb_interface *intf,
                         const struct usb_device_id *id) {
    struct usb_device *udev = interface_to_usbdev(intf);
    struct guncon3 *guncon3;
    struct usb_endpoint_descriptor *epirq;
    size_t xfer_size;
    void *xfer_buf;
    int error;

    /*
   * Locate the endpoint information. This device only has an
   * interrupt endpoint.
   */
    error = usb_find_common_endpoints(intf->cur_altsetting,
                                      NULL, NULL, &epirq, NULL);
    if (error) {
        dev_err(&intf->dev, "Could not find endpoint\n");
        return error;
    }

    /* Allocate memory for the guncon3 struct using devm */
    guncon3 = devm_kzalloc(&intf->dev, sizeof(*guncon3), GFP_KERNEL);
    if (!guncon3)
        return -ENOMEM;

    mutex_init(&guncon3->pm_mutex);
    guncon3->intf = intf;

    usb_set_intfdata(guncon3->intf, guncon3);

    xfer_size = usb_endpoint_maxp(epirq);
    xfer_buf = devm_kmalloc(&intf->dev, xfer_size, GFP_KERNEL);
    if (!xfer_buf)
        return -ENOMEM;

    guncon3->urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!guncon3->urb)
        return -ENOMEM;

    error = devm_add_action_or_reset(&intf->dev, guncon3_free_urb, guncon3);
    if (error)
        return error;

    /* set to URB for the interrupt interface  */
    usb_fill_int_urb(guncon3->urb, udev,
                     usb_rcvintpipe(udev, epirq->bEndpointAddress),
                     xfer_buf, xfer_size, guncon3_usb_irq, guncon3, 1);

    /* get path tree for the usb device */
    usb_make_path(udev, guncon3->phys, sizeof(guncon3->phys));
    strlcat(guncon3->phys, "/input0", sizeof(guncon3->phys));

    /* Button related */
    guncon3->input_device = devm_input_allocate_device(&intf->dev);
    if (!guncon3->input_device) {
        dev_err(&intf->dev, "couldn't allocate input_device input device\n");
        return -ENOMEM;
    }

    guncon3->input_device->name = "Namco GunCon 3";
    guncon3->input_device->phys = guncon3->phys;

    guncon3->input_device->open = guncon3_open;
    guncon3->input_device->close = guncon3_close;



    usb_to_input_id(udev, &guncon3->input_device->id);

 
  
    // Aiming
    input_set_capability(guncon3->input_device, EV_ABS, ABS_X); //
    input_set_capability(guncon3->input_device, EV_ABS, ABS_Y);
    input_set_abs_params(guncon3->input_device, ABS_X, X_MIN, X_MAX, 0, 0);
    input_set_abs_params(guncon3->input_device, ABS_Y, Y_MIN, Y_MAX, 0, 0);
        
   
     // A-Stick
    input_set_capability(guncon3->input_device, EV_ABS, ABS_RX); 
    input_set_capability(guncon3->input_device, EV_ABS, ABS_RY); 
    input_set_abs_params(guncon3->input_device, ABS_RX, 0, 255, 0, 0);
    input_set_abs_params(guncon3->input_device, ABS_RY, 0, 255, 0, 0);

    // B-Stick
    input_set_capability(guncon3->input_device, EV_ABS, ABS_HAT0X); 
    input_set_capability(guncon3->input_device, EV_ABS, ABS_HAT0Y); 
    input_set_abs_params(guncon3->input_device, ABS_HAT0X, 0, 255, 0, 0);
    input_set_abs_params(guncon3->input_device, ABS_HAT0Y, 0, 255, 0, 0);
    
    // Buttons
    input_set_capability(guncon3->input_device, EV_KEY, BTN_LEFT);//TRIGGER
    input_set_capability(guncon3->input_device, EV_KEY, BTN_MIDDLE); //A1
    input_set_capability(guncon3->input_device, EV_KEY, BTN_RIGHT); //A2
    input_set_capability(guncon3->input_device, EV_KEY, BTN_WEST); //A3
    input_set_capability(guncon3->input_device, EV_KEY, BTN_Z); //B1
    input_set_capability(guncon3->input_device, EV_KEY, BTN_TL);//B2 
    input_set_capability(guncon3->input_device, EV_KEY, BTN_TR); //B3
    input_set_capability(guncon3->input_device, EV_KEY, BTN_SELECT); //C2 
    input_set_capability(guncon3->input_device, EV_KEY, BTN_START);  //C1
    input_set_capability(guncon3->input_device, EV_KEY, BTN_TL2); //ONE LED REFERENCE
    //input_set_capability(guncon3->input_device, EV_KEY, BTN_TR2);  //OUT OF RANGE




    input_set_drvdata(guncon3->input_device, guncon3);

    //guncon3_send_key
    error = input_register_device(guncon3->input_device);
    if (error)
        return error;

    return 0;
}

static void guncon3_disconnect(struct usb_interface *intf) {
    /* All driver resources are devm-managed. */
}

static int guncon3_suspend(struct usb_interface *intf, pm_message_t message) {
    struct guncon3 *guncon3 = usb_get_intfdata(intf);

    mutex_lock(&guncon3->pm_mutex);
    if (guncon3->is_open) {
        usb_kill_urb(guncon3->urb);
    }
    mutex_unlock(&guncon3->pm_mutex);

    return 0;
}

static int guncon3_resume(struct usb_interface *intf) {
    struct guncon3 *guncon3 = usb_get_intfdata(intf);
    int retval = 0;

    mutex_lock(&guncon3->pm_mutex);
    if (guncon3->is_open && usb_submit_urb(guncon3->urb, GFP_KERNEL) < 0) {
        retval = -EIO;
    }

    mutex_unlock(&guncon3->pm_mutex);
    return retval;
}

static int guncon3_pre_reset(struct usb_interface *intf) {
    struct guncon3 *guncon3 = usb_get_intfdata(intf);

    mutex_lock(&guncon3->pm_mutex);
    usb_kill_urb(guncon3->urb);
    return 0;
}

static int guncon3_post_reset(struct usb_interface *intf) {
    struct guncon3 *guncon3 = usb_get_intfdata(intf);
    int retval = 0;

    if (guncon3->is_open && usb_submit_urb(guncon3->urb, GFP_KERNEL) < 0) {
        retval = -EIO;
    }

    mutex_unlock(&guncon3->pm_mutex);

    return retval;
}

static int guncon3_reset_resume(struct usb_interface *intf) {
    return guncon3_resume(intf);
}

static const struct usb_device_id guncon3_table[] = {
        {USB_DEVICE(NAMCO_VENDOR_ID, GUNCON3_PRODUCT_ID)},
        {}};

MODULE_DEVICE_TABLE(usb, guncon3_table);

static struct usb_driver guncon3_driver = {
        .name = "guncon3",
        .probe = guncon3_probe,
        .disconnect = guncon3_disconnect,
        .id_table = guncon3_table,
        .suspend = guncon3_suspend,
        .resume = guncon3_resume,
        .pre_reset = guncon3_pre_reset,
        .post_reset = guncon3_post_reset,
        .reset_resume = guncon3_reset_resume,
};

module_usb_driver(guncon3_driver);

MODULE_AUTHOR("beardypig <beardypig@protonmail.com>");
MODULE_DESCRIPTION("Namco GunCon 2");
MODULE_LICENSE("GPL v2");
