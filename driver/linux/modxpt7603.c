//////////////////////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////////////////////

#include "../include/common.h"
#include "lvgl/src/lv_hal/lv_hal_indev.h"
#include "lvgl/src/lv_core/lv_disp.h"
#include "py/obj.h"
#include "py/runtime.h"


#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>


//////////////////////////////////////////////////////////////////////////////
// Defines
//////////////////////////////////////////////////////////////////////////////

#define XPT7603_AVG 4
#define XPT7603_READ_X  (0xc8)
#define XPT7603_READ_Y  (0xd8)
#define XPT7603_READ_Z1  (0xb8)
#define XPT7603_READ_Z2  (0xa8)


#define I2C_DEV_PATH   "/dev/i2c-0"

#define GPIO_REG_BLOCK_ADDR			0x10000000
#define GPIO_REG_DATA0_OFFSET		392
#define GPIO_IRQ_NUMBER    14


//////////////////////////////////////////////////////////////////////////////
// Module definition
//////////////////////////////////////////////////////////////////////////////

typedef struct _xpt7603_obj_t
{
    mp_obj_base_t base;

    int16_t x_min;
    int16_t y_min;
    int16_t x_max;
    int16_t y_max;
    bool x_inv;
    bool y_inv;    
    bool xy_swap;

    int16_t avg_buf_x[XPT7603_AVG];
    int16_t avg_buf_y[XPT7603_AVG];
    uint8_t avg_last;

} xpt7603_obj_t;

// Unfortunately, lvgl doesn't pass user_data to callbacks, so we use this global.
// This means we can have only one active touch driver instance, pointed by this global.
STATIC xpt7603_obj_t *g_xpt7603 = NULL;

STATIC mp_obj_t mp_activate_xpt7603(mp_obj_t self_in)
{
    xpt7603_obj_t *self = MP_OBJ_TO_PTR(self_in);
    g_xpt7603 = self;
    return mp_const_none;
}

STATIC mp_obj_t xpt7603_make_new(const mp_obj_type_t *type,
                               size_t n_args,
                               size_t n_kw,
                               const mp_obj_t *all_args)
{
    enum{
        ARG_x_min,
        ARG_y_min,
        ARG_x_max,
        ARG_y_max,
        ARG_x_inv,
        ARG_y_inv,
        ARG_xy_swap,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x_min, MP_ARG_INT, {.u_int = 200}},
        { MP_QSTR_y_min, MP_ARG_INT, {.u_int = 200}},
        { MP_QSTR_x_max, MP_ARG_INT, {.u_int = 3800}},
        { MP_QSTR_y_max, MP_ARG_INT, {.u_int = 3800}},
        { MP_QSTR_x_inv, MP_ARG_BOOL, {.u_obj = mp_const_true}},
        { MP_QSTR_y_inv, MP_ARG_BOOL, {.u_obj = mp_const_true}},
        { MP_QSTR_xy_swap, MP_ARG_BOOL, {.u_obj = mp_const_true}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    xpt7603_obj_t *self = m_new_obj(xpt7603_obj_t);
    self->base.type = type;

    self->x_min = args[ARG_x_min].u_int;
    self->y_min = args[ARG_y_min].u_int;
    self->x_max = args[ARG_x_max].u_int;
    self->y_max = args[ARG_y_max].u_int;
    self->x_inv = args[ARG_x_inv].u_bool;
    self->y_inv = args[ARG_y_inv].u_bool;
    self->xy_swap = args[ARG_xy_swap].u_bool;
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_xpt7603_init(mp_obj_t self_in);
STATIC mp_obj_t mp_xpt7603_deinit(mp_obj_t self_in);
STATIC bool xpt7603_read(lv_indev_data_t *data);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_init_xpt7603_obj, mp_xpt7603_init);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_deinit_xpt7603_obj, mp_xpt7603_deinit);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_activate_xpt7603_obj, mp_activate_xpt7603);
DEFINE_PTR_OBJ(xpt7603_read);

STATIC const mp_rom_map_elem_t xpt7603_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_init_xpt7603_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&mp_deinit_xpt7603_obj) },
    { MP_ROM_QSTR(MP_QSTR_activate), MP_ROM_PTR(&mp_activate_xpt7603_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&PTR_OBJ(xpt7603_read)) },
};

STATIC MP_DEFINE_CONST_DICT(xpt7603_locals_dict, xpt7603_locals_dict_table);

STATIC const mp_obj_type_t xpt7603_type = {
    { &mp_type_type },
    .name = MP_QSTR_xpt7603,
    //.print = xpt7603_print,
    .make_new = xpt7603_make_new,
    .locals_dict = (mp_obj_dict_t*)&xpt7603_locals_dict,
};

STATIC const mp_rom_map_elem_t xpt7603_globals_table[] = {
        { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_xpt7603) },
        { MP_ROM_QSTR(MP_QSTR_xpt7603), (mp_obj_t)&xpt7603_type},
};
         

STATIC MP_DEFINE_CONST_DICT (
    mp_module_xpt7603_globals,
    xpt7603_globals_table
);

const mp_obj_module_t mp_module_xpt7603 = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_xpt7603_globals
};

//////////////////////////////////////////////////////////////////////////////
// Module implementation
//////////////////////////////////////////////////////////////////////////////

STATIC mp_obj_t mp_xpt7603_init(mp_obj_t self_in)
{
    // printf("xpt7603_init\n");
    mp_activate_xpt7603(self_in);
    
    return mp_const_none;
}

STATIC mp_obj_t mp_xpt7603_deinit(mp_obj_t self_in)
{
    //xpt7603_obj_t *self = MP_OBJ_TO_PTR(self_in);

    return mp_const_none;
}

static void xpt7603_corr(xpt7603_obj_t *self, int16_t * x, int16_t * y);
static void xpt7603_avg(xpt7603_obj_t *self, int16_t * x, int16_t * y);
static uint16_t om_i2c_read(int addr);
static int om_read_gpio(int gpioNumber);

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no ore data to be read
 */
static bool xpt7603_read(lv_indev_data_t * data)
{
    xpt7603_obj_t *self = MP_OBJ_TO_PTR(g_xpt7603 );
    if (!self) nlr_raise(
            mp_obj_new_exception_msg(
                &mp_type_RuntimeError, "xpt7603 instance needs to be created before callback is called!"));
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    bool pressed = true;

    int16_t x = 0;
    int16_t y = 0;

    // read interrupt gpio
    uint8_t irq = (uint8_t)om_read_gpio(GPIO_IRQ_NUMBER);

    if(irq == 0) {
        // read and shift X
        x = (int16_t)(om_i2c_read(XPT7603_READ_X) >> 4);
        
        // read and shift Y
        y = (int16_t)(om_i2c_read(XPT7603_READ_Y) >> 4);

        /*Normalize Data*/
        xpt7603_corr(self, &x, &y);
        xpt7603_avg(self, &x, &y);
        last_x = x;
        last_y = y;
    
        // printf("xpt7603_read on irq: x = %d, y = %d\n", x,y);
    } else {
        x = last_x;
        y = last_y;
        self->avg_last = 0;
        pressed = false;
    }
    
    // printf("xpt7603_read: x = %d, y = %d\n", x,y);

    data->point.x = x;
    data->point.y = y;
    data->state = pressed == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;
    
    // printf("xpt7603_read data: x = %d, y = %d, pressed = %d, state = %d\n", data->point.x, data->point.y, (pressed == true ? 1 : 0), data->state);

    // return pressed;
    return false;
}

/**********************
 *   HELPER FUNCTIONS
 **********************/

static uint16_t om_i2c_read(int addr) {
    int 	status, size;
// 	char 	pathname[255];
	int     numBytes = 2;
	int     devAddr = 0x48;
	int     fd;
    uint8_t buffer[2];
    uint16_t ret = 0;

	//// define the path to open
	//status = sprintf(pathname, I2C_DEV_PATH);
    fd = open(I2C_DEV_PATH, O_RDWR);
    
    //// set the device address
    // set to 7-bit addr
	if ( ioctl(fd, I2C_TENBIT, 0) < 0 ) {
		return EXIT_FAILURE;
	}

	//// write the  address
	if ( ioctl(fd, I2C_SLAVE, devAddr) < 0 ) {
	    close(fd);
		return EXIT_FAILURE;
	}
	
	//// set addr
	// clear the buffer
	memset( buffer, 0, numBytes );
	// push the address and data values into the buffer
	buffer[0]	= (addr & 0xff);
	size 		= 1;

	// write to the i2c device
	status = write(fd, buffer, size);
	
	// read from the i2c device
	size 	= numBytes;
	status 	= read(fd, &buffer, size);
	
	ret = (buffer[0] & 0xff) << 8;
	ret += buffer[1] & 0xff;
	
	// release the device file handle
	if ( close(fd) < 0 ) {
		return EXIT_FAILURE;
	}
	
	return ret;
}
 
static int om_read_gpio(int gpioNumber) {
    int  m_mfd;
    int regAddrOffset = GPIO_REG_DATA0_OFFSET;
    unsigned long int *regAddress;
    unsigned long int 	value = 0x0;
    int gpioVal;
    
    if ((m_mfd = open("/dev/mem", O_RDWR)) < 0)
	{
		return EXIT_FAILURE;	// maybe return -1
	}
	regAddress = (unsigned long int*)mmap	(	NULL, 
												1024, 
												PROT_READ|PROT_WRITE, 
												MAP_FILE|MAP_SHARED, 
												m_mfd, 
												GPIO_REG_BLOCK_ADDR
											);
    close(m_mfd);
    
	// read the value 
	value = *(regAddress + regAddrOffset);
// 	printf("GPIO_DATA_0 = 0x%08x\n", value);
	
	gpioVal = ((value >> gpioNumber) & 0x1);
// 	printf("gpioVal = %d\n", gpioVal);
	
	return gpioVal;
}

static void xpt7603_corr(xpt7603_obj_t *self, int16_t * x, int16_t * y)
{
    if (self->xy_swap){
        int16_t swap_tmp;
        swap_tmp = *x;
        *x = *y;
        *y = swap_tmp;
    }

    if((*x) > self->x_min)(*x) -= self->x_min;
    else(*x) = 0;

    if((*y) > self->y_min)(*y) -= self->y_min;
    else(*y) = 0;

    (*x) = (uint32_t)((uint32_t)(*x) * LV_HOR_RES) /
           (self->x_max - self->x_min);

    (*y) = (uint32_t)((uint32_t)(*y) * LV_VER_RES) /
           (self->y_max - self->y_min);

    if (self->x_inv){
        (*x) =  LV_HOR_RES - (*x);
    }

    if (self->y_inv){
        (*y) =  LV_VER_RES - (*y);
    }
}


static void xpt7603_avg(xpt7603_obj_t *self, int16_t * x, int16_t * y)
{
    /*Shift out the oldest data*/
    uint8_t i;
    for(i = XPT7603_AVG - 1; i > 0 ; i--) {
        self->avg_buf_x[i] = self->avg_buf_x[i - 1];
        self->avg_buf_y[i] = self->avg_buf_y[i - 1];
    }

    /*Insert the new point*/
    self->avg_buf_x[0] = *x;
    self->avg_buf_y[0] = *y;
    if(self->avg_last < XPT7603_AVG) self->avg_last++;

    /*Sum the x and y coordinates*/
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for(i = 0; i < self->avg_last ; i++) {
        x_sum += self->avg_buf_x[i];
        y_sum += self->avg_buf_y[i];
    }

    /*Normalize the sums*/
    (*x) = (int32_t)x_sum / self->avg_last;
    (*y) = (int32_t)y_sum / self->avg_last;
}
