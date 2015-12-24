/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Daniel Campora, Emmanuel Blot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "py/mpstate.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/stream.h"
#include "bufhelper.h"
#include "inc/hw_types.h"
#include "inc/hw_mcspi.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "rom_map.h"
#include "interrupt.h"
#include "pin.h"
#include "prcm.h"
#include "spi.h"
#include "utils.h"
#include "pybspislave.h"
#include "mpexception.h"
#include "mpirq.h"
#include "pybsleep.h"
#include "pybioctl.h"
#include "pybpin.h"
#include "pins.h"

/// \moduleref pyb
/// \class SPI - a master-driven serial protocol

//------------------------------------------------------------
// Type definitions
//------------------------------------------------------------

typedef struct _pyb_spislave_obj_t {
    mp_obj_base_t base;
    uint config;
    byte submode;
    byte *read_buf;                 // read buffer pointer
    volatile uint8_t read_buf_head; // indexes first empty slot
    uint8_t read_buf_tail;          // indexes first full slot (not full if equals head)
    byte irq_trigger;
    bool irq_enabled;
    byte irq_flags;
} pyb_spislave_obj_t;

//------------------------------------------------------------
// Constants
//------------------------------------------------------------

#define SPI_FIFO_SIZE                          64 // may be shared on TX+RX
#define PYBSPI_RX_TIMEOUT_US                10000 // 10 ms
#define PYBSPI_RX_BUFFER_LEN                  128

#define PYBSPI_TRIGGER_RX_ANY                0x01
#define PYBSPI_TRIGGER_TX_ANY                0x02

//------------------------------------------------------------
// Private data
//------------------------------------------------------------
STATIC pyb_spislave_obj_t pyb_spislave_obj;

STATIC const mp_irq_methods_t pyb_spislave_irq_methods;

STATIC const mp_obj_t pyb_spislave_def_pin[4] = 
    {&pin_GP14, &pin_GP16, &pin_GP30, &pin_GP17};

//------------------------------------------------------------
// Private API
//------------------------------------------------------------

#if 0
STATIC void trace(const char * msg)
{
    while ( *msg ) {
        MAP_UARTCharPut(UARTA0_BASE, *msg++);
    }
}
#else
#define trace(_msg_)
#endif 

// only master mode is available for the moment
STATIC void pyb_spihw_init (pyb_spislave_obj_t *self) {
    // enable the peripheral clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);

    // configure the interface (only slave mode supported)
    MAP_SPIConfigSetExpClk (GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                            0, SPI_MODE_SLAVE, self->submode, self->config);

    // re-allocate the read buffer after resetting the SPI 
    // (which automatically disables any irqs)
    self->read_buf_head = 0;
    self->read_buf_tail = 0;
    self->read_buf = MP_OBJ_NULL; // useless
    self->read_buf = m_new(byte, PYBSPI_RX_BUFFER_LEN);

    // enable the interface
    MAP_SPIEnable(GSPI_BASE);

    // enable the RX FIFO
    MAP_SPIFIFOEnable(GSPI_BASE, SPI_RX_FIFO);

    // Configure the FIFO interrupt levels
    MAP_SPIFIFOLevelSet(GSPI_BASE, 0, 0);
}

STATIC void pyb_spislave_irq_enable (mp_obj_t self_in) {
    trace("irq_enable\r\n");
    pyb_spislave_obj_t *self = self_in;
    // check for any of the rx interrupt types
    if (self->irq_trigger & (SPI_INT_RX_OVRFLOW | SPI_INT_RX_FULL)) {
        MAP_SPIIntClear(GSPI_BASE, SPI_INT_RX_OVRFLOW | SPI_INT_RX_FULL);
        MAP_SPIIntEnable(GSPI_BASE, SPI_INT_RX_OVRFLOW | SPI_INT_RX_FULL);
    }
    self->irq_enabled = true;
}

STATIC void pyb_spislave_irq_disable (mp_obj_t self_in) {
    trace("irq_disable\r\n");
    pyb_spislave_obj_t *self = self_in;
    self->irq_enabled = false;
}

STATIC int pyb_spislave_irq_flags (mp_obj_t self_in) {
    pyb_spislave_obj_t *self = self_in;
    return self->irq_flags;
}

STATIC void pyb_spislave_irq_handler(void) {
    trace("irq_h\r\n");
    pyb_spislave_obj_t *self = &pyb_spislave_obj;
    uint32_t status;

    status = MAP_SPIIntStatus(GSPI_BASE, true);
    // receive interrupt
    if (status & (SPI_INT_RX_OVRFLOW | SPI_INT_RX_FULL)) {
        // set the flags
        self->irq_flags = PYBSPI_TRIGGER_RX_ANY;
        MAP_SPIIntClear(GSPI_BASE, SPI_INT_RX_OVRFLOW | SPI_INT_RX_FULL);
        while (SPIDataIsAvailable(GSPI_BASE)) {
            uint32_t data;
            SPIDataGetNonBlocking(GSPI_BASE, &data);
            // there's always a read buffer available
            uint16_t next_head = (self->read_buf_head + 1) % PYBSPI_RX_BUFFER_LEN;
            if (next_head != self->read_buf_tail) {
                // only store data if room in buf
                self->read_buf[self->read_buf_head] = (byte)data;
                self->read_buf_head = next_head;
            }
        }
    }

    // check the flags to see if the user handler should be called
    if ((self->irq_trigger & self->irq_flags) && self->irq_enabled) {
        // call the user defined handler
        mp_irq_handler(mp_irq_find(self));
    }

    // clear the flags
    self->irq_flags = 0;
}

STATIC mp_obj_t pyb_spislave_irq_new (pyb_spislave_obj_t *self, byte trigger, mp_int_t priority, mp_obj_t handler) {
    trace("irq_new\r\n");

    // disable the SPI interrupts before updating anything
    pyb_spislave_irq_disable (self);

    MAP_IntPrioritySet(INT_GSPI, priority);
    MAP_SPIIntRegister(GSPI_BASE, pyb_spislave_irq_handler);

    // create the callback
    mp_obj_t _irq = mp_irq_new ((mp_obj_t)self, handler, &pyb_spislave_irq_methods);

    // enable the interrupts now
    self->irq_trigger = trigger;
    pyb_spislave_irq_enable (self);
    return _irq;
}

STATIC void pyb_spihw_drain (pyb_spislave_obj_t *self, uint32_t *count) {
    uint32_t rxdata;
    uint32_t c = 0;
    while ( MAP_SPIDataGetNonBlocking(GSPI_BASE, &rxdata) ) {
        (void)rxdata;
        c += 1;
    }
    *count = c;
}

STATIC uint32_t pyb_spislave_rx_any(pyb_spislave_obj_t *self) {
    trace("rx_any:");
    if (self->read_buf_tail != self->read_buf_head) {
        trace("buf\r\n");
        // buffering  via irq
        return (self->read_buf_head > self->read_buf_tail) ? self->read_buf_head - self->read_buf_tail :
                PYBSPI_RX_BUFFER_LEN - self->read_buf_tail + self->read_buf_head;
    }
    trace("hw\r\n");
    int any = SPIDataIsAvailable(GSPI_BASE);
    trace("ok\r\n");
    return (uint32_t)any;
}

STATIC int pyb_spislave_rx_char(pyb_spislave_obj_t *self) {
    trace("rx_char\r\n");
    if (self->read_buf_tail != self->read_buf_head) {
        // buffering via irq
        int data = self->read_buf[self->read_buf_tail];
        self->read_buf_tail = (self->read_buf_tail + 1) % PYBSPI_RX_BUFFER_LEN;
        return data;
    } else {
        // no buffering, not called if no data is available
        uint32_t data;
        MAP_SPIDataGetNonBlocking(GSPI_BASE, &data);
        return (int)data;
    }
}

// Waits at most timeout microseconds for at least 1 byte to become ready for
// reading (from buf or for direct reading).
// Returns true if something available, false if not.
STATIC bool pyb_spislave_rx_wait (pyb_spislave_obj_t *self) {
    trace("rx_wait\r\n");
    int timeout = PYBSPI_RX_TIMEOUT_US;
    for ( ; ; ) {
        if (pyb_spislave_rx_any(self)) {
            trace("w true\r\n");
            return true; // we have at least 1 char ready for reading
        }
        if (timeout > 0) {
            UtilsDelay(UTILS_DELAY_US_TO_COUNT(1));
            timeout--;
        }
        else {
            trace("w false\r\n");
            return false;
        }
    }
}

STATIC void pyb_spislave_check_init(pyb_spislave_obj_t *self) {
    // not initialized
    if (!self->config) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_request_not_possible));
    }
}

STATIC void pyb_spislave_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_spislave_obj_t *self = self_in;
    mp_printf(print, "SPI(0, SPI.SLAVE, polarity=%u, phase=%u, firstbit=SPI.MSB)",
              (self->submode >> 1) & 0x1, self->submode & 0x1);
}

STATIC mp_obj_t pyb_spislave_init_helper(pyb_spislave_obj_t *self, const mp_arg_val_t *args) {
    uint polarity = args[0].u_int;
    uint phase = args[1].u_int;
    if (polarity > 1 || phase > 1) {
        goto invalid_args;
    }

    // build the configuration
    self->config = SPI_WL_8 | SPI_CS_ACTIVELOW | SPI_4PIN_MODE | SPI_TURBO_OFF;
    self->submode = (polarity << 1) | phase;

    // assign the pins
    mp_obj_t pins_o = args[2].u_obj;
    if (pins_o != mp_const_none) {
        mp_obj_t *pins;
        if (pins_o == MP_OBJ_NULL) {
            // use the default pins
            pins = (mp_obj_t *)pyb_spislave_def_pin;
        } else {
            mp_obj_get_array_fixed_n(pins_o, 4, &pins);
        }
        pin_assign_pins_af (pins, 4, PIN_TYPE_STD_PU, PIN_FN_SPI, 0);
    }

    // init the bus
    pyb_spihw_init(self);

    // register it with the sleep module
    pyb_sleep_add((const mp_obj_t)self, (WakeUpCB_t)pyb_spihw_init);

    // enable the callback
    pyb_spislave_irq_new (self, PYBSPI_TRIGGER_RX_ANY, INT_PRIORITY_LVL_3, mp_const_none);
    // disable the irq (from the user point of view)
    pyb_spislave_irq_disable(self);

    return mp_const_none;

invalid_args:
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}

static const mp_arg_t pyb_spislave_init_args[] = {
    { MP_QSTR_id,                             MP_ARG_INT,  {.u_int = 0} },
    { MP_QSTR_polarity,     MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
    { MP_QSTR_phase,        MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
    { MP_QSTR_pins,         MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL} },
};

STATIC mp_obj_t pyb_spislave_make_new(mp_obj_t type_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args) {
    trace("make_new\r\n");

    // parse args
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t args[MP_ARRAY_SIZE(pyb_spislave_init_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(args), pyb_spislave_init_args, args);

    // check the peripheral id
    if (args[0].u_int != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_resource_not_avaliable));
    }

    // setup the object
    pyb_spislave_obj_t *self = &pyb_spislave_obj;
    self->base.type = &pyb_spislave_type;

    // start the peripheral
    pyb_spislave_init_helper(self, &args[1]);

    return self;
}

STATIC mp_obj_t pyb_spislave_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(pyb_spislave_init_args) - 1];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), &pyb_spislave_init_args[1], args);
    return pyb_spislave_init_helper(pos_args[0], args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_spislave_init_obj, 1, pyb_spislave_init);

STATIC mp_obj_t pyb_spislave_deinit(mp_obj_t self_in) {
    // disable the peripheral
    MAP_SPIDisable(GSPI_BASE);
    MAP_PRCMPeripheralClkDisable(PRCM_GSPI, PRCM_RUN_MODE_CLK | PRCM_SLP_MODE_CLK);
    // unregister it with the sleep module
    pyb_sleep_remove((const mp_obj_t)self_in);
    pyb_spislave_obj_t *self = (pyb_spislave_obj_t *)self_in;
    self->config = 0;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_spislave_deinit_obj, pyb_spislave_deinit);

STATIC mp_obj_t pyb_spislave_any(mp_obj_t self_in) {
    pyb_spislave_obj_t *self = self_in;
    pyb_spislave_check_init(self);
    trace("any...");
    int any = pyb_spislave_rx_any(self);
    trace("...any\r\n");
    return mp_obj_new_int(any);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_spislave_any_obj, pyb_spislave_any);

STATIC mp_obj_t pyb_spislave_drain(mp_obj_t self_in) {
    // parse args
    pyb_spislave_obj_t *self = (pyb_spislave_obj_t *)self_in;
    uint32_t count;
    pyb_spihw_drain(self, &count);

    // return the number of bytes received
    return mp_obj_new_int(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_spislave_drain_obj, pyb_spislave_drain);

STATIC mp_uint_t pyb_spislave_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    pyb_spislave_obj_t *self = self_in;
    byte *buf = buf_in;
    pyb_spislave_check_init(self);

    trace("read\r\n");

    // make sure we want at least 1 char
    if (size == 0) {
        return 0;
    }

    // wait for first char to become available
    if (!pyb_spislave_rx_wait(self)) {
        // return EAGAIN error to indicate non-blocking (then read() method returns None)
        *errcode = EAGAIN;
        return MP_STREAM_ERROR;
    }

    // read the data
    byte *orig_buf = buf;
    for ( ; ; ) {
        *buf++ = pyb_spislave_rx_char(self);
        if (--size == 0 || !pyb_spislave_rx_wait(self)) {
            // return number of bytes read
            return buf - orig_buf;
        }
    }
}

STATIC mp_uint_t pyb_spislave_ioctl(mp_obj_t self_in, mp_uint_t request, mp_uint_t arg, int *errcode) {
    pyb_spislave_obj_t *self = self_in;
    mp_uint_t ret;
    pyb_spislave_check_init(self);

    trace("ioctl\r\n");

    if (request == MP_IOCTL_POLL) {
        mp_uint_t flags = arg;
        ret = 0;
        if ((flags & MP_IOCTL_POLL_RD) && pyb_spislave_rx_any(self)) {
            ret |= MP_IOCTL_POLL_RD;
        }
    } else {
        *errcode = EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

/// \method irq(trigger, priority, handler, wake)
STATIC mp_obj_t pyb_spislave_irq(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    trace("irq\r\n");

    mp_arg_val_t args[mp_irq_INIT_NUM_ARGS];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, mp_irq_INIT_NUM_ARGS, mp_irq_init_args, args);

    // check if any parameters were passed
    pyb_spislave_obj_t *self = pos_args[0];
    pyb_spislave_check_init(self);

    // convert the priority to the correct value
    uint priority = mp_irq_translate_priority (args[1].u_int);

    // check the power mode
    uint8_t pwrmode = (args[3].u_obj == mp_const_none) ? PYB_PWR_MODE_ACTIVE : mp_obj_get_int(args[3].u_obj);
    if (PYB_PWR_MODE_ACTIVE != pwrmode) {
        goto invalid_args;
    }

    // check the trigger
    uint trigger = mp_obj_get_int(args[0].u_obj);
    if (!trigger || trigger > (PYBSPI_TRIGGER_RX_ANY)) {
        goto invalid_args;
    }

    // register a new callback
    return pyb_spislave_irq_new (self, trigger, priority, args[2].u_obj);

invalid_args:
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_spislave_irq_obj, 1, pyb_spislave_irq);

STATIC const mp_map_elem_t pyb_spislave_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_init),         (mp_obj_t)&pyb_spislave_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_deinit),       (mp_obj_t)&pyb_spislave_deinit_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_irq),          (mp_obj_t)&pyb_spislave_irq_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_drain),        (mp_obj_t)&pyb_spislave_drain_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_any),          (mp_obj_t)&pyb_spislave_any_obj },

    /// \method read([nbytes])
    { MP_OBJ_NEW_QSTR(MP_QSTR_read),         (mp_obj_t)&mp_stream_read_obj },
    /// \method readall()
    { MP_OBJ_NEW_QSTR(MP_QSTR_readall),      (mp_obj_t)&mp_stream_readall_obj },
    /// \method readline()
    { MP_OBJ_NEW_QSTR(MP_QSTR_readline),     (mp_obj_t)&mp_stream_unbuffered_readline_obj},
    /// \method readinto(buf[, nbytes])
    { MP_OBJ_NEW_QSTR(MP_QSTR_readinto),     (mp_obj_t)&mp_stream_readinto_obj },

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_RX_ANY),        MP_OBJ_NEW_SMALL_INT(PYBSPI_TRIGGER_RX_ANY) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_spislave_locals_dict, pyb_spislave_locals_dict_table);

STATIC const mp_stream_p_t pyb_spislave_stream_p = {
    .read = pyb_spislave_read,
    // .write = pyb_spislave_write,
    .ioctl = pyb_spislave_ioctl,
    .is_text = false,
};

STATIC const mp_irq_methods_t pyb_spislave_irq_methods = {
    .init = pyb_spislave_irq,
    .enable = pyb_spislave_irq_enable,
    .disable = pyb_spislave_irq_disable,
    .flags = pyb_spislave_irq_flags
};

const mp_obj_type_t pyb_spislave_type = {
    { &mp_type_type },
    .name = MP_QSTR_SPISlave,
    .print = pyb_spislave_print,
    .make_new = pyb_spislave_make_new,
    .getiter = mp_identity,
    .iternext = mp_stream_unbuffered_iter,
    .stream_p = &pyb_spislave_stream_p,
    .locals_dict = (mp_obj_t)&pyb_spislave_locals_dict,
};

