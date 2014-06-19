/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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

#include <stdio.h>
#include <assert.h>

#include "mpconfig.h"
#include "nlr.h"
#include "misc.h"
#include "qstr.h"
#include "obj.h"
#include "objstr.h"
#include "runtime0.h"
#include "runtime.h"
#include "builtin.h"

#if MICROPY_PY_BUILTINS_FLOAT
#include <math.h>
#endif

// args[0] is function from class body
// args[1] is class name
// args[2:] are base objects
STATIC mp_obj_t mp_builtin___build_class__(uint n_args, const mp_obj_t *args) {
    assert(2 <= n_args);

    // set the new classes __locals__ object
    mp_obj_dict_t *old_locals = mp_locals_get();
    mp_obj_t class_locals = mp_obj_new_dict(0);
    mp_locals_set(class_locals);

    // call the class code
    mp_obj_t cell = mp_call_function_0(args[0]);

    // restore old __locals__ object
    mp_locals_set(old_locals);

    // get the class type (meta object) from the base objects
    mp_obj_t meta;
    if (n_args == 2) {
        // no explicit bases, so use 'type'
        meta = (mp_obj_t)&mp_type_type;
    } else {
        // use type of first base object
        meta = mp_obj_get_type(args[2]);
    }

    // TODO do proper metaclass resolution for multiple base objects

    // create the new class using a call to the meta object
    mp_obj_t meta_args[3];
    meta_args[0] = args[1]; // class name
    meta_args[1] = mp_obj_new_tuple(n_args - 2, args + 2); // tuple of bases
    meta_args[2] = class_locals; // dict of members
    mp_obj_t new_class = mp_call_function_n_kw(meta, 3, 0, meta_args);

    // store into cell if neede
    if (cell != mp_const_none) {
        mp_obj_cell_set(cell, new_class);
    }

    return new_class;
}

MP_DEFINE_CONST_FUN_OBJ_VAR(mp_builtin___build_class___obj, 2, mp_builtin___build_class__);

STATIC mp_obj_t mp_builtin___repl_print__(mp_obj_t o) {
    if (o != mp_const_none) {
        mp_obj_print(o, PRINT_REPR);
        printf("\n");
    }
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin___repl_print___obj, mp_builtin___repl_print__);

mp_obj_t mp_builtin_abs(mp_obj_t o_in) {
    if (MP_OBJ_IS_SMALL_INT(o_in)) {
        mp_small_int_t val = MP_OBJ_SMALL_INT_VALUE(o_in);
        if (val < 0) {
            val = -val;
        }
        return MP_OBJ_NEW_SMALL_INT(val);
#if MICROPY_PY_BUILTINS_FLOAT
    } else if (MP_OBJ_IS_TYPE(o_in, &mp_type_float)) {
        mp_float_t value = mp_obj_float_get(o_in);
        // TODO check for NaN etc
        if (value < 0) {
            return mp_obj_new_float(-value);
        } else {
            return o_in;
        }
    } else if (MP_OBJ_IS_TYPE(o_in, &mp_type_complex)) {
        mp_float_t real, imag;
        mp_obj_complex_get(o_in, &real, &imag);
        return mp_obj_new_float(MICROPY_FLOAT_C_FUN(sqrt)(real*real + imag*imag));
#endif
    } else {
        assert(0);
        return mp_const_none;
    }
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_abs_obj, mp_builtin_abs);

STATIC mp_obj_t mp_builtin_all(mp_obj_t o_in) {
    mp_obj_t iterable = mp_getiter(o_in);
    mp_obj_t item;
    while ((item = mp_iternext(iterable)) != MP_OBJ_STOP_ITERATION) {
        if (!mp_obj_is_true(item)) {
            return mp_const_false;
        }
    }
    return mp_const_true;
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_all_obj, mp_builtin_all);

STATIC mp_obj_t mp_builtin_any(mp_obj_t o_in) {
    mp_obj_t iterable = mp_getiter(o_in);
    mp_obj_t item;
    while ((item = mp_iternext(iterable)) != MP_OBJ_STOP_ITERATION) {
        if (mp_obj_is_true(item)) {
            return mp_const_true;
        }
    }
    return mp_const_false;
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_any_obj, mp_builtin_any);

STATIC mp_obj_t mp_builtin_bin(mp_obj_t o_in) {
    mp_obj_t args[] = { MP_OBJ_NEW_QSTR(MP_QSTR__brace_open__colon__hash_b_brace_close_), o_in };
    return mp_obj_str_format(ARRAY_SIZE(args), args);
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_bin_obj, mp_builtin_bin);

STATIC mp_obj_t mp_builtin_callable(mp_obj_t o_in) {
    if (mp_obj_is_callable(o_in)) {
        return mp_const_true;
    } else {
        return mp_const_false;
    }
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_callable_obj, mp_builtin_callable);

STATIC mp_obj_t mp_builtin_chr(mp_obj_t o_in) {
    int c = mp_obj_get_int(o_in);
    char str[4];
    int len = 0;
    if (c < 0x80) {
        *str = c; len = 1;
    } else if (c < 0x800) {
        str[0] = (c >> 6) | 0xC0;
        str[1] = (c & 0x3F) | 0x80;
        len = 2;
    } else if (c < 0x10000) {
        str[0] = (c >> 12) | 0xE0;
        str[1] = ((c >> 6) & 0x3F) | 0x80;
        str[2] = (c & 0x3F) | 0x80;
        len = 3;
    } else if (c < 0x110000) {
        str[0] = (c >> 18) | 0xF0;
        str[1] = ((c >> 12) & 0x3F) | 0x80;
        str[2] = ((c >> 6) & 0x3F) | 0x80;
        str[3] = (c & 0x3F) | 0x80;
        len = 4;
    } else {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "chr() arg not in range(0x110000)"));
    }
    return mp_obj_new_str(str, len, true);
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_chr_obj, mp_builtin_chr);

STATIC mp_obj_t mp_builtin_dir(uint n_args, const mp_obj_t *args) {
    // TODO make this function more general and less of a hack

    mp_obj_dict_t *dict = NULL;
    if (n_args == 0) {
        // make a list of names in the local name space
        dict = mp_locals_get();
    } else { // n_args == 1
        // make a list of names in the given object
        if (MP_OBJ_IS_TYPE(args[0], &mp_type_module)) {
            dict = mp_obj_module_get_globals(args[0]);
        } else {
            mp_obj_type_t *type;
            if (MP_OBJ_IS_TYPE(args[0], &mp_type_type)) {
                type = args[0];
            } else {
                type = mp_obj_get_type(args[0]);
            }
            if (type->locals_dict != MP_OBJ_NULL && MP_OBJ_IS_TYPE(type->locals_dict, &mp_type_dict)) {
                dict = type->locals_dict;
            }
        }
    }

    mp_obj_t dir = mp_obj_new_list(0, NULL);
    if (dict != NULL) {
        for (uint i = 0; i < dict->map.alloc; i++) {
            if (MP_MAP_SLOT_IS_FILLED(&dict->map, i)) {
                mp_obj_list_append(dir, dict->map.table[i].key);
            }
        }
    }

    return dir;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_builtin_dir_obj, 0, 1, mp_builtin_dir);

STATIC mp_obj_t mp_builtin_divmod(mp_obj_t o1_in, mp_obj_t o2_in) {
    if (MP_OBJ_IS_SMALL_INT(o1_in) && MP_OBJ_IS_SMALL_INT(o2_in)) {
        mp_small_int_t i1 = MP_OBJ_SMALL_INT_VALUE(o1_in);
        mp_small_int_t i2 = MP_OBJ_SMALL_INT_VALUE(o2_in);
        mp_obj_t args[2];
        args[0] = MP_OBJ_NEW_SMALL_INT(i1 / i2);
        args[1] = MP_OBJ_NEW_SMALL_INT(i1 % i2);
        return mp_obj_new_tuple(2, args);
    } else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "unsupported operand type(s) for divmod(): '%s' and '%s'", mp_obj_get_type_str(o1_in), mp_obj_get_type_str(o2_in)));
    }
}

MP_DEFINE_CONST_FUN_OBJ_2(mp_builtin_divmod_obj, mp_builtin_divmod);

STATIC mp_obj_t mp_builtin_hash(mp_obj_t o_in) {
    // TODO hash will generally overflow small integer; can we safely truncate it?
    return mp_obj_new_int(mp_obj_hash(o_in));
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_hash_obj, mp_builtin_hash);

STATIC mp_obj_t mp_builtin_hex(mp_obj_t o_in) {
    return mp_binary_op(MP_BINARY_OP_MODULO, MP_OBJ_NEW_QSTR(MP_QSTR__percent__hash_x), o_in);
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_hex_obj, mp_builtin_hex);

STATIC mp_obj_t mp_builtin_iter(mp_obj_t o_in) {
    return mp_getiter(o_in);
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_iter_obj, mp_builtin_iter);

STATIC mp_obj_t mp_builtin_len(mp_obj_t o_in) {
    mp_obj_t len = mp_obj_len_maybe(o_in);
    if (len == NULL) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "object of type '%s' has no len()", mp_obj_get_type_str(o_in)));
    } else {
        return len;
    }
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_len_obj, mp_builtin_len);

STATIC mp_obj_t mp_builtin_max(uint n_args, const mp_obj_t *args) {
    if (n_args == 1) {
        // given an iterable
        mp_obj_t iterable = mp_getiter(args[0]);
        mp_obj_t max_obj = NULL;
        mp_obj_t item;
        while ((item = mp_iternext(iterable)) != MP_OBJ_STOP_ITERATION) {
            if (max_obj == NULL || (mp_binary_op(MP_BINARY_OP_LESS, max_obj, item) == mp_const_true)) {
                max_obj = item;
            }
        }
        if (max_obj == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "max() arg is an empty sequence"));
        }
        return max_obj;
    } else {
        // given many args
        mp_obj_t max_obj = args[0];
        for (int i = 1; i < n_args; i++) {
            if (mp_binary_op(MP_BINARY_OP_LESS, max_obj, args[i]) == mp_const_true) {
                max_obj = args[i];
            }
        }
        return max_obj;
    }
}

MP_DEFINE_CONST_FUN_OBJ_VAR(mp_builtin_max_obj, 1, mp_builtin_max);

STATIC mp_obj_t mp_builtin_min(uint n_args, const mp_obj_t *args) {
    if (n_args == 1) {
        // given an iterable
        mp_obj_t iterable = mp_getiter(args[0]);
        mp_obj_t min_obj = NULL;
        mp_obj_t item;
        while ((item = mp_iternext(iterable)) != MP_OBJ_STOP_ITERATION) {
            if (min_obj == NULL || (mp_binary_op(MP_BINARY_OP_LESS, item, min_obj) == mp_const_true)) {
                min_obj = item;
            }
        }
        if (min_obj == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "min() arg is an empty sequence"));
        }
        return min_obj;
    } else {
        // given many args
        mp_obj_t min_obj = args[0];
        for (int i = 1; i < n_args; i++) {
            if (mp_binary_op(MP_BINARY_OP_LESS, args[i], min_obj) == mp_const_true) {
                min_obj = args[i];
            }
        }
        return min_obj;
    }
}

MP_DEFINE_CONST_FUN_OBJ_VAR(mp_builtin_min_obj, 1, mp_builtin_min);

STATIC mp_obj_t mp_builtin_next(mp_obj_t o) {
    mp_obj_t ret = mp_iternext_allow_raise(o);
    if (ret == MP_OBJ_STOP_ITERATION) {
        nlr_raise(mp_obj_new_exception(&mp_type_StopIteration));
    } else {
        return ret;
    }
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_next_obj, mp_builtin_next);

STATIC mp_obj_t mp_builtin_oct(mp_obj_t o_in) {
    return mp_binary_op(MP_BINARY_OP_MODULO, MP_OBJ_NEW_QSTR(MP_QSTR__percent__hash_o), o_in);
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_oct_obj, mp_builtin_oct);

STATIC mp_obj_t mp_builtin_ord(mp_obj_t o_in) {
    uint len;
    const char *str = mp_obj_str_get_data(o_in, &len);
    uint charlen = unichar_charlen(str, len);
    if (charlen == 1) {
        if (MP_OBJ_IS_STR(o_in) && UTF8_IS_NONASCII(*str)) {
            machine_int_t ord = *str++ & 0x7F;
            for (machine_int_t mask = 0x40; ord & mask; mask >>= 1) {
                ord &= ~mask;
            }
            while (UTF8_IS_CONT(*str)) {
                ord = (ord << 6) | (*str++ & 0x3F);
            }
            return mp_obj_new_int(ord);
        } else {
            return mp_obj_new_int(((const byte*)str)[0]);
        }
    } else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "ord() expected a character, but string of length %d found", charlen));
    }
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_ord_obj, mp_builtin_ord);

STATIC mp_obj_t mp_builtin_pow(uint n_args, const mp_obj_t *args) {
    assert(2 <= n_args && n_args <= 3);
    switch (n_args) {
        case 2: return mp_binary_op(MP_BINARY_OP_POWER, args[0], args[1]);
        default: return mp_binary_op(MP_BINARY_OP_MODULO, mp_binary_op(MP_BINARY_OP_POWER, args[0], args[1]), args[2]); // TODO optimise...
    }
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_builtin_pow_obj, 2, 3, mp_builtin_pow);

STATIC mp_obj_t mp_builtin_print(uint n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    mp_map_elem_t *sep_elem = mp_map_lookup(kwargs, MP_OBJ_NEW_QSTR(MP_QSTR_sep), MP_MAP_LOOKUP);
    mp_map_elem_t *end_elem = mp_map_lookup(kwargs, MP_OBJ_NEW_QSTR(MP_QSTR_end), MP_MAP_LOOKUP);
    const char *sep_data = " ";
    uint sep_len = 1;
    const char *end_data = "\n";
    uint end_len = 1;
    if (sep_elem != NULL && sep_elem->value != mp_const_none) {
        sep_data = mp_obj_str_get_data(sep_elem->value, &sep_len);
    }
    if (end_elem != NULL && end_elem->value != mp_const_none) {
        end_data = mp_obj_str_get_data(end_elem->value, &end_len);
    }
    for (int i = 0; i < n_args; i++) {
        if (i > 0) {
            printf("%.*s", sep_len, sep_data);
        }
        mp_obj_print(args[i], PRINT_STR);
    }
    printf("%.*s", end_len, end_data);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_print_obj, 0, mp_builtin_print);

STATIC mp_obj_t mp_builtin_repr(mp_obj_t o_in) {
    vstr_t *vstr = vstr_new();
    mp_obj_print_helper((void (*)(void *env, const char *fmt, ...))vstr_printf, vstr, o_in, PRINT_REPR);
    mp_obj_t s = mp_obj_new_str(vstr->buf, vstr->len, false);
    vstr_free(vstr);
    return s;
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_repr_obj, mp_builtin_repr);

STATIC mp_obj_t mp_builtin_sum(uint n_args, const mp_obj_t *args) {
    assert(1 <= n_args && n_args <= 2);
    mp_obj_t value;
    switch (n_args) {
        case 1: value = mp_obj_new_int(0); break;
        default: value = args[1]; break;
    }
    mp_obj_t iterable = mp_getiter(args[0]);
    mp_obj_t item;
    while ((item = mp_iternext(iterable)) != MP_OBJ_STOP_ITERATION) {
        value = mp_binary_op(MP_BINARY_OP_ADD, value, item);
    }
    return value;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_builtin_sum_obj, 1, 2, mp_builtin_sum);

STATIC mp_obj_t mp_builtin_sorted(uint n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    assert(n_args >= 1);
    if (n_args > 1) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_TypeError,
                                          "must use keyword argument for key function"));
    }
    mp_obj_t self = mp_type_list.make_new((mp_obj_t)&mp_type_list, 1, 0, args);
    mp_obj_list_sort(1, &self, kwargs);

    return self;
}

MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_sorted_obj, 1, mp_builtin_sorted);

STATIC mp_obj_t mp_builtin_id(mp_obj_t o_in) {
    return mp_obj_new_int((machine_int_t)o_in);
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_builtin_id_obj, mp_builtin_id);

// See mp_load_attr() if making any changes
STATIC inline mp_obj_t mp_load_attr_default(mp_obj_t base, qstr attr, mp_obj_t defval) {
    mp_obj_t dest[2];
    // use load_method, raising or not raising exception
    ((defval == MP_OBJ_NULL) ? mp_load_method : mp_load_method_maybe)(base, attr, dest);
    if (dest[0] == MP_OBJ_NULL) {
        return defval;
    } else if (dest[1] == MP_OBJ_NULL) {
        // load_method returned just a normal attribute
        return dest[0];
    } else {
        // load_method returned a method, so build a bound method object
        return mp_obj_new_bound_meth(dest[0], dest[1]);
    }
}

STATIC mp_obj_t mp_builtin_getattr(uint n_args, const mp_obj_t *args) {
    mp_obj_t attr = args[1];
    if (MP_OBJ_IS_TYPE(attr, &mp_type_str)) {
        attr = mp_obj_str_intern(attr);
    } else if (!MP_OBJ_IS_QSTR(attr)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_TypeError, "string required"));
    }
    mp_obj_t defval = MP_OBJ_NULL;
    if (n_args > 2) {
        defval = args[2];
    }
    return mp_load_attr_default(args[0], MP_OBJ_QSTR_VALUE(attr), defval);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_builtin_getattr_obj, 2, 3, mp_builtin_getattr);

STATIC mp_obj_t mp_builtin_hasattr(mp_obj_t object_in, mp_obj_t attr_in) {
    assert(MP_OBJ_IS_QSTR(attr_in));

    mp_obj_t dest[2];
    // TODO: https://docs.python.org/3/library/functions.html?highlight=hasattr#hasattr
    // explicitly says "This is implemented by calling getattr(object, name) and seeing
    // whether it raises an AttributeError or not.", so we should explicitly wrap this
    // in nlr_push and handle exception.
    mp_load_method_maybe(object_in, MP_OBJ_QSTR_VALUE(attr_in), dest);

    return MP_BOOL(dest[0] != MP_OBJ_NULL);
}

MP_DEFINE_CONST_FUN_OBJ_2(mp_builtin_hasattr_obj, mp_builtin_hasattr);

// These two are defined in terms of MicroPython API functions right away
MP_DEFINE_CONST_FUN_OBJ_0(mp_builtin_globals_obj, mp_globals_get);
MP_DEFINE_CONST_FUN_OBJ_0(mp_builtin_locals_obj, mp_locals_get);
