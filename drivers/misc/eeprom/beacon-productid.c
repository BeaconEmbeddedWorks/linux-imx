/*
 * logicpd-new-product-id
 *
 * Copyright (C) 2017 Logic Product Development, Inc.
 * Adam Ford <adam.ford@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>   // Needed by segment descriptors

#define EEPROM_PATH "/sys/devices/platform/30a40000.i2c/i2c-2/2-0050/eeprom"

unsigned char *id_data_buf;

// This is for debugging purposes only. Set to zero to deactivate.
#define SPOOF_VERSION_CODE 0

#ifdef DEBUG
#define DPRINTF(fmt, args...) printk(KERN_DBG fmt, ## args)
#else
#define DPRINTF(fmt, ...)
#endif

#undef ID_KEY_STRINGS
#define ID_KEY_ENUMS

#undef ID_KEY_START
#undef ID_KEY_ENTRY
#undef ID_KEY_END

#if defined(ID_KEY_STRINGS)
/* This is the usage to build the keys for the compiler; we define
 * an array of strings whose index is the value */
#define ID_KEY_START static char *id_keys[] = {
#define ID_KEY_ENTRY(XX) #XX,
#define ID_KEY_END };
#elif defined(ID_KEY_ENUMS)
/* This is the usage by people using the library to access the data */
#define ID_KEY_START typedef enum {
#define ID_KEY_ENTRY(XX) ID_KEY_ ## XX,
#define ID_KEY_END } id_keys_t;
#else
#error "Need either ID_KEY_INTERFACE or ID_KEY_COMPILER defined!"
#endif

/* There are some implied conventions here: */
/* - names of keys that contain other keys (dictionaries) end in "_group" */
/* - names of keys that provide a register setting end in "_reg"          */
/* - any keys that specify a unit of measure, include units in the name (ie. _mhz, _degf, _bytes) */

ID_KEY_START

/* Manufacturing unique data for each SOM */
ID_KEY_ENTRY(serialization_group)
ID_KEY_ENTRY(serial_number)
ID_KEY_ENTRY(wifi_ethaddr1)
ID_KEY_ENTRY(wifi_ethaddr2)
ID_KEY_ENTRY(wifi_ethaddr3)
ID_KEY_ENTRY(wifi_ethaddr4)
ID_KEY_ENTRY(nvs)

/* BOM Model number information */
ID_KEY_ENTRY(model_group)
ID_KEY_ENTRY(model_name)
ID_KEY_ENTRY(part_number)
ID_KEY_ENTRY(version_code)
ID_KEY_ENTRY(hardware_platform)

/* CPU specific information */
ID_KEY_ENTRY(cpu0_group)
ID_KEY_ENTRY(type)
ID_KEY_ENTRY(number)
ID_KEY_ENTRY(speed_mhz)
ID_KEY_ENTRY(temp_class)

/* CPU bus information */
ID_KEY_ENTRY(cpu0_bus_group)

/* DRAM bus information */
ID_KEY_ENTRY(dram_bus_group)
ID_KEY_ENTRY(sysconfig_reg)
ID_KEY_ENTRY(sharing_reg)
ID_KEY_ENTRY(dlla_ctrl_reg)
ID_KEY_ENTRY(cs_cfg_reg)
// ID_KEY_ENTRY(cs0_group) Used in the dram_bus_group, but key defined below after local_bus_group
// ID_KEY_ENTRY(cs1_group) Used in the dram_bus_group, but key defined below after local_bus_group
ID_KEY_ENTRY(mcfg_reg)
ID_KEY_ENTRY(mr_reg)
ID_KEY_ENTRY(rfr_ctrl_reg)
ID_KEY_ENTRY(emr2_reg)
ID_KEY_ENTRY(actim_ctrla_reg)
ID_KEY_ENTRY(actim_ctrlb_reg)
ID_KEY_ENTRY(power_reg)

/* GPMC keys */
ID_KEY_ENTRY(local_bus_group)
ID_KEY_ENTRY(cs0_group)
ID_KEY_ENTRY(cs1_group)
ID_KEY_ENTRY(cs2_group)
ID_KEY_ENTRY(cs3_group)
ID_KEY_ENTRY(cs4_group)
ID_KEY_ENTRY(cs5_group)
ID_KEY_ENTRY(cs6_group)
ID_KEY_ENTRY(config1_reg)
ID_KEY_ENTRY(config2_reg)
ID_KEY_ENTRY(config3_reg)
ID_KEY_ENTRY(config4_reg)
ID_KEY_ENTRY(config5_reg)
ID_KEY_ENTRY(config6_reg)
ID_KEY_ENTRY(config7_reg)

/* Manufacturing unique data for each SOM */
ID_KEY_ENTRY(lan_ethaddr1)
ID_KEY_ENTRY(lan_ethaddr2)
ID_KEY_ENTRY(lan_ethaddr3)
ID_KEY_ENTRY(lan_ethaddr4)

/* End of keys */
ID_KEY_END

typedef enum {
	/* Number */
	IDENUM_NEG_NUM = 0,
	IDENUM_POS_NUM,

	/* String/Hex String */
	IDENUM_STR,
	IDENUM_HEXSTR,

	/* Array */
	IDENUM_ARRAY,

	/* Dictionary */
	IDENUM_DICT,

	/* Key */
	IDENUM_KEY,

	/* Any string */
	IDENUM_ANY_STRING,

	/* Any number */
	IDENUM_ANY_NUMBER,

} idenum_t;

/* structure of builtin keys */
struct id_key {
	unsigned char *ptr;
	unsigned int size;
};

#define ID_EOK		0	/* Okay */
#define ID_ENOENT	2	/* No such key */
#define ID_ENOMEM	12	/* Out of memory */
#define ID_EACCES	13	/* Permission denied */
#define ID_ENODEV	19	/* No such device */
#define ID_EINVAL	22	/* Invalid arcument */
#define ID_EDOM		33	/* argument out of domain of func */
#define ID_ERANGE	34	/* Out of range */
#define	ID_EL2NSYNC	45	/* Level 2 not synchronized */
#define	ID_ENOMEDIUM	123	/* No medium found */

/*
 * return a byte from the ID data at offset 'offset' and set *oor to zero
 * if offset is in range of the device.  If offset is out of range then
 * set *oor to non-zero
 */

static unsigned char id_fetch_byte(int offset, int *oor)
{
	unsigned char *p = (unsigned char *)&id_data_buf[0];

	if (offset < (32<<10)) {
		*oor = ID_EOK;
		return p[offset];
	}

	*oor = -ID_ERANGE;
	return 0;
}

struct id_data {
	unsigned int root_size;
	unsigned int root_offset;
};

/* Function to do the initial startup (i.e. figure out how much data, offset of
 * key table, etc */
static int id_startup(struct id_data *data);
/*
 * Functions provided back to callers for use in accessing data
 */

/* ID data "cookie" used to access data; ultimately this will be opaque
 * to the callers as they don't need to know whats in it, just pass it around
 */
struct id_cookie {
	unsigned int start_offset;	/* start offset from beginning of data */
	unsigned int size;		/* size of data in bytes */
	unsigned int offset;		/* current read offset */
};

/* Initialize the cookie to cover the whole root dictionary */
static int id_init_cookie(struct id_data *data, struct id_cookie *cookie);

/* What is the read pointer cookie is pointing at */
static int id_whatis(struct id_cookie *cookie, idenum_t *type);

/* User interface functions */

static int id_dict_find_key(struct id_cookie *cookie, id_keys_t key);
static int id_find_dict(struct id_cookie *cookie, id_keys_t key, idenum_t type);
static int id_find_string(struct id_cookie *cookie, id_keys_t key, unsigned char *str_ptr, unsigned int *str_size);
static int id_find_number(struct id_cookie *cookie, id_keys_t key, int *num);
static int id_find_numbers(struct id_cookie *cookie, id_keys_t *key, int key_size, int *nums);

/*
 * Extract an unsigned packed number, first byte is in 'pack_bits'
 * of first byte, starting at offset 'offset' */
static unsigned int extract_unsigned_pnum(struct id_cookie *cookie, int pack_bits, int *err);
static int extract_signed_pnum(struct id_cookie *cookie, int pack_bits, int *err);



#define ID_MAX_KEY_SIZE 32

static int id_extract_size(struct id_cookie *cookie, int *err);

/* struct id_data id_data; */

struct __attribute__ ((packed)) id_header {
	unsigned char signature[4];
	unsigned char id_fmt_ver;
	unsigned char unused0;
	unsigned short data_length;
} id_header;

struct __attribute__ ((packed)) id_checksums {
	unsigned short header;
	unsigned short data;
};

/*
 * Calculate a CRC-15 of a data buffer passed in
 */

void crc_15_step(unsigned short *crc, unsigned char byte)
{
	int i;
	unsigned short crcnext;

	for (i = 0; i < 7; ++i) {
		crcnext = (byte & 1) ^ (*crc>>14);
		*crc = (*crc << 1) & 0x7fff;
		if (crcnext)
			*crc ^= 0x4599;
		byte >>= 1;
	}
}

unsigned short crc_15(void *buf, int len)
{
	unsigned char *p = buf;
	unsigned short xsum = 0;
	int i;

	for (i = 0; i < len; ++i) {
		crc_15_step(&xsum, p[i]);
	}
	return xsum;
}

static int id_startup(struct id_data *data)
{
	int i, err;
	struct id_cookie cookie;
	unsigned char byte, *p;
	char *header_tag = "LpId";
	unsigned short xsum;
	struct id_header hdr;
	struct id_checksums xsums;

	cookie.offset = 0;

	/* Data starts with the header, should be 'LpId' */
	for (i = 0; i < 4; ++i) {
		byte = id_fetch_byte(cookie.offset, &err);
		hdr.signature[i] = byte;
		cookie.offset++;
		if (err != ID_EOK) {
			printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
			goto err_ret;
		}
		if (hdr.signature[i] != header_tag[i]) {
			printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
			err = ID_ENODEV;
			goto err_ret;
		}
	}

	/* First LE 8-bit value is ID format version */
	hdr.id_fmt_ver = id_fetch_byte(cookie.offset++, &err);

	/* Second LE 8-bit value is currently not used */
		hdr.unused0 = id_fetch_byte(cookie.offset++, &err);

	/* Next LE 16-bit value is length of data */
	hdr.data_length = id_fetch_byte(cookie.offset++, &err);
	hdr.data_length |= (id_fetch_byte(cookie.offset++, &err) << 8);

	/* Next LE 16-bit value is xsum of header */
	xsums.header = id_fetch_byte(cookie.offset++, &err);
	xsums.header |= (id_fetch_byte(cookie.offset++, &err) << 8);

	/* Checksum the header */
	xsum = crc_15(&hdr, sizeof(hdr));
	p = (unsigned char *)&hdr;
	//for (i = 0; i < sizeof(hdr); ++i)
	//	crc_15_step(&xsum, p[i]);


	if (xsum != xsums.header) {
		printk(KERN_DEBUG "%s[%u] xsum: 0x%04x, xsums.header: 0x%04x\n",
			__FILE__, __LINE__, xsum, xsums.header);
		err = -ID_EL2NSYNC;
		goto err_ret;
	}

	/* Next LE 16-bit value is xsum of data */
	xsums.data = id_fetch_byte(cookie.offset++, &err);
	xsums.data |= (id_fetch_byte(cookie.offset++, &err) << 8);


	/* Checksum the data (next id_len bytes), must match xsums.data */
	xsum = 0;
	for (i = 0; i < hdr.data_length; ++i) {
		byte = id_fetch_byte(cookie.offset + i, &err);

		if (err != ID_EOK) {
			printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
			goto err_ret;
		}
		crc_15_step(&xsum, byte);
	}
	if (xsum != xsums.data) {
		printk(KERN_DEBUG "%s[%u] xsum: 0x%04x, xsums.data: 0x%04x\n",
			__FILE__, __LINE__, xsum, xsums.data);
		err = -ID_EL2NSYNC;
		goto err_ret;
	}

	/* offset is now at the first byte of the root dictionary which
	   contains its span */
	data->root_offset = cookie.offset;
	data->root_size = extract_unsigned_pnum(&cookie, 5, &err);
	if (err != ID_EOK) {
		printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
		goto err_ret;
	}

	data->root_size += cookie.offset - data->root_offset;

	return ID_EOK;

err_ret:
	return err;
}

/*
 * Reset the cookie to cover the whole root dictionary
 */
int id_init_cookie(struct id_data *data, struct id_cookie *cookie)
{
	if (!cookie)
		return -ID_EINVAL;
	cookie->start_offset = data->root_offset;
	cookie->size = data->root_size;
	cookie->offset = cookie->start_offset;
	return ID_EOK;
}

unsigned int extract_unsigned_pnum(struct id_cookie *cookie, int start_bit, int *err)
{
	unsigned int value = 0;
	unsigned int bit_offset = 0;
	unsigned char bits;
	unsigned char ch;
	int oor;

	*err = ID_EOK;
	for (;;) {
		ch = id_fetch_byte(cookie->offset++, &oor);
		if (oor != ID_EOK) {
			*err = oor;
			printk(KERN_ERR "extract runs oor");
			return 0;
		}
		if (ch & (1<<(start_bit-1))) {
			/* more to go, accumulate bits */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			bit_offset += start_bit-1;
			start_bit = 8;
		} else {
			/* last byte of number */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			break;
		}
	}
	return value;
}

int extract_signed_pnum(struct id_cookie *cookie, int start_bit, int *err)
{
	int value = 0;
	unsigned int bit_offset = 0;
	unsigned char bits;
	unsigned char ch;
	int oor;

	*err = ID_EOK;
	for (;;) {
		ch = id_fetch_byte(cookie->offset++, &oor);
		if (oor != ID_EOK) {
			*err = oor;
			printk(KERN_ERR "extract runs oor");
			return 0;
		}
		if (ch & (1<<(start_bit-1))) {
			/* more to go, accumulate bits */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			bit_offset += start_bit-1;
			start_bit = 8;
		} else {
			/* last byte of number */
			bits = ch & ((1<<(start_bit - 2)) - 1);
			value |= (bits << bit_offset);
			if (ch & (1<<(start_bit - 2)))
				value = -value;
			break;
		}
	}
	return value;
}

int id_whatis(struct id_cookie *cookie, idenum_t *type)
{
	unsigned char byte;
	int oor;

	if (!cookie)
		return -ID_EINVAL;

	byte = id_fetch_byte(cookie->offset, &oor);
	if (oor != ID_EOK)
		return -ID_ERANGE;

	byte >>= 5;
	*type = (idenum_t)byte;

	return ID_EOK;
}

int id_extract_size(struct id_cookie *cookie, int *err)
{
	idenum_t type;
	struct id_cookie s_cookie;
	int size;

	s_cookie = *cookie;

	*err = id_whatis(&s_cookie, &type);
	if (*err != ID_EOK)
		return *err;

	switch (type) {
	case IDENUM_DICT:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_ARRAY:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_STR:
	case IDENUM_HEXSTR:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_POS_NUM:
	case IDENUM_NEG_NUM:
		extract_signed_pnum(&s_cookie, 5, err);
		size = (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_KEY:
		extract_unsigned_pnum(&s_cookie, 5, err);
		size = (s_cookie.offset - cookie->offset);
		break;
	default:
		*err = -ID_EDOM;
		size = 0;
		break;
	}
	if (*err != ID_EOK)
		return *err;

	return size;
}

static int id_extract_key(struct id_cookie *cookie, id_keys_t *key)
{
	int err;
	id_keys_t keyval;

	keyval = (id_keys_t)extract_unsigned_pnum(cookie, 5, &err);
	if (err != ID_EOK)
		return err;
	*key = keyval;
	return ID_EOK;
}

/* in dictionary that cookie points to find key "key"; if found
 * update cookie to associated "key" entry and return ID_EOK;
 * else return -ID_ENOENT */
static int id_dict_find_key(struct id_cookie *cookie, id_keys_t key)
{
	int err;
	unsigned int size;
	id_keys_t d_key;
	idenum_t type;
	struct id_cookie d_cookie = *cookie;
	struct id_cookie t_cookie;

	err = id_whatis(cookie, &type);
	if (err != ID_EOK)
		return err;

	/* Header has to be a dictionary */
	if (type != IDENUM_DICT)
		return -ID_EINVAL;

	/* Extract size of dictionary */
	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	d_cookie.size = size;
	d_cookie.start_offset = d_cookie.offset;

	/* cookie->offset is now at first key */
	while (d_cookie.offset < d_cookie.start_offset + d_cookie.size) {
		/* Extract the key and move the cookie over key */
		err = id_extract_key(&d_cookie, &d_key);
		if (err != ID_EOK)
			return err;
		t_cookie = d_cookie;
		/* move forward over the value */
		size = id_extract_size(&d_cookie, &err);
		if (err != ID_EOK)
			return err;
		if (key == d_key) {
			d_cookie.size = size;
			d_cookie.start_offset = t_cookie.offset;
			d_cookie.offset = t_cookie.offset;
			*cookie = d_cookie;
			return ID_EOK;
		}
		d_cookie.offset += size;
	}
	return -ID_ENOENT;
}

/* Are these two types a match? */
static int id_match_type(idenum_t type_a, idenum_t type_b)
{
	idenum_t tmp;

	if (type_a == type_b)
		return 1;

	/* Oder the types (so the "*ANY*" types are in type_b) */
	if ((int)type_a > (int)type_b) {
		tmp = type_a;
		type_a = type_b;
		type_b = tmp;
	}
	if (type_b == IDENUM_ANY_STRING && (type_a == IDENUM_STR || type_a == IDENUM_HEXSTR))
		return 1;

	if (type_b == IDENUM_ANY_NUMBER && (type_a == IDENUM_NEG_NUM || type_a == IDENUM_POS_NUM))
		return 1;

	return 0;
}

/* Find in dictionary (that cookie points to) key "key" that is type "type" */
static int id_find_dict(struct id_cookie *cookie, id_keys_t key, idenum_t type)
{
	int err;
	struct id_cookie d_cookie = *cookie;
	idenum_t l_type;

	err = id_dict_find_key(&d_cookie, key);
	if (err != ID_EOK)
		return err;
	err = id_whatis(&d_cookie, &l_type);
	if (err != ID_EOK)
		return err;
	if (!id_match_type(l_type, type))
		return -ID_EINVAL;
	*cookie = d_cookie;
	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the key "key"; verify its a
 * string and copy its value */
static int id_find_string(struct id_cookie *cookie, id_keys_t key, unsigned char *str_ptr, unsigned int *str_size)
{
	int err, i;
	unsigned char byte;
	unsigned int size;
	struct id_cookie d_cookie = *cookie;

	err = id_find_dict(&d_cookie, key, IDENUM_ANY_STRING);

	if (err != ID_EOK)
		return err;
	/* Extract the string size */
	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	/* If handed a NULL str_ptr, only set the size and return */
	if (!str_ptr) {
		*str_size = size;
		return ID_EOK;
	}

	if (size > *str_size)
		return -ID_ERANGE;

	for (i = 0; i < size; ++i) {
		byte = id_fetch_byte(d_cookie.offset++, &err);
		if (err)
			return err;
		str_ptr[i] = byte;
	}
	*str_size = size;

	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the key "key"; verify its a
 * number (either pos/neg) and return its value through *num */
static int id_find_number(struct id_cookie *cookie, id_keys_t key, int *num)
{
	int err;
	int l_num;
	idenum_t l_type;
	struct id_cookie d_cookie = *cookie;

	err = id_find_dict(&d_cookie, key, IDENUM_ANY_NUMBER);

	if (err != ID_EOK)
		return err;
	err = id_whatis(&d_cookie, &l_type);
	if (err != ID_EOK)
		return err;
	/* Extract the number size */
#if SPOOF_VERSION_CODE
	if ((cookie->offset == 509) && (key == ID_KEY_version_code)) {
		*num = SPOOF_VERSION_CODE;
		return ID_EOK;
	}
#endif
	l_num = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	if (l_type == IDENUM_NEG_NUM)
		l_num = -l_num;

	*num = l_num;
	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the list of keys; verify they are
 * numbers (either pos/neg) and return their value through *nums */
static int id_find_numbers(struct id_cookie *cookie, id_keys_t *keys, int key_size, int *nums)
{
	int i, err;
	struct id_cookie d_cookie;

	for (i = 0; i < key_size; ++i) {
		d_cookie = *cookie;
		err = id_find_number(&d_cookie, keys[i], &nums[i]);
		if (err != ID_EOK)
			return err;
	}
	return ID_EOK;
}

/* --------------------------------------------------------- */

/*
 * Here down is the code to interface to the kernel to extract product
 * ID information from the SRAM/AT24 chip.
 */

struct id_data id_data;
static int found_id_data;
static struct id_cookie cpu0_group_cookie;
static struct id_cookie serialization_group_cookie;
static struct id_cookie model_group_cookie;

static int beacon_mfg_find_model_group_cookie(struct id_cookie *mg_cookie)
{
	int ret;
	struct id_cookie cookie;

	if (!found_id_data) {
		return -1;
	}

	if (model_group_cookie.offset) {
		*mg_cookie = model_group_cookie;
		return ID_EOK;
	}

	/* Reinitialise cookie back to the root */
	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* find /model_group from root */
	ret = id_find_dict(&cookie, ID_KEY_model_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	model_group_cookie = cookie;
	*mg_cookie = cookie;
	return ret;
}

static int beacon_mfg_find_serialization_cookie(struct id_cookie *s_cookie)
{
	int ret;
	struct id_cookie cookie;

	if (!found_id_data) {
		return -1;
	}

	if (serialization_group_cookie.offset) {
		*s_cookie = serialization_group_cookie;
		return ID_EOK;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* find /serialization_group from root */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	serialization_group_cookie = cookie;
	*s_cookie = cookie;
	return ID_EOK;
}

static int beacon_mfg_find_cpu0_group_cookie(struct id_cookie *s_cookie)
{
	int ret;
	struct id_cookie cookie;

	if (!found_id_data) {
		return -1;
	}

	if (cpu0_group_cookie.offset) {
		*s_cookie = cpu0_group_cookie;
		return ID_EOK;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* find /cpu0_group from root */
	ret = id_find_dict(&cookie, ID_KEY_cpu0_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	cpu0_group_cookie = cookie;
	*s_cookie = cookie;
	return ID_EOK;
}

int beacon_mfg_extract_new_part_number(u32 *part_number)
{
	int ret;
	struct id_cookie cookie;

	ret = beacon_mfg_find_model_group_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* Find part number */
	ret = id_find_number(&cookie, ID_KEY_part_number, part_number);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}
	return ret;
}

int beacon_mfg_extract_new_version_code(u32 *version_code)
{
	int ret;
	struct id_cookie cookie;

	ret = beacon_mfg_find_model_group_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return -EINVAL;
	}

	/* Find part number */
	ret = id_find_number(&cookie, ID_KEY_version_code, version_code);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return -EINVAL;
	}
	return ret;
}

int beacon_mfg_extract_new_speed_mhz(u32 *speed_mhz)
{
	int ret;
	struct id_cookie cookie;

	ret = beacon_mfg_find_cpu0_group_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* Find part number */
	ret = id_find_number(&cookie, ID_KEY_speed_mhz, speed_mhz);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}
	return ret;
}

struct ddr_timings {
	u32 sysconfig;
	u32 sharing;
	u32 power;
	u32 cfg;
	struct ddr_cs {
		u32 mcfg;
		u32 mr;
		u32 rfr;
		u32 emr;
		u32 actima;
		u32 actimb;
		u32 dlla;
	} cs;
} ddr_timings;

id_keys_t dram_cs_group_keys[] = {
	ID_KEY_mcfg_reg,
	ID_KEY_mr_reg,
	ID_KEY_rfr_ctrl_reg,
	ID_KEY_emr2_reg,
	ID_KEY_actim_ctrla_reg,
	ID_KEY_actim_ctrlb_reg,
	ID_KEY_dlla_ctrl_reg,
};
id_keys_t dram_bus_group_keys[] = {
	ID_KEY_sysconfig_reg,
	ID_KEY_sharing_reg,
	ID_KEY_power_reg,
	ID_KEY_cs_cfg_reg,
};

int beacon_mfg_extract_new_ddr_timings(struct ddr_timings *ddr_timings)
{
	int ret;
	struct id_cookie cookie, dram_bus_group_cookie;
	int dram_bus_group_values[ARRAY_SIZE(dram_bus_group_keys)];
	int dram_cs_group_values[ARRAY_SIZE(dram_cs_group_keys)];

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /cpu0_bus_group from root */
	ret = id_find_dict(&cookie, ID_KEY_cpu0_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /dram_bus_group from /cpu0_bus_group */
	ret = id_find_dict(&cookie, ID_KEY_dram_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	dram_bus_group_cookie = cookie;
	ret = id_find_numbers(&dram_bus_group_cookie, dram_bus_group_keys, ARRAY_SIZE(dram_bus_group_keys), dram_bus_group_values);
	if (ret != ID_EOK) {
		return ret;
	}

	ddr_timings->sysconfig = dram_bus_group_values[0];
	ddr_timings->sharing = dram_bus_group_values[1];
	ddr_timings->power = dram_bus_group_values[2];
	ddr_timings->cfg = dram_bus_group_values[3];

	ret = id_find_dict(&dram_bus_group_cookie, ID_KEY_cs0_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	ret = id_find_numbers(&dram_bus_group_cookie, dram_cs_group_keys, ARRAY_SIZE(dram_cs_group_keys), dram_cs_group_values);
	if (ret != ID_EOK) {
		return ret;
	}
	ddr_timings->cs.mcfg = dram_cs_group_values[0];
	ddr_timings->cs.mr = dram_cs_group_values[1];
	ddr_timings->cs.rfr = dram_cs_group_values[2];
	ddr_timings->cs.emr = dram_cs_group_values[3];
	ddr_timings->cs.actima = dram_cs_group_values[4];
	ddr_timings->cs.actimb = dram_cs_group_values[5];
	ddr_timings->cs.dlla = dram_cs_group_values[6];

	return ret;
}

static int beacon_mfg_extract_new_model_name(char *model_name, u32 *model_name_size)
{
	int ret;
	struct id_cookie cookie;

	ret = beacon_mfg_find_model_group_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = id_find_string(&cookie, ID_KEY_model_name, model_name, model_name_size);
	if (ret != ID_EOK) {
		if (ret != -ID_ENOENT) {
			printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		}
		return ret;
	}

	return ret;
}

int beacon_mfg_extract_new_serial_number(u8 *serial_number, u32 *serial_number_size)
{
	int ret;
	struct id_cookie cookie;

	ret = beacon_mfg_find_serialization_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* Find serial_number */
	ret = id_find_string(&cookie, ID_KEY_serial_number, serial_number, serial_number_size);
	if (ret != ID_EOK) {
		if (ret != -ID_ENOENT)
			printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	return ret;
}

int beacon_mfg_extract_new_nvs_data(u8 *nvs_data, u32 *nvs_data_size)
{
	int ret;
	struct id_cookie cookie;

	ret = beacon_mfg_find_serialization_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* Find serial_number */
	ret = id_find_string(&cookie, ID_KEY_nvs, nvs_data, nvs_data_size);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __func__, __LINE__, ret);
		return ret;
	}
	return ret;
}

#if 0
/* Extract GPMC timings for particular CS register */
id_keys_t gpmc_ncs_keys[] = {
	ID_KEY_cs0_group,
	ID_KEY_cs1_group,
	ID_KEY_cs2_group,
	ID_KEY_cs3_group,
	ID_KEY_cs4_group,
	ID_KEY_cs5_group,
	ID_KEY_cs6_group,
};

id_keys_t gpmc_config_reg_keys[] = {
	ID_KEY_config1_reg,
	ID_KEY_config2_reg,
	ID_KEY_config3_reg,
	ID_KEY_config4_reg,
	ID_KEY_config5_reg,
	ID_KEY_config6_reg,
	ID_KEY_config7_reg,
};

#endif

/* Initialize the product ID data and return 0 if found */
static int product_id_init(void)
{
	int ret;

	memset(&id_data, 0, sizeof(id_data));

	ret = id_startup(&id_data);
	if (ret != ID_EOK) {
		return -1;
	}

	return 0;
}

static int logic_has_new_product_id(void)
{
	if (!found_id_data) {
		if (!product_id_init()) {
			found_id_data = 1;
		}
	}
	return found_id_data;
}

static int beacon_mfg_init_new_product_id(void)
{
	if (!logic_has_new_product_id()) {
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		return -ENOENT;
	}

	return 0;
}

/* Extract the Wired LAN ethaddr, and return !0 if its valid */
static int beacon_mfg_extract_new_lan_ethaddr(u8 *ethaddr)
{
	int ret;
	struct id_cookie cookie;
	int ethaddr_size;

	if (!found_id_data) {
		ret = -ENXIO;
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		goto done;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		goto done;
	}

	/* Find /serialization_group */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		goto done;
	}

	/* Find /lan_ethaddr1 */
	ethaddr_size = 6;
	ret = id_find_string(&cookie, ID_KEY_lan_ethaddr1, ethaddr, &ethaddr_size);
	if (ret != ID_EOK) {
		goto done;
	}
	if (ethaddr_size != 6) {
		ret = -E2BIG;
		printk(KERN_ERR "%s:%d ethaddr_size %u\n", __func__, __LINE__, ethaddr_size);
		goto done;
	}
	ret = 0;

done:
	return ret;
}

/* Extract the WiFi ethaddr, and return !0 if its valid */
static int beacon_mfg_extract_new_wifi_ethaddr(u8 *ethaddr)
{
	int ret;
	struct id_cookie cookie;
	int ethaddr_size;

	if (!found_id_data) {
		ret = -ENXIO;
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		goto done;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		goto done;
	}

	/* Find /serialization_group */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d\n", __func__, __LINE__);
		goto done;
	}

	/* Find /lan_ethaddr2 */
	ethaddr_size = 6;
	ret = id_find_string(&cookie, ID_KEY_wifi_ethaddr1, ethaddr, &ethaddr_size);
	if (ret != ID_EOK) {
		goto done;
	}
	if (ethaddr_size != 6) {
		ret = -E2BIG;
		printk(KERN_ERR "%s:%d ethadr_size %d\n", __func__, __LINE__, ethaddr_size);
		goto done;
	}

	ret = 0;
done:
	return ret;
}

static ssize_t product_id_show_wifi_macaddr(struct class *class, struct class_attribute *attr, char *buf)
{
	u8 macaddr[7];
	int ret;
	int len;

	ret = beacon_mfg_extract_new_wifi_ethaddr(macaddr);
	if (!ret) {
		len = sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			macaddr[0], macaddr[1], macaddr[2],
			macaddr[3], macaddr[4], macaddr[5]);
	}
	return len;
}

static ssize_t product_id_show_lan_macaddr(struct class *class, struct class_attribute *attr, char *buf)
{
	u8 macaddr[7];
	int ret;
	int len;

	ret = beacon_mfg_extract_new_lan_ethaddr(macaddr);
	if (!ret) {
		len = sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			macaddr[0], macaddr[1], macaddr[2],
			macaddr[3], macaddr[4], macaddr[5]);
	}
	return len;
}

static ssize_t product_id_show_part_number(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 part_number;
	int len;

	beacon_mfg_extract_new_part_number(&part_number);
	len = sprintf(buf, "%d\n", part_number);
	return len;
}

static ssize_t product_id_show_model_name(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 model_name_size = 128;
	int ret;

	ret = beacon_mfg_extract_new_model_name((u8 *)buf, &model_name_size);

	buf[model_name_size] = '\n';
	return model_name_size + 1;
}

static ssize_t product_id_show_version_code(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 version_code;
	int len;

	beacon_mfg_extract_new_version_code(&version_code);
	len = sprintf(buf, "%u\n", version_code);
	return len;
}

static ssize_t product_id_show_serial_number(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 serial_number_size = 128;

	beacon_mfg_extract_new_serial_number((u8 *)buf, &serial_number_size);
	buf[serial_number_size] = '\n';
	return serial_number_size + 1;
}

static ssize_t product_id_show_speed_mhz(struct class *clase, struct class_attribute *attr, char *buf)
{
	u32 speed_mhz;
	int len;

	beacon_mfg_extract_new_speed_mhz(&speed_mhz);
	len = sprintf(buf, "%u\n", speed_mhz);
	return len;
}

static ssize_t product_id_show_wifi_config_data(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 wifi_config_size = PAGE_SIZE;
	int ret;

	ret = beacon_mfg_extract_new_nvs_data(buf, &wifi_config_size);

	if (ret == ID_EOK)
		return wifi_config_size;

	return ret;
}

#define DUMP_DDR_TIMING(REG) i += sprintf(&buf[i], "%-9s: %08x\n", #REG, ddr_timings. REG)
static ssize_t product_id_show_ddr_timings(struct class *class, struct class_attribute *attr, char *buf)
{
	int i = 0;

	DUMP_DDR_TIMING(sysconfig);
	DUMP_DDR_TIMING(sharing);
	DUMP_DDR_TIMING(power);
	DUMP_DDR_TIMING(cfg);
	DUMP_DDR_TIMING(cs.mcfg);
	DUMP_DDR_TIMING(cs.mr);
	DUMP_DDR_TIMING(cs.rfr);
	DUMP_DDR_TIMING(cs.emr);
	DUMP_DDR_TIMING(cs.actima);
	DUMP_DDR_TIMING(cs.actimb);
	DUMP_DDR_TIMING(cs.dlla);

	return i;
}

static struct {
	struct class_attribute attr;
	int *test_value;
} product_id_class_attributes[] = {
	{
		__ATTR(lan_macaddr, 0444, product_id_show_lan_macaddr, NULL),
		NULL,
	},
	{
		__ATTR(wifi_macaddr, 0444, product_id_show_wifi_macaddr, NULL),
		NULL,
	},
	{
		__ATTR(part_number, 0444, product_id_show_part_number, NULL),
		NULL,
	},
	{
		__ATTR(version_code, 0444, product_id_show_version_code, NULL),
		NULL,
	},
	{
		__ATTR(model_name, 0444, product_id_show_model_name, NULL),
		NULL,
	},
	{
		__ATTR(serial_number, 0444, product_id_show_serial_number, NULL),
		NULL,
	},
	{
		__ATTR(speed_mhz, 0444, product_id_show_speed_mhz, NULL),
		NULL,
	},
	{
		__ATTR(wifi_config_data, 0444, product_id_show_wifi_config_data, NULL),
		NULL,
	},
	{
		__ATTR(ddr_timings, 0444, product_id_show_ddr_timings, NULL),
		NULL,
	},
};

static void product_id_dev_release(struct device *dev)
{
}

static struct class product_id_class = {
	.name = "product_id",
	.dev_release = product_id_dev_release,
};

static int beacon_mfg_create_new_product_id_sysfs(void)
{
	int i, rc;

	rc = class_register(&product_id_class);
	if (rc != 0) {
		printk(KERN_ERR "%s: failed to register product_id class\n", __func__);
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(product_id_class_attributes); ++i) {
		if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
			rc = class_create_file(&product_id_class, &product_id_class_attributes[i].attr);
			if (unlikely(rc)) {
				printk(KERN_ERR "%s: failed to create product_id class file\n", __func__);
				while (--i >= 0) {
					if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
						class_remove_file(&product_id_class, &product_id_class_attributes[i].attr);
					}
				}
				class_unregister(&product_id_class);
				return -EPERM;
			}
		}
	}

	return 0;
}

static int beacon_mfg_extract_nvs_data(u8 *nvs_data, u32 *nvs_data_size)
{
	return beacon_mfg_extract_new_nvs_data(nvs_data, nvs_data_size);
}
/* Extract the version code for the SOM */
int beacon_mfg_extract_version_code(void)
{
	int err;
	u32 version_code;

	err = beacon_mfg_extract_new_version_code(&version_code);
	if (!err)
		return version_code;

	return -EINVAL;
}

#ifdef EEPROM_PATH


static inline ssize_t
spl_kernel_read(struct file *file, void *buf, size_t count, loff_t *pos)
{
	return kernel_read(file, buf, count, pos);
}

static int read_eeprom(void)
{

	struct file *f;
	mm_segment_t fs;
	loff_t pos = 0;
	long sz;

	struct id_header hdr;
	struct id_checksums xsums;

	f = filp_open(EEPROM_PATH, O_RDONLY, 0);
	if (IS_ERR(f))
		printk(KERN_ALERT "filp_open error!!.\n");
	else{
		fs = get_fs();
		set_fs(KERNEL_DS);

		/* Read the header */
		sz = sizeof(hdr);
		spl_kernel_read(f, (void *)&hdr, sz, &pos);

		/* Read the checksums */
		sz = sizeof(xsums);
		spl_kernel_read(f, (void *)&xsums, sz, &pos);

		/* Size of data = header + 2 LE Checksums + data_length */
		sz = sizeof(hdr) + sizeof(xsums) + hdr.data_length;

		id_data_buf = (char *)kzalloc(sz, GFP_KERNEL);
		if (id_data_buf == NULL) {
			printk("KZMALLOC ERROR\n");
			filp_close(f, NULL);
			return -1;
		}
		/* Copy the header into the final space */
		memcpy(&id_data_buf[0], &hdr, sizeof(hdr));

		/* Copy the checksums into the final space */
		memcpy(&(id_data_buf[sizeof(hdr)]), &xsums, sizeof(xsums));

		/* Only read the data_length worth of data */
		sz = hdr.data_length;
		spl_kernel_read(f, (void *)&id_data_buf[sizeof(hdr) + sizeof(xsums)], sz, &pos);
		set_fs(fs);
		filp_close(f, NULL);
		return 0;
	}
	return -1;
}
#endif

static int __init productid_init(void)
{
#ifdef EEPROM_PATH
	read_eeprom();
#endif
	beacon_mfg_init_new_product_id();
	beacon_mfg_create_new_product_id_sysfs();
	return 0;
}

module_init(productid_init);

static void __exit productid_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(product_id_class_attributes); ++i) {
		class_remove_file(&product_id_class, &product_id_class_attributes[i].attr);
	}
	class_unregister(&product_id_class);
}

module_exit(productid_exit);

EXPORT_SYMBOL(beacon_mfg_extract_nvs_data);
EXPORT_SYMBOL(beacon_mfg_extract_version_code);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Adam Ford <aford@beaconembedded.com>");
MODULE_DESCRIPTION("Beacon EmbeddedWorks EEPROM reader");
MODULE_VERSION("printk");

