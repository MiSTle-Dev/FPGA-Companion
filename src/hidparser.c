// http://www.frank-zhao.com/cache/hid_tutorial_1.php

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hidparser.h"
#include "debug.h"

#if 1
#define hidp_extreme_debugf(...) hidp_debugf(__VA_ARGS__)
#else
#define hidp_extreme_debugf(...)
#endif

typedef struct {
  uint8_t bSize: 2;
  uint8_t bType: 2;
  uint8_t bTag: 4;
} __attribute__((packed)) item_t;

// flags for joystick components required
#define JOY_MOUSE_REQ_AXIS_X  0x01
#define JOY_MOUSE_REQ_AXIS_Y  0x02
#define JOY_MOUSE_REQ_BTN_0   0x04
#define JOY_MOUSE_REQ_BTN_1   0x08
#define JOYSTICK_COMPLETE     (JOY_MOUSE_REQ_AXIS_X | JOY_MOUSE_REQ_AXIS_Y | JOY_MOUSE_REQ_BTN_0)
#define MOUSE_COMPLETE        (JOY_MOUSE_REQ_AXIS_X | JOY_MOUSE_REQ_AXIS_Y | JOY_MOUSE_REQ_BTN_0 | JOY_MOUSE_REQ_BTN_1)

#define USAGE_PAGE_GENERIC_DESKTOP  1
#define USAGE_PAGE_SIMULATION       2
#define USAGE_PAGE_VR               3
#define USAGE_PAGE_SPORT            4
#define USAGE_PAGE_GAMING           5
#define USAGE_PAGE_GENERIC_DEVICE   6
#define USAGE_PAGE_KEYBOARD         7
#define USAGE_PAGE_LEDS             8
#define USAGE_PAGE_BUTTON           9
#define USAGE_PAGE_ORDINAL         10
#define USAGE_PAGE_TELEPHONY       11
#define USAGE_PAGE_CONSUMER        12


#define USAGE_POINTER   1
#define USAGE_MOUSE     2
#define USAGE_JOYSTICK  4
#define USAGE_GAMEPAD   5
#define USAGE_KEYBOARD  6
#define USAGE_KEYPAD    7
#define USAGE_MULTIAXIS 8

#define USAGE_X       48
#define USAGE_Y       49
#define USAGE_Z       50
#define USAGE_RX      51
#define USAGE_RY      52
#define USAGE_RZ      53
#define USAGE_WHEEL   56
#define USAGE_HAT     57

// ---- helpers ----
#define MIN(a, b) ((a) < (b) ? (a) : (b))

static inline int32_t sign_extend(uint32_t v, uint8_t size_bytes)
{
    if (size_bytes == 1)
        return (int32_t)(int8_t)v;
    if (size_bytes == 2)
        return (int32_t)(int16_t)v;
    return (int32_t)v;
}

#ifndef HID_LOCAL_USAGE_MAX
#define HID_LOCAL_USAGE_MAX 32
#endif

typedef struct
{
    uint8_t report_size;
    uint8_t report_count;
    int32_t logical_min;
    int32_t logical_max;
    int32_t physical_min;
    int32_t physical_max;
    uint16_t usage_page;
    uint8_t report_id_present;
    uint8_t report_id;
} hid_globals_t;

#ifndef HID_GLOBAL_STACK_DEPTH
#define HID_GLOBAL_STACK_DEPTH 4
#endif

// ---------------- core ----------------

static bool report_is_usable(uint16_t bit_count, uint8_t report_complete, hid_report_t *conf) {
	hidp_debugf("  - total bit count: %d (%d bytes, %d bits)", 
	      bit_count, bit_count/8, bit_count%8);

    // FIX: round report size up
    conf->report_size = (bit_count + 7) / 8;

    if (((conf->type == REPORT_TYPE_JOYSTICK) && ((report_complete & JOYSTICK_COMPLETE) == JOYSTICK_COMPLETE)) ||
        ((conf->type == REPORT_TYPE_MOUSE) && ((report_complete & MOUSE_COMPLETE) == MOUSE_COMPLETE)) ||
        (conf->type == REPORT_TYPE_KEYBOARD))
    {
	hidp_debugf("  - report %d is usable", conf->report_id);

        hidp_debugf("=== RETURNED STRUCTURE ===");
        hidp_debugf("conf->type = %d", conf->type);
        hidp_debugf("conf->report_id_present = %d", conf->report_id_present);
        hidp_debugf("conf->report_id = %d", conf->report_id);
        hidp_debugf("conf->report_size = %d", conf->report_size);

        if (conf->type == REPORT_TYPE_JOYSTICK || conf->type == REPORT_TYPE_MOUSE)
        {
            hidp_debugf("JOYSTICK/MOUSE DATA:");

            for (int i = 0; i < MAX_AXES; i++)
            {
                hidp_debugf("conf->joystick_mouse.axis[%d].offset = %d", i, conf->joystick_mouse.axis[i].offset);
                hidp_debugf("conf->joystick_mouse.axis[%d].size = %d", i, conf->joystick_mouse.axis[i].size);
                hidp_debugf("conf->joystick_mouse.axis[%d].logical.min = %d", i, conf->joystick_mouse.axis[i].logical.min);
                hidp_debugf("conf->joystick_mouse.axis[%d].logical.max = %d", i, conf->joystick_mouse.axis[i].logical.max);
            }

            for (int i = 0; i < 32; i++)
            { // or MAX_HID_BUTTONS
                if (conf->joystick_mouse.button[i].bitmask != 0)
                {
                    hidp_debugf("conf->joystick_mouse.button[%d].byte_offset = %d", i, conf->joystick_mouse.button[i].byte_offset);
                    hidp_debugf("conf->joystick_mouse.button[%d].bitmask = %d", i, conf->joystick_mouse.button[i].bitmask);
                }
            }

            hidp_debugf("conf->joystick_mouse.hat.offset = %d", conf->joystick_mouse.hat.offset);
            hidp_debugf("conf->joystick_mouse.hat.size = %d", conf->joystick_mouse.hat.size);
            hidp_debugf("conf->joystick_mouse.hat.logical.min = %d", conf->joystick_mouse.hat.logical.min);
            hidp_debugf("conf->joystick_mouse.hat.logical.max = %d", conf->joystick_mouse.hat.logical.max);
        }

        hidp_debugf("===========================");

	return true;
	}

	hidp_debugf("  - unusable report %d", conf->report_id);
	return false;
}

bool parse_report_descriptor(const uint8_t *rep, uint16_t rep_size, hid_report_t *conf, uint16_t *rbytes) {
	int8_t app_collection = 0;
	int8_t phys_log_collection = 0;
	uint8_t skip_collection = 0;
	int8_t generic_desktop = -1;   // depth at which first gen_desk was found
	uint8_t collection_depth = 0;

	uint8_t i;

	//
	uint8_t buttons = 0;
	uint8_t report_size = 0, report_count = 0;
    uint16_t bit_count = 0;
    int32_t logical_minimum=0, logical_maximum=0;
    int32_t physical_minimum=0, physical_maximum=0;

    // FIX: set active Report ID at the end
    uint8_t active_report_id = 0;
    bool have_active_report_id = false;
    bool report_has_input = false;

	memset(conf, 0, sizeof(hid_report_t));

	uint8_t report_complete = 0;

	// joystick/mouse components
	int8_t axis[MAX_AXES];
	uint8_t btns = 0;
	int8_t hat = -1;

	for (i=0; i<MAX_AXES; i++) axis[i] = -1;

	conf->type = REPORT_TYPE_NONE;

    uint16_t usage_list[HID_LOCAL_USAGE_MAX];
    uint8_t usage_list_len = 0;
    uint16_t usage_min = 0, usage_max = 0;
    bool usage_range_set = false;

    hid_globals_t g = (hid_globals_t){0};
    hid_globals_t gstack[HID_GLOBAL_STACK_DEPTH];
    uint8_t gsp = 0;

    if (rbytes)
        *rbytes = 0;

    while (rep_size)
    {

        // FIX: long item (0xFE)
        if (*rep == 0xFE)
        {
            if (rep_size < 3)
            {
                hidp_debugf("descriptor truncated at long item header");
                return false;
            }
            uint8_t long_size = rep[1];
            uint8_t long_tag = rep[2];
            (void)long_tag;
            rep += 3;
            rep_size -= 3;
            if (rep_size < long_size)
            {
                hidp_debugf("descriptor truncated in long item payload");
                return false;
            }
            rep += long_size;
            rep_size -= long_size;
            if (rbytes)
                *rbytes += (uint16_t)(3 + long_size);
            hidp_extreme_debugf("LONG_ITEM(tag=0x%02x, size=%u) skipped", long_tag, long_size);
            continue;
		}

        if (rep_size < 1)
            break;

        uint8_t prefix = *rep;
        item_t hdr = *(const item_t *)rep;

        uint8_t tag = hdr.bTag;
        uint8_t type = hdr.bType;
        uint8_t size = hdr.bSize;

        rep++;
			rep_size--;
        if (rbytes)
            (*rbytes)++;

        uint8_t data_len = (size == 3) ? 4 : size;
        if (rep_size < data_len)
        {
            hidp_debugf("descriptor truncated (need %u bytes, have %u)", data_len, rep_size);
            return false;
        }

        uint32_t value = 0;
        if (data_len >= 1)
            value |= (uint32_t)rep[0];
        if (data_len >= 2)
            value |= ((uint32_t)rep[1] << 8);
        if (data_len >= 3)
            value |= ((uint32_t)rep[2] << 16);
        if (data_len >= 4)
            value |= ((uint32_t)rep[3] << 24);

        rep += data_len;
        rep_size -= data_len;
        if (rbytes)
            (*rbytes) += data_len;

        if (skip_collection)
        {
            if (!type)
            {
                if (tag == 10)
                {
					skip_collection++;
					collection_depth++;
				}
                if (tag == 12)
                {
					skip_collection--;
					collection_depth--;
                    if (generic_desktop > collection_depth)
						generic_desktop = -1;
				}
			}
            continue;
        }

        switch (type)
        {
        case 0: // MAIN
            switch (tag)
            {
				case 8:
            { // INPUT
                hidp_extreme_debugf("INPUT(%lu)", value);

                // Decode flag (HID 1.11 §6.2.2.4)
                const bool is_constant = (value & 0x01) != 0; // 0=Data, 1=Constant

                // If Constant → ignore data and increase bit_count
                if (is_constant)
                {
                    hidp_debugf("  -> INPUT Constant: skip as data (padding), size=%u count=%u", report_size, report_count);

                    bit_count += (uint16_t)report_count * (uint16_t)report_size;
                    report_has_input = true;

                    usage_list_len = 0;
                    usage_range_set = false;
                    btns = 0;
                    buttons = 0;
                    break; 
                }

                // skip if suspicious size
                if ((conf->type == REPORT_TYPE_JOYSTICK || conf->type == REPORT_TYPE_MOUSE) &&
                    (report_count > 32))
                {
                    hidp_debugf("  -> INPUT ignored: probably Vendor specific data");
                    bit_count += (uint16_t)report_count * (uint16_t)report_size;

                    break;
                }

                uint16_t local_usage_at_pos[HID_LOCAL_USAGE_MAX];
                uint8_t assignable = MIN(report_count, (uint8_t)HID_LOCAL_USAGE_MAX);
                uint8_t pos = 0;
                for (; pos < assignable && pos < usage_list_len; ++pos)
                    local_usage_at_pos[pos] = usage_list[pos];
                if (pos < assignable && usage_range_set)
                {
                    uint16_t u = usage_min;
                    for (; pos < assignable && u <= usage_max; ++pos, ++u)
                        local_usage_at_pos[pos] = u;
                }

                // buttons
                if (btns && (conf->type == REPORT_TYPE_JOYSTICK || conf->type == REPORT_TYPE_MOUSE))
                {
                    uint8_t max_buttons = MIN(report_count, (uint8_t)32); // If more than 12 modify hid_report_t 
                    for (uint8_t b = 0; b < max_buttons; b++)
                    {
                        uint16_t this_bit = bit_count + b;
                        hidp_debugf("BUTTON%d @ %d (byte %d, mask %d)", b, this_bit, this_bit / 8, 1 << (this_bit % 8));
                        conf->joystick_mouse.button[b].byte_offset = this_bit / 8;
                        conf->joystick_mouse.button[b].bitmask = 1 << (this_bit % 8);
                        buttons = b + 1;
								}
                    if (buttons >= 1)
							report_complete |= JOY_MOUSE_REQ_BTN_0;
                    if (buttons >= 2)
                        report_complete |= JOY_MOUSE_REQ_BTN_1;
					}

                // axes / hat
                for (int c = 0; c < MAX_AXES; c++)
                    axis[c] = -1;
                hat = -1;

                if (conf->type == REPORT_TYPE_JOYSTICK || conf->type == REPORT_TYPE_MOUSE)
                {
                    for (uint8_t idx = 0; idx < assignable; ++idx)
                    {
                        uint16_t u = local_usage_at_pos[idx];
                        if (u == USAGE_X)
                        {
                            if (axis[0] == -1)
                                axis[0] = idx;
                        }
                        else if (u == USAGE_Y)
                        {
                            if (axis[1] == -1)
                                axis[1] = idx;
                        }
                        else if (u == USAGE_Z)
                        {
                            if (axis[2] == -1)
                                axis[2] = idx;
                        }
                        else if (u == USAGE_RX)
                        {
                            if (axis[3] == -1)
                                axis[3] = idx;
                        }
                        else if (u == USAGE_RY)
                        {
                            if (axis[4] == -1)
                                axis[4] = idx;
                        }
                        else if (u == USAGE_RZ)
                        {
                            if (axis[5] == -1)
                                axis[5] = idx;
                        }
                        else if (u == USAGE_WHEEL)
                            axis[2] = idx;
                        else if (u == USAGE_HAT)
                            hat = idx;
                    }
                }

					for(int c=0;c<MAX_AXES;c++) {
						if(axis[c] >= 0) {
							uint16_t cnt = bit_count + report_size * axis[c];
							hidp_debugf("  (%c-AXIS @ %d (byte %d, bit %d))", 'X'+c,
							  cnt, cnt/8, cnt&7);

							if((conf->type == REPORT_TYPE_JOYSTICK) || (conf->type == REPORT_TYPE_MOUSE)) {
								// save in joystick report
								conf->joystick_mouse.axis[c].offset = cnt;
								conf->joystick_mouse.axis[c].size = report_size;
								conf->joystick_mouse.axis[c].logical.min = logical_minimum;
								conf->joystick_mouse.axis[c].logical.max = logical_maximum;
								if(c==0) report_complete |= JOY_MOUSE_REQ_AXIS_X;
								if(c==1) report_complete |= JOY_MOUSE_REQ_AXIS_Y;
							}
						}
					}

                if (hat >= 0 && conf->type == REPORT_TYPE_JOYSTICK) {
					uint16_t cnt = bit_count + report_size * hat;
                    hidp_debugf("  (HAT @ %d (byte %d, bit %d), size %d)", cnt, cnt / 8, cnt & 7, report_size);
					conf->joystick_mouse.hat.offset = cnt;
					conf->joystick_mouse.hat.size = report_size;
                    conf->joystick_mouse.hat.logical.min = (int16_t)logical_minimum;
                    conf->joystick_mouse.hat.logical.max = (int16_t)logical_maximum;
                    conf->joystick_mouse.hat.physical.min = (int16_t)physical_minimum;
                    conf->joystick_mouse.hat.physical.max = (int16_t)physical_maximum;
					}

                bit_count += (uint16_t)report_count * (uint16_t)report_size;
                report_has_input = true;

                usage_list_len = 0;
                usage_range_set = false;
					btns = 0;
                buttons = 0;
					break;
            }

				case 9:
					hidp_extreme_debugf("OUTPUT(%lu)", value);
					break;
				case 11:
					hidp_extreme_debugf("FEATURE(%lu)", value);
					break;

            case 10: // COLLECTION
					hidp_extreme_debugf("COLLECTION(%lu)", value);
					collection_depth++;
                usage_list_len = 0;
                usage_range_set = false;
                if (value == 1)
                {
						hidp_extreme_debugf("  -> application");
						app_collection++;
                }
                else if (value == 0)
                {
						hidp_extreme_debugf("  -> physical");
						phys_log_collection++;
                }
                else if (value == 2)
                {
						hidp_extreme_debugf("  -> logical");
						phys_log_collection++;
                }
                else
                {
						hidp_extreme_debugf("skipping unsupported collection");
						skip_collection++;
					}
					break;

            case 12: // END_COLLECTION
					hidp_extreme_debugf("END_COLLECTION(%lu)", value);
                if (collection_depth == 0)
                {
                    hidp_debugf("END_COLLECTION underflow");
                    return false;
                }

					collection_depth--;
                if (generic_desktop > collection_depth)
						generic_desktop = -1;

                if (phys_log_collection)
                {
						hidp_extreme_debugf("  -> phys/log end");
						phys_log_collection--;
                }
                else if (app_collection)
                {
						hidp_extreme_debugf("  -> app end");
						app_collection--;

                    if (report_has_input && report_is_usable(bit_count, report_complete, conf))
                    {
                        conf->report_id_present = have_active_report_id;
                        conf->report_id = have_active_report_id ? active_report_id : 0;
						        return true;
                    }
                    else
                    {
							bit_count = 0;
							report_complete = 0;
                        report_has_input = false;
                    }
                }
                else
                {
                    hidp_debugf(" -> END_COLLECTION outside of any collection");
						return false;
					}
					break;

				default:
                hidp_debugf("unexpected main item tag=%u (prefix=0x%02x) – skipping", tag, prefix);
					break;
				}
				break;

        case 1: // GLOBAL
            switch (tag)
            {
            case 0: // USAGE_PAGE
					hidp_extreme_debugf("USAGE_PAGE(%lu/0x%lx)", value, value);
                g.usage_page = (uint16_t)value;
                if (value == USAGE_PAGE_KEYBOARD)
						hidp_extreme_debugf(" -> Keyboard");
                else if (value == USAGE_PAGE_GAMING)
						hidp_extreme_debugf(" -> Game device");
                else if (value == USAGE_PAGE_LEDS)
						hidp_extreme_debugf(" -> LEDs");
                else if (value == USAGE_PAGE_CONSUMER)
						hidp_extreme_debugf(" -> Consumer");
                else if (value == USAGE_PAGE_BUTTON)
                {
						hidp_extreme_debugf(" -> Buttons");
						btns = 1;
                }
                else if (value == USAGE_PAGE_GENERIC_DESKTOP)
                {
						hidp_extreme_debugf(" -> Generic Desktop");
                    if (generic_desktop < 0)
							generic_desktop = collection_depth;
                }
                else
						hidp_extreme_debugf(" -> UNSUPPORTED USAGE_PAGE");
					break;

				case 1:
                logical_minimum = sign_extend(value, data_len);
                hidp_extreme_debugf("LOGICAL_MINIMUM(%ld)", (long)logical_minimum);
					break;
				case 2:
                logical_maximum = sign_extend(value, data_len);
                hidp_extreme_debugf("LOGICAL_MAXIMUM(%ld)", (long)logical_maximum);
					break;
				case 3:
                physical_minimum = sign_extend(value, data_len);
                hidp_extreme_debugf("PHYSICAL_MINIMUM(%ld)", (long)physical_minimum);
					break;
				case 4:
                physical_maximum = sign_extend(value, data_len);
                hidp_extreme_debugf("PHYSICAL_MAXIMUM(%ld)", (long)physical_maximum);
					break;

				case 5:
					hidp_extreme_debugf("UNIT_EXPONENT(%lu)", value);
					break;
				case 6:
					hidp_extreme_debugf("UNIT(%lu)", value);
					break;

				case 7:
                report_size = (uint8_t)value;
                g.report_size = report_size;
                hidp_extreme_debugf("REPORT_SIZE(%u)", report_size);
                break;
            case 9:
                report_count = (uint8_t)value;
                g.report_count = report_count;
                hidp_extreme_debugf("REPORT_COUNT(%u)", report_count);
					break;

				case 8:
            { // REPORT_ID
                uint8_t new_id = (uint8_t)value;

                if (have_active_report_id && report_has_input && report_is_usable(bit_count, report_complete, conf))
                {
					conf->report_id_present = 1;
                    conf->report_id = active_report_id; // prev ID
                    return true;
                }

                active_report_id = new_id;
                have_active_report_id = true;

                bit_count = 0;
                report_complete = 0;
                report_has_input = false;

                usage_list_len = 0;
                usage_range_set = false;
                btns = 0;
                buttons = 0;
                for (i = 0; i < MAX_AXES; i++)
                    axis[i] = -1;
                hat = -1;

                hidp_extreme_debugf("REPORT_ID(%u)", new_id);
                break;
            }

            case 10: // PUSH
                if (gsp < HID_GLOBAL_STACK_DEPTH)
                {
                    g.logical_min = logical_minimum;
                    g.logical_max = logical_maximum;
                    g.physical_min = physical_minimum;
                    g.physical_max = physical_maximum;
                    gstack[gsp++] = g;
                    hidp_extreme_debugf("PUSH (gsp=%u)", gsp);
                }
                else
                {
                    hidp_debugf("PUSH ignored (global stack overflow)");
                }
					break;

            case 11: // POP
                if (gsp > 0)
                {
                    g = gstack[--gsp];
                    logical_minimum = g.logical_min;
                    logical_maximum = g.logical_max;
                    physical_minimum = g.physical_min;
                    physical_maximum = g.physical_max;
                    report_size = g.report_size;
                    report_count = g.report_count;

                    hidp_extreme_debugf("POP (gsp=%u)", gsp);
                }
                else
                {
                    hidp_debugf("POP underflow – ignored");
                }
					break;

				default:
                hidp_debugf("unexpected global item %u – skipping", tag);
					break;
				}
				break;

        case 2: // LOCAL
            switch (tag)
            {
            case 0: // USAGE
					hidp_extreme_debugf("USAGE(%lu/0x%lx)", value, value);
                if (usage_list_len < HID_LOCAL_USAGE_MAX)
                    usage_list[usage_list_len++] = (uint16_t)value;

                if (!collection_depth && value == USAGE_KEYBOARD)
                {
						hidp_debugf(" -> Keyboard");
						conf->type = REPORT_TYPE_KEYBOARD;
                }
                else if (!collection_depth && value == USAGE_MOUSE)
                {
						hidp_debugf(" -> Mouse");
						conf->type = REPORT_TYPE_MOUSE;
                }
                else if (!collection_depth && (value == USAGE_GAMEPAD || value == USAGE_JOYSTICK))
                {
							hidp_debugf("Gamepad/Joystick usage found");
							conf->type = REPORT_TYPE_JOYSTICK;
                }
                else if (value == USAGE_POINTER && app_collection)
                {
                    hidp_debugf(" -> Pointer");
                }
					break;

				case 1:
                usage_min = (uint16_t)value;
                usage_range_set = true;
                hidp_extreme_debugf("USAGE_MINIMUM(%u)", usage_min);
					break;
				case 2:
                usage_max = (uint16_t)value;
                usage_range_set = true;
                hidp_extreme_debugf("USAGE_MAXIMUM(%u)", usage_max);
					break;

				default:
                hidp_extreme_debugf("unexpected local item %u – skipping", tag);
					break;
				}
				break;

        default: // reserved
            hidp_extreme_debugf("unexpected reserved item type=%u tag=%u – skipping", type, tag);
				break;
			}
		}


    if (have_active_report_id && report_has_input && report_is_usable(bit_count, report_complete, conf))
    {
        conf->report_id_present = 1;
        conf->report_id = active_report_id;
        return true;
    }

	return false;
}
