#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
import sys
from urllib.request import urlopen

DB_URL = "https://raw.githubusercontent.com/gabomdq/SDL_GameControllerDB/master/gamecontrollerdb.txt"
HEADER_GUARD = "USB_CONTROLLER_MAPS_H_"

C_HEADER_PREAMBLE = r"""\
/*
 * Auto-generated from SDL_GameControllerDB (USB-only).
 * Source: {src}
 *
 * GUID -> VID/PID/Version decoded as little-endian (matches SDL_GetJoystickGUIDInfo).
 * Mappings follow SDL GameController semantics.
 *
 * NOTE for BL616 (m0s dock / Bouffalo SDK):
 * - All data are const and can be placed in dedicated flash sections to avoid RAM usage.
 * - Add the following to your linker script to pin sections in FLASH (XIP):
 *
 *   .rodata.controllerdb :
 *   {{
 *     KEEP(*(.rodata.controllerdb))
 *     KEEP(*(.rodata.controllerdb.*))
 *   }} > FLASH
 *
 *   .rodata.controllerdb.str :
 *   {{
 *     KEEP(*(.rodata.controllerdb.str))
 *     KEEP(*(.rodata.controllerdb.str.*))
 *   }} > FLASH
 */

#ifndef {guard}
#define {guard}

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* Section attributes: keep in flash, never pulled into RAM */
#ifndef CONTROLLERDB_ATTR
#define CONTROLLERDB_ATTR __attribute__((section(".rodata.controllerdb"), used, aligned(4)))
#endif

#ifndef CONTROLLERDB_STR_ATTR
#define CONTROLLERDB_STR_ATTR __attribute__((section(".rodata.controllerdb.str"), used, aligned(1)))
#endif

/* Helper macro for defining per-entry strings in the string section. */
#define CONTROLLERDB_STR(sym, literal) CONTROLLERDB_STR_ATTR static const char sym[] = literal
"""

C_HEADER_POSTAMBLE = r"""
#ifdef __cplusplus
}}
#endif

#endif /* {guard} */
"""

GUID_RE = re.compile(r"^[0-9a-fA-F]{32}$")

def hexstr_to_bytes_16(hexstr: str) -> bytes:
    if not GUID_RE.fullmatch(hexstr):
        raise ValueError(f"Invalid GUID: {hexstr!r}")
    return bytes.fromhex(hexstr)

def is_usb_guid(guid_hex: str) -> bool:
    return guid_hex.lower().startswith("03000000")  # USB == 0x00000003

def extract_vid_pid_version(guid_hex: str):
    b = hexstr_to_bytes_16(guid_hex.lower())
    vid = b[4] | (b[5] << 8)
    pid = b[8] | (b[9] << 8)
    ver = b[12] | (b[13] << 8)
    return vid, pid, ver

BTN_KEYS = {
    "a": "btn_a", "b": "btn_b", "x": "btn_x", "y": "btn_y",
    "back": "btn_back", "guide": "btn_guide", "start": "btn_start",
    "leftshoulder": "btn_leftshoulder", "rightshoulder": "btn_rightshoulder",
    "leftstick": "btn_leftstick", "rightstick": "btn_rightstick",
    "dpup": "btn_dpad_up", "dpdown": "btn_dpad_down",
    "dpleft": "btn_dpad_left", "dpright": "btn_dpad_right",
}

AXIS_KEYS = {
    "leftx": "axis_lx", "lefty": "axis_ly", "rightx": "axis_rx", "righty": "axis_ry",
    "lefttrigger": "axis_lt", "righttrigger": "axis_rt",

    "dpdown": "axis_ly", "dpright": "axis_lx"
}

def c_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n")

def parse_mapping_to_record(mapping_str: str):
    rec = {
        **{v: -1 for v in BTN_KEYS.values()},
        "dpad_hat": -1, "dpad_hat_up": 0, "dpad_hat_right": 0,
        "dpad_hat_down": 0, "dpad_hat_left": 0,
        **{v: -1 for v in AXIS_KEYS.values()},
        "axis_lx_invert": 0, "axis_ly_invert": 0,
        "axis_rx_invert": 0, "axis_ry_invert": 0,
        "axis_lt_invert": 0, "axis_rt_invert": 0,
    }

    for part in [p.strip() for p in mapping_str.split(",") if p.strip()]:
        if ":" not in part: continue
        key, val = part.split(":", 1)
        key, val = key.strip().lower(), val.strip().lower()
        if key in ("platform","crc","type"): continue
        if key in BTN_KEYS:
            if val.startswith("b"):
                try: rec[BTN_KEYS[key]] = int(val[1:])
                except: pass
            elif val.startswith("h") and key.startswith("dp"):
                m = re.match(r"h(\d+)\.(\d+)", val)
                if m:
                    hat, mask = int(m.group(1)), int(m.group(2))
                    rec["dpad_hat"] = hat
                    if key=="dpup": rec["dpad_hat_up"]=mask
                    elif key=="dpright": rec["dpad_hat_right"]=mask
                    elif key=="dpdown": rec["dpad_hat_down"]=mask
                    elif key=="dpleft": rec["dpad_hat_left"]=mask                    
        if key in AXIS_KEYS:
            invert = 0
            if val.endswith("~"): invert, val = 1, val[:-1]
            if val.startswith("-"): invert, val = 1, val[1:]
            if val.startswith("+"): invert, val = 0, val[1:]
            if val.startswith("a"):
                try:
                    idx = int(val[1:])
                    rec[AXIS_KEYS[key]] = idx
                    rec[AXIS_KEYS[key]+"_invert"]=invert
                except: pass
    return rec

def parse_db_lines_usb(lines):
    out=[]
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("//"): continue
        line = re.split(r"\s//", line, maxsplit=1)[0].strip()
        parts=line.split(",")
        if len(parts)<3: continue
        guid=parts[0].strip()
        if not GUID_RE.fullmatch(guid) or not is_usb_guid(guid): continue
        name=parts[1].strip()
        mapping=",".join(parts[2:]).strip()
        try: vid,pid,ver=extract_vid_pid_version(guid)
        except: vid=pid=ver=0
        rec=parse_mapping_to_record(mapping)
        out.append({"guid":guid,"name":name,"mapping_raw":mapping,
                    "vid":vid,"pid":pid,"version":ver,"rec":rec})
    return out

def read_from_url_or_file(src):
    if re.match(r"^https?://", src):
        with urlopen(src) as resp:
            return resp.read().decode("utf-8", errors="replace").splitlines()
    return open(src,"r",encoding="utf-8").read().splitlines()

def sort_entries(entries,mode):
    if mode=="name": return sorted(entries,key=lambda e:(e["name"].lower(),e["vid"],e["pid"]))
    if mode=="vidpid": return sorted(entries,key=lambda e:(e["vid"],e["pid"],e["name"].lower()))
    return entries

def emit_typedef(f, include_map):
    f.write(r"""
typedef struct {
    const char *guid; const char *name;
    uint16_t vid, pid, version;
    int8_t btn_a,btn_b,btn_x,btn_y;
    int8_t btn_back,btn_guide,btn_start;
    int8_t btn_leftshoulder,btn_rightshoulder;
    int8_t btn_leftstick,btn_rightstick;
    int8_t btn_dpad_up,btn_dpad_down,btn_dpad_left,btn_dpad_right;
    int8_t dpad_hat; uint8_t dpad_hat_up,dpad_hat_right,dpad_hat_down,dpad_hat_left;
    int8_t axis_lx,axis_ly,axis_rx,axis_ry;
    uint8_t axis_lx_invert,axis_ly_invert,axis_rx_invert,axis_ry_invert;
    int8_t axis_lt,axis_rt; uint8_t axis_lt_invert,axis_rt_invert;""")
    if include_map: f.write("\n    const char *mapping_raw;")
    f.write("\n} UsbGamepadMap;\n\n")

def write_c_header(entries, out_path, source_url, strings_mode, include_map, no_comments):
    with open(out_path,"w",encoding="utf-8",newline="\n") as f:
        f.write(C_HEADER_PREAMBLE.format(src=source_url,guard=HEADER_GUARD))
        emit_typedef(f,include_map)

        if strings_mode=="split":
            for i,e in enumerate(entries):
                f.write(f'CONTROLLERDB_STR(g_guid_{i}, "{c_escape(e["guid"])}");\n')
                f.write(f'CONTROLLERDB_STR(g_name_{i}, "{c_escape(e["name"])}");\n')
                if include_map:
                    f.write(f'CONTROLLERDB_STR(g_map_{i}, "{c_escape(e["mapping_raw"])}");\n')
            f.write("\n")

        f.write("CONTROLLERDB_ATTR\nstatic const UsbGamepadMap kUsbGamepadMaps[] = {\n")

        btn=["btn_a","btn_b","btn_x","btn_y","btn_back","btn_guide","btn_start",
             "btn_leftshoulder","btn_rightshoulder","btn_leftstick","btn_rightstick",
             "btn_dpad_up","btn_dpad_down","btn_dpad_left","btn_dpad_right"]
        hat=["dpad_hat","dpad_hat_up","dpad_hat_right","dpad_hat_down","dpad_hat_left"]
        axes=["axis_lx","axis_ly","axis_rx","axis_ry","axis_lt","axis_rt"]
        inv=["axis_lx_invert","axis_ly_invert","axis_rx_invert","axis_ry_invert","axis_lt_invert","axis_rt_invert"]

        for i,e in enumerate(entries):
            r=e["rec"]
            if strings_mode=="split":
                guid=f"g_guid_{i}"; name=f"g_name_{i}"; mapf=f"g_map_{i}" if include_map else None
            else:
                guid=f"\"{c_escape(e['guid'])}\""; name=f"\"{c_escape(e['name'])}\""
                mapf=f"\"{c_escape(e['mapping_raw'])}\"" if include_map else None

            f.write("  { ")
            f.write(f"{guid}, {name}, 0x{e['vid']:04X}, 0x{e['pid']:04X}, 0x{e['version']:04X}, ")
            f.write(", ".join(str(int(r[k])) for k in btn)+", ")
            f.write(", ".join(str(int(r[k])) for k in hat)+", ")
            f.write(", ".join(str(int(r[k])) for k in axes)+", ")
            f.write(", ".join(str(int(r[k])) for k in inv))
            if include_map: f.write(f", {mapf}")
            f.write(" },")
            if not no_comments:
                preview=e["mapping_raw"]
                if len(preview)>200: preview=preview[:200]+"...(trimmed)"
                f.write(f" /* SDL: {preview} */")
            f.write("\n")

        f.write("};\n\nCONTROLLERDB_ATTR\n")
        f.write("static const unsigned kUsbGamepadMapsCount = "
                "(unsigned)(sizeof(kUsbGamepadMaps)/sizeof(kUsbGamepadMaps[0]));\n")
        f.write(C_HEADER_POSTAMBLE.format(guard=HEADER_GUARD))

def main():
    ap=argparse.ArgumentParser(description="Generate USB-only C maps from SDL_GameControllerDB.")
    ap.add_argument("--url",default=DB_URL)
    ap.add_argument("-o","--out",default="usb_controller_maps.h")
    ap.add_argument("--strings",choices=["inline","split"],default="split")
    ap.add_argument("--include-mapping-raw",choices=["yes","no"],default="yes")
    ap.add_argument("--sort",choices=["none","name","vidpid"],default="none")
    ap.add_argument("--no-comments",action="store_true",help="Do not emit SDL mapping comments.")
    args=ap.parse_args()

    lines=read_from_url_or_file(args.url)
    entries=parse_db_lines_usb(lines)
    if not entries:
        print("No USB entries parsed.",file=sys.stderr); sys.exit(1)
    entries=sort_entries(entries,args.sort)
    write_c_header(entries,args.out,args.url,args.strings,
                   include_map=(args.include_mapping_raw=="yes"),
                   no_comments=args.no_comments)
    print(f"OK — generated {args.out} with {len(entries)} USB mappings.")

if __name__=="__main__":
    main()
