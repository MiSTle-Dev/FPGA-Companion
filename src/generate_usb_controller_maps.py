#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate USB-only C maps from SDL_GameControllerDB.

Targets bare-metal MCU projects (for example BL616 + CherryUSB) that need a
compile-time table of controller mappings keyed by USB VID/PID/bcdDevice.

Output schemas:
  * extended (default) - preserves digital triggers, half-axis modes,
    per-direction axis-backed D-pads, axis inversion and optional paddles.
  * legacy             - keeps the original C structure layout. Information
    that cannot be represented safely is dropped and reported.

The generator deliberately does not pretend to support every SDL binding.
Unsupported bindings are reported instead of being silently misrepresented.
"""

import argparse
import os
import re
import sys
import tempfile
from collections import Counter, defaultdict
from urllib.request import urlopen

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GENERATOR_VERSION = "2.1-fixed"
DB_URL = (
    "https://raw.githubusercontent.com/gabomdq/SDL_GameControllerDB/"
    "master/gamecontrollerdb.txt"
)
HEADER_GUARD = "USB_CONTROLLER_MAPS_H_"

DEFAULT_MAX_BUTTONS = 32
DEFAULT_MAX_AXES = 8
DEFAULT_MAX_HATS = 1

AXIS_MODE_FULL = 0
AXIS_MODE_POSITIVE = 1
AXIS_MODE_NEGATIVE = 2

# Lower number means higher preference in --platform auto mode.
PLATFORM_PRIORITY = {
    "linux": 0,
    "windows": 1,
    "mac os x": 2,
    "ios": 3,
    "android": 4,
}
PLATFORM_PRIORITY_UNKNOWN = 5

PLATFORM_ALIASES = {
    "linux": "linux",
    "windows": "windows",
    "mac os x": "mac os x",
    "macos": "mac os x",
    "osx": "mac os x",
    "ios": "ios",
    "android": "android",
}
CLI_PLATFORM_MAP = {
    "auto": None,
    "linux": "linux",
    "windows": "windows",
    "macos": "mac os x",
    "ios": "ios",
    "android": "android",
}

GUID_RE = re.compile(r"^[0-9a-fA-F]{32}$")
HAT_RE = re.compile(r"h(\d+)\.(\d+)")
BTN_VAL_RE = re.compile(r"b(\d+)")
AXIS_VAL_RE = re.compile(r"([+-]?)a(\d+)(~?)")

BTN_KEYS = {
    "a": "btn_a",
    "b": "btn_b",
    "x": "btn_x",
    "y": "btn_y",
    "back": "btn_back",
    "guide": "btn_guide",
    "start": "btn_start",
    "leftshoulder": "btn_leftshoulder",
    "rightshoulder": "btn_rightshoulder",
    "leftstick": "btn_leftstick",
    "rightstick": "btn_rightstick",
}

DPAD_KEYS = {
    "dpup": "up",
    "dpdown": "down",
    "dpleft": "left",
    "dpright": "right",
}

STICK_AXIS_KEYS = {
    "leftx": "axis_lx",
    "lefty": "axis_ly",
    "rightx": "axis_rx",
    "righty": "axis_ry",
}

TRIGGER_KEYS = {
    "lefttrigger": "lt",
    "righttrigger": "rt",
}

PADDLE_KEYS = {"paddle1", "paddle2", "paddle3", "paddle4"}
DPAD_DIRECTIONS = ("up", "down", "left", "right")

# These are intentionally ignored because the current target structure does
# not expose them or because they are SDL metadata rather than input bindings.
IGNORED_KEYS = {
    "platform",
    "crc",
    "type",
    "touchpad",
    "misc1",
    "misc2",
    "misc3",
    "misc4",
    "misc5",
}

# SDL output-side prefixes. They describe an input feeding one half of a
# virtual stick. If an entry has no explicit D-pad, these bindings can be used
# to synthesize one. The input binding itself is still parsed losslessly.
OUTPUT_PREFIX_TO_DIR = {
    "-lefty": ("left", "up"),
    "+lefty": ("left", "down"),
    "-leftx": ("left", "left"),
    "+leftx": ("left", "right"),
    "-righty": ("right", "up"),
    "+righty": ("right", "down"),
    "-rightx": ("right", "left"),
    "+rightx": ("right", "right"),
}

# ---------------------------------------------------------------------------
# C header text
# ---------------------------------------------------------------------------

C_HEADER_PREAMBLE = r"""/*
 * Auto-generated from SDL_GameControllerDB (USB-only).
 * Generator: {generator_version}
 * Source:    {src}
 *
 * Schema:    {schema}{paddles_note}
 * Sort:      {sort_mode}
 * Dedup:     by (VID, PID, version), then platform selection.
 * Platform:  {platform_mode}
 * Limits:    buttons={max_buttons}, axes={max_axes}, hats={max_hats}
 *
 * GUID -> VID/PID/Version is decoded as little-endian, matching
 * SDL_GetJoystickGUIDInfo. Only USB GUIDs are retained
 * (GUID bytes 0..1 == 0x0003).
 *
 * Sentinel values:
 *   -1 in an int8_t index field means "not mapped".
 *    0 in a hat mask means "not mapped".
 *
 * Axis mode values used by the extended schema:
 *   CONTROLLERDB_AXIS_FULL     = full input axis
 *   CONTROLLERDB_AXIS_POSITIVE = positive half (+aN)
 *   CONTROLLERDB_AXIS_NEGATIVE = negative half (-aN)
 *
 * For a binding with the SDL '~' suffix, apply inversion before interpreting
 * the axis mode.
 *
 * Notes for BL616 / Bouffalo SDK:
 *   All data are const and can be kept in flash. A matching linker snippet is:
 *
 *     .rodata.controllerdb :
 *     {{
 *       KEEP(*(.rodata.controllerdb))
 *       KEEP(*(.rodata.controllerdb.*))
 *     }} > FLASH
 *
 *     .rodata.controllerdb.str :
 *     {{
 *       KEEP(*(.rodata.controllerdb.str))
 *       KEEP(*(.rodata.controllerdb.str.*))
 *     }} > FLASH
 *
 * With --sort vidpid the array is ordered by (vid, pid, version, name).
 */

#ifndef {guard}
#define {guard}

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

#ifndef CONTROLLERDB_ATTR
#define CONTROLLERDB_ATTR __attribute__((section(".rodata.controllerdb"), used, aligned(4)))
#endif

#ifndef CONTROLLERDB_STR_ATTR
#define CONTROLLERDB_STR_ATTR __attribute__((section(".rodata.controllerdb.str"), used, aligned(1)))
#endif

#define CONTROLLERDB_STR(sym, literal) CONTROLLERDB_STR_ATTR static const char sym[] = literal
"""

C_HEADER_POSTAMBLE = r"""
#ifdef __cplusplus
}}
#endif

#endif /* {guard} */
"""

# ---------------------------------------------------------------------------
# Statistics and issue handling
# ---------------------------------------------------------------------------


class Stats:
    def __init__(self):
        self.parsed_usb = 0
        self.kept = 0
        self.duplicates_removed = 0
        self.platform_skipped = 0
        self.platform_fallbacks = 0
        self.platform_winners = Counter()
        self.synthesized_dpad = 0

        self.dropped_trigger_button = 0
        self.dropped_halfaxis_trigger = 0
        self.dropped_halfaxis_stick = 0
        self.dropped_dpad_axis = 0
        self.dropped_paddles = 0

        self.issue_counts = Counter()
        self.issue_examples = []

    def add_issue(self, code, message):
        self.issue_counts[code] += 1
        self.issue_examples.append((code, message))

    @property
    def issue_total(self):
        return sum(self.issue_counts.values())


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def hexstr_to_bytes_16(hexstr):
    if not GUID_RE.fullmatch(hexstr):
        raise ValueError("Invalid GUID: {!r}".format(hexstr))
    return bytes.fromhex(hexstr)


def is_usb_guid(guid_hex):
    # Bus type is a little-endian uint16 in bytes 0..1. USB == 0x0003.
    return guid_hex.lower().startswith("0300")


def extract_vid_pid_version(guid_hex):
    b = hexstr_to_bytes_16(guid_hex.lower())
    vid = b[4] | (b[5] << 8)
    pid = b[8] | (b[9] << 8)
    version = b[12] | (b[13] << 8)
    return vid, pid, version


def normalize_platform(value):
    normalized = " ".join(value.strip().lower().split())
    return PLATFORM_ALIASES.get(normalized, normalized)


def c_escape(value):
    return (
        value.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
    )


def add_entry_issue(issues, code, message):
    issues.append((code, message))


def parse_btn_val(value):
    match = BTN_VAL_RE.fullmatch(value)
    return int(match.group(1)) if match else None


def parse_hat_val(value):
    match = HAT_RE.fullmatch(value)
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))


def parse_axis_val(value):
    """Parse aN, aN~, +aN, -aN and combinations.

    Returns (index, mode, invert), where mode is one of AXIS_MODE_*.
    """
    match = AXIS_VAL_RE.fullmatch(value)
    if not match:
        return None

    sign, index_text, invert_text = match.groups()
    if sign == "+":
        mode = AXIS_MODE_POSITIVE
    elif sign == "-":
        mode = AXIS_MODE_NEGATIVE
    else:
        mode = AXIS_MODE_FULL

    return int(index_text), mode, 1 if invert_text == "~" else 0


def effective_axis_mode(mode, invert):
    """Return the half-axis mode after applying SDL inversion."""
    if not invert:
        return mode
    if mode == AXIS_MODE_POSITIVE:
        return AXIS_MODE_NEGATIVE
    if mode == AXIS_MODE_NEGATIVE:
        return AXIS_MODE_POSITIVE
    return mode


def empty_record():
    record = {
        # Face / shoulder / system / stick-click buttons.
        "btn_a": -1,
        "btn_b": -1,
        "btn_x": -1,
        "btn_y": -1,
        "btn_back": -1,
        "btn_guide": -1,
        "btn_start": -1,
        "btn_leftshoulder": -1,
        "btn_rightshoulder": -1,
        "btn_leftstick": -1,
        "btn_rightstick": -1,

        # D-pad as four buttons.
        "btn_dpad_up": -1,
        "btn_dpad_down": -1,
        "btn_dpad_left": -1,
        "btn_dpad_right": -1,

        # D-pad as one SDL hat.
        "dpad_hat": -1,
        "dpad_hat_up": 0,
        "dpad_hat_right": 0,
        "dpad_hat_down": 0,
        "dpad_hat_left": 0,

        # Sticks.
        "axis_lx": -1,
        "axis_ly": -1,
        "axis_rx": -1,
        "axis_ry": -1,
        "axis_lx_invert": 0,
        "axis_ly_invert": 0,
        "axis_rx_invert": 0,
        "axis_ry_invert": 0,
        "axis_lx_mode": AXIS_MODE_FULL,
        "axis_ly_mode": AXIS_MODE_FULL,
        "axis_rx_mode": AXIS_MODE_FULL,
        "axis_ry_mode": AXIS_MODE_FULL,

        # Triggers.
        "axis_lt": -1,
        "axis_rt": -1,
        "axis_lt_invert": 0,
        "axis_rt_invert": 0,
        "axis_lt_mode": AXIS_MODE_FULL,
        "axis_rt_mode": AXIS_MODE_FULL,
        "btn_lt": -1,
        "btn_rt": -1,

        # Optional paddles.
        "btn_paddle1": -1,
        "btn_paddle2": -1,
        "btn_paddle3": -1,
        "btn_paddle4": -1,
    }

    # Axis-backed D-pad. Each direction is preserved independently so the
    # generator never loses axis index, half-axis polarity or inversion.
    for direction in DPAD_DIRECTIONS:
        record["dpad_axis_{}".format(direction)] = -1
        record["dpad_axis_{}_mode".format(direction)] = AXIS_MODE_FULL
        record["dpad_axis_{}_invert".format(direction)] = 0

    return record


def has_any_dpad(record):
    if any(record["btn_dpad_{}".format(d)] != -1 for d in DPAD_DIRECTIONS):
        return True
    if record["dpad_hat"] != -1:
        return True
    return any(record["dpad_axis_{}".format(d)] != -1 for d in DPAD_DIRECTIONS)


def set_dpad_axis_binding(record, direction, axis_value):
    index, mode, invert = axis_value
    record["dpad_axis_{}".format(direction)] = index
    record["dpad_axis_{}_mode".format(direction)] = mode
    record["dpad_axis_{}_invert".format(direction)] = invert


# ---------------------------------------------------------------------------
# SDL mapping parsing
# ---------------------------------------------------------------------------


def parse_mapping_to_record(mapping_str, context):
    """Parse one SDL mapping body.

    Returns (record, platform, synthesis_pool, issues). Issues are attached to
    the entry and only reported if that entry survives platform selection and
    deduplication.
    """
    record = empty_record()
    platform = ""
    synthesis_pool = {"left": {}, "right": {}}
    issues = []

    for part in (piece.strip() for piece in mapping_str.split(",")):
        if not part:
            continue
        if ":" not in part:
            add_entry_issue(
                issues,
                "malformed-binding",
                "{}: binding without ':' ignored: {!r}".format(context, part),
            )
            continue

        key, value = part.split(":", 1)
        key = key.strip().lower()
        value = value.strip().lower()

        if key == "platform":
            platform = normalize_platform(value)
            continue
        if key in IGNORED_KEYS:
            continue

        if key in OUTPUT_PREFIX_TO_DIR:
            side, direction = OUTPUT_PREFIX_TO_DIR[key]
            synthesis_pool[side][direction] = value
            continue

        if key in BTN_KEYS:
            button = parse_btn_val(value)
            if button is not None:
                record[BTN_KEYS[key]] = button
            else:
                add_entry_issue(
                    issues,
                    "unsupported-button-binding",
                    "{}: {}:{} is not a button binding; dropped".format(
                        context, key, value
                    ),
                )
            continue

        if key in DPAD_KEYS:
            direction = DPAD_KEYS[key]

            button = parse_btn_val(value)
            if button is not None:
                record["btn_dpad_{}".format(direction)] = button
                continue

            hat_value = parse_hat_val(value)
            if hat_value is not None:
                hat_index, mask = hat_value
                if record["dpad_hat"] == -1:
                    record["dpad_hat"] = hat_index
                if record["dpad_hat"] != hat_index:
                    add_entry_issue(
                        issues,
                        "multiple-dpad-hats",
                        (
                            "{}: D-pad directions use hats {} and {}; "
                            "{}:{} dropped"
                        ).format(
                            context,
                            record["dpad_hat"],
                            hat_index,
                            key,
                            value,
                        ),
                    )
                else:
                    record["dpad_hat_{}".format(direction)] = mask
                continue

            axis_value = parse_axis_val(value)
            if axis_value is not None:
                set_dpad_axis_binding(record, direction, axis_value)
                continue

            add_entry_issue(
                issues,
                "unsupported-dpad-binding",
                "{}: {}:{} cannot be represented; dropped".format(
                    context, key, value
                ),
            )
            continue

        if key in STICK_AXIS_KEYS:
            axis_value = parse_axis_val(value)
            if axis_value is None:
                add_entry_issue(
                    issues,
                    "unsupported-stick-binding",
                    "{}: {}:{} is not an axis binding; dropped".format(
                        context, key, value
                    ),
                )
                continue

            index, mode, invert = axis_value
            field = STICK_AXIS_KEYS[key]
            record[field] = index
            record[field + "_mode"] = mode
            record[field + "_invert"] = invert
            continue

        if key in TRIGGER_KEYS:
            side = TRIGGER_KEYS[key]

            button = parse_btn_val(value)
            if button is not None:
                record["btn_{}".format(side)] = button
                continue

            axis_value = parse_axis_val(value)
            if axis_value is not None:
                index, mode, invert = axis_value
                record["axis_{}".format(side)] = index
                record["axis_{}_mode".format(side)] = mode
                record["axis_{}_invert".format(side)] = invert
                continue

            add_entry_issue(
                issues,
                "unsupported-trigger-binding",
                "{}: {}:{} cannot be represented; dropped".format(
                    context, key, value
                ),
            )
            continue

        if key in PADDLE_KEYS:
            button = parse_btn_val(value)
            if button is not None:
                record["btn_{}".format(key)] = button
            else:
                add_entry_issue(
                    issues,
                    "unsupported-paddle-binding",
                    "{}: {}:{} is not a button binding; dropped".format(
                        context, key, value
                    ),
                )
            continue

        add_entry_issue(
            issues,
            "unknown-mapping-key",
            "{}: unknown mapping key {}:{} ignored".format(context, key, value),
        )

    return record, platform, synthesis_pool, issues


def apply_synth_binding(record, direction, value, issues, context):
    button = parse_btn_val(value)
    if button is not None:
        record["btn_dpad_{}".format(direction)] = button
        return True

    hat_value = parse_hat_val(value)
    if hat_value is not None:
        hat_index, mask = hat_value
        if record["dpad_hat"] == -1:
            record["dpad_hat"] = hat_index
        if record["dpad_hat"] != hat_index:
            add_entry_issue(
                issues,
                "multiple-synth-dpad-hats",
                "{}: synthesized D-pad uses more than one hat; {}:{} dropped".format(
                    context, direction, value
                ),
            )
            return False
        record["dpad_hat_{}".format(direction)] = mask
        return True

    axis_value = parse_axis_val(value)
    if axis_value is not None:
        set_dpad_axis_binding(record, direction, axis_value)
        return True

    add_entry_issue(
        issues,
        "unsupported-synth-dpad-binding",
        "{}: cannot synthesize D-pad direction {} from {!r}".format(
            context, direction, value
        ),
    )
    return False


def maybe_synthesize_dpad(record, synthesis_pool, issues, context):
    if has_any_dpad(record):
        return False

    source = synthesis_pool.get("left") or synthesis_pool.get("right")
    if not source:
        return False

    applied = False
    for direction, value in source.items():
        if apply_synth_binding(record, direction, value, issues, context):
            applied = True
    return applied


def parse_db_lines_usb(lines, stats, synthesize_dpad):
    entries = []

    for line_number, raw in enumerate(lines, start=1):
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue

        # Strip trailing comments only when // is preceded by whitespace.
        line = re.split(r"\s//", line, maxsplit=1)[0].strip()
        head = line.split(",", 2)
        if len(head) < 3:
            stats.add_issue(
                "malformed-db-line",
                "line {}: expected GUID,name,mapping".format(line_number),
            )
            continue

        guid = head[0].strip()
        name = head[1].strip()
        mapping = head[2].strip().rstrip(",").strip()

        if not GUID_RE.fullmatch(guid):
            stats.add_issue(
                "invalid-guid",
                "line {}: invalid GUID {!r}".format(line_number, guid),
            )
            continue
        if not is_usb_guid(guid):
            continue

        try:
            vid, pid, version = extract_vid_pid_version(guid)
        except ValueError as exc:
            stats.add_issue("invalid-guid", "line {}: {}".format(line_number, exc))
            continue

        context = "line {} {} (VID={:04X} PID={:04X} VER={:04X})".format(
            line_number, name, vid, pid, version
        )
        record, platform, synthesis_pool, issues = parse_mapping_to_record(
            mapping, context
        )
        synthesized = False
        if synthesize_dpad:
            synthesized = maybe_synthesize_dpad(
                record, synthesis_pool, issues, context
            )

        entries.append(
            {
                "guid": guid,
                "name": name,
                "vid": vid,
                "pid": pid,
                "version": version,
                "platform": platform,
                "record": record,
                "issues": issues,
                "synthesized_dpad": synthesized,
                "line_number": line_number,
                "order": len(entries),
            }
        )
        stats.parsed_usb += 1

    return entries


# ---------------------------------------------------------------------------
# Platform selection, deduplication and validation
# ---------------------------------------------------------------------------


BUTTON_INDEX_FIELDS = [
    "btn_a",
    "btn_b",
    "btn_x",
    "btn_y",
    "btn_back",
    "btn_guide",
    "btn_start",
    "btn_leftshoulder",
    "btn_rightshoulder",
    "btn_leftstick",
    "btn_rightstick",
    "btn_dpad_up",
    "btn_dpad_down",
    "btn_dpad_left",
    "btn_dpad_right",
    "btn_lt",
    "btn_rt",
    "btn_paddle1",
    "btn_paddle2",
    "btn_paddle3",
    "btn_paddle4",
]

STICK_AXIS_FIELDS = ["axis_lx", "axis_ly", "axis_rx", "axis_ry"]
TRIGGER_AXIS_FIELDS = ["axis_lt", "axis_rt"]
DPAD_AXIS_FIELDS = ["dpad_axis_{}".format(d) for d in DPAD_DIRECTIONS]
ALL_AXIS_INDEX_FIELDS = STICK_AXIS_FIELDS + TRIGGER_AXIS_FIELDS + DPAD_AXIS_FIELDS


def raw_range_penalty(entry, max_buttons, max_axes, max_hats):
    record = entry["record"]
    penalty = 0
    for field in BUTTON_INDEX_FIELDS:
        value = record[field]
        if value < -1 or value >= max_buttons:
            penalty += 1
    for field in ALL_AXIS_INDEX_FIELDS:
        value = record[field]
        if value < -1 or value >= max_axes:
            penalty += 1
    if record["dpad_hat"] < -1 or record["dpad_hat"] >= max_hats:
        penalty += 1
    return penalty


def candidate_quality_key(entry, max_buttons, max_axes, max_hats):
    return (
        len(entry["issues"]) + raw_range_penalty(
            entry, max_buttons, max_axes, max_hats
        ),
        entry["order"],
    )


def platform_priority(entry):
    return PLATFORM_PRIORITY.get(entry["platform"], PLATFORM_PRIORITY_UNKNOWN)


def select_and_deduplicate(
    entries,
    stats,
    preferred_platform,
    strict_platform,
    max_buttons,
    max_axes,
    max_hats,
):
    """Select one entry per (VID, PID, version).

    A requested platform is selected when present. Without --strict-platform,
    missing platform variants fall back to the normal priority order.
    Within the same platform, the mapping with fewer parse/range issues wins;
    source order breaks the remaining tie.
    """
    groups = defaultdict(list)
    for entry in entries:
        groups[(entry["vid"], entry["pid"], entry["version"])].append(entry)

    selected = []
    for key in sorted(groups):
        group = groups[key]
        pool = group

        if preferred_platform is not None:
            exact = [e for e in group if e["platform"] == preferred_platform]
            if exact:
                pool = exact
            elif strict_platform:
                stats.platform_skipped += 1
                continue
            else:
                stats.platform_fallbacks += 1

        if preferred_platform is not None and pool is not group:
            winner = min(
                pool,
                key=lambda e: candidate_quality_key(
                    e, max_buttons, max_axes, max_hats
                ),
            )
        else:
            winner = min(
                pool,
                key=lambda e: (
                    platform_priority(e),
                    candidate_quality_key(e, max_buttons, max_axes, max_hats),
                ),
            )

        selected.append(winner)
        stats.platform_winners[winner["platform"] or "(unknown)"] += 1

    stats.kept = len(selected)
    # This count includes discarded platform variants and entries removed by
    # strict platform selection. platform_skipped separately reports groups.
    stats.duplicates_removed = stats.parsed_usb - stats.kept
    return selected


def reset_axis_binding(record, field):
    record[field] = -1
    record[field + "_mode"] = AXIS_MODE_FULL
    record[field + "_invert"] = 0


def validate_index(value, limit):
    return value == -1 or 0 <= value < limit


def validate_and_sanitize_entry(entry, max_buttons, max_axes, max_hats):
    record = entry["record"]
    context = "{} (VID={:04X} PID={:04X} VER={:04X})".format(
        entry["name"], entry["vid"], entry["pid"], entry["version"]
    )

    for field in BUTTON_INDEX_FIELDS:
        value = record[field]
        if not validate_index(value, max_buttons):
            add_entry_issue(
                entry["issues"],
                "button-index-out-of-range",
                "{}: {}={} outside -1..{}; replaced with -1".format(
                    context, field, value, max_buttons - 1
                ),
            )
            record[field] = -1

    for field in STICK_AXIS_FIELDS + TRIGGER_AXIS_FIELDS:
        value = record[field]
        if not validate_index(value, max_axes):
            add_entry_issue(
                entry["issues"],
                "axis-index-out-of-range",
                "{}: {}={} outside -1..{}; binding removed".format(
                    context, field, value, max_axes - 1
                ),
            )
            reset_axis_binding(record, field)

    for direction in DPAD_DIRECTIONS:
        field = "dpad_axis_{}".format(direction)
        value = record[field]
        if not validate_index(value, max_axes):
            add_entry_issue(
                entry["issues"],
                "axis-index-out-of-range",
                "{}: {}={} outside -1..{}; binding removed".format(
                    context, field, value, max_axes - 1
                ),
            )
            reset_axis_binding(record, field)

    hat = record["dpad_hat"]
    if not validate_index(hat, max_hats):
        add_entry_issue(
            entry["issues"],
            "hat-index-out-of-range",
            "{}: dpad_hat={} outside -1..{}; hat binding removed".format(
                context, hat, max_hats - 1
            ),
        )
        record["dpad_hat"] = -1
        for direction in DPAD_DIRECTIONS:
            record["dpad_hat_{}".format(direction)] = 0

    for direction in DPAD_DIRECTIONS:
        field = "dpad_hat_{}".format(direction)
        mask = record[field]
        if not 0 <= mask <= 0xFF:
            add_entry_issue(
                entry["issues"],
                "hat-mask-out-of-range",
                "{}: {}={} outside 0..255; replaced with 0".format(
                    context, field, mask
                ),
            )
            record[field] = 0

    if record["dpad_hat"] == -1:
        for direction in DPAD_DIRECTIONS:
            record["dpad_hat_{}".format(direction)] = 0

    validate_dpad_axis_semantics(entry, context)


def validate_dpad_axis_semantics(entry, context):
    record = entry["record"]
    expected_modes = {
        "up": AXIS_MODE_NEGATIVE,
        "down": AXIS_MODE_POSITIVE,
        "left": AXIS_MODE_NEGATIVE,
        "right": AXIS_MODE_POSITIVE,
    }

    for direction in DPAD_DIRECTIONS:
        axis_field = "dpad_axis_{}".format(direction)
        if record[axis_field] == -1:
            continue

        mode = record[axis_field + "_mode"]
        invert = record[axis_field + "_invert"]
        effective = effective_axis_mode(mode, invert)

        if mode == AXIS_MODE_FULL:
            add_entry_issue(
                entry["issues"],
                "dpad-full-axis-binding",
                (
                    "{}: {} uses a full axis; the consumer must define a "
                    "directional threshold explicitly"
                ).format(context, direction),
            )
        elif effective != expected_modes[direction]:
            add_entry_issue(
                entry["issues"],
                "dpad-axis-polarity",
                (
                    "{}: {} axis binding has effective mode {}, expected {}. "
                    "Binding is preserved exactly."
                ).format(
                    context, direction, effective, expected_modes[direction]
                ),
            )

    for first, second, label in (
        ("left", "right", "horizontal"),
        ("up", "down", "vertical"),
    ):
        first_axis = record["dpad_axis_{}".format(first)]
        second_axis = record["dpad_axis_{}".format(second)]
        if first_axis != -1 and second_axis != -1 and first_axis != second_axis:
            add_entry_issue(
                entry["issues"],
                "dpad-axis-pair-mismatch",
                (
                    "{}: {} D-pad pair uses axes {} and {}; bindings are "
                    "preserved per direction"
                ).format(context, label, first_axis, second_axis),
            )


def finalize_selected_issues(entries, stats):
    for entry in entries:
        if entry["synthesized_dpad"]:
            stats.synthesized_dpad += 1
        for code, message in entry["issues"]:
            stats.add_issue(code, message)


# ---------------------------------------------------------------------------
# Schema filters and sorting
# ---------------------------------------------------------------------------


def count_legacy_drops(entries, stats):
    for entry in entries:
        record = entry["record"]

        for side in ("lt", "rt"):
            if record["btn_{}".format(side)] != -1:
                stats.dropped_trigger_button += 1
            if (
                record["axis_{}".format(side)] != -1
                and record["axis_{}_mode".format(side)] != AXIS_MODE_FULL
            ):
                stats.dropped_halfaxis_trigger += 1

        for field in STICK_AXIS_FIELDS:
            if record[field] != -1 and record[field + "_mode"] != AXIS_MODE_FULL:
                stats.dropped_halfaxis_stick += 1

        if any(
            record["dpad_axis_{}".format(direction)] != -1
            for direction in DPAD_DIRECTIONS
        ):
            stats.dropped_dpad_axis += 1

        stats.dropped_paddles += sum(
            record["btn_paddle{}".format(index)] != -1
            for index in (1, 2, 3, 4)
        )


def apply_legacy_filter(entries):
    for entry in entries:
        record = entry["record"]

        record["btn_lt"] = -1
        record["btn_rt"] = -1

        for field in STICK_AXIS_FIELDS + TRIGGER_AXIS_FIELDS:
            if record[field + "_mode"] != AXIS_MODE_FULL:
                reset_axis_binding(record, field)

        for direction in DPAD_DIRECTIONS:
            reset_axis_binding(record, "dpad_axis_{}".format(direction))

        for index in (1, 2, 3, 4):
            record["btn_paddle{}".format(index)] = -1


def apply_extended_filter(entries, include_paddles):
    for entry in entries:
        record = entry["record"]

        # If a malformed/duplicated SDL line supplied both forms, retain the
        # digital trigger and remove the axis form deterministically.
        for side in ("lt", "rt"):
            if (
                record["btn_{}".format(side)] != -1
                and record["axis_{}".format(side)] != -1
            ):
                reset_axis_binding(record, "axis_{}".format(side))

        if not include_paddles:
            for index in (1, 2, 3, 4):
                record["btn_paddle{}".format(index)] = -1


def sort_entries(entries, mode):
    if mode == "name":
        return sorted(
            entries,
            key=lambda e: (
                e["name"].lower(),
                e["vid"],
                e["pid"],
                e["version"],
            ),
        )
    if mode == "vidpid":
        return sorted(
            entries,
            key=lambda e: (
                e["vid"],
                e["pid"],
                e["version"],
                e["name"].lower(),
            ),
        )
    return entries


# ---------------------------------------------------------------------------
# C output
# ---------------------------------------------------------------------------


LEGACY_BUTTON_FIELDS = [
    "btn_a",
    "btn_b",
    "btn_x",
    "btn_y",
    "btn_back",
    "btn_guide",
    "btn_start",
    "btn_leftshoulder",
    "btn_rightshoulder",
    "btn_leftstick",
    "btn_rightstick",
    "btn_dpad_up",
    "btn_dpad_down",
    "btn_dpad_left",
    "btn_dpad_right",
]
LEGACY_HAT_FIELDS = [
    "dpad_hat",
    "dpad_hat_up",
    "dpad_hat_right",
    "dpad_hat_down",
    "dpad_hat_left",
]
LEGACY_STICK_FIELDS = ["axis_lx", "axis_ly", "axis_rx", "axis_ry"]
LEGACY_STICK_INVERT_FIELDS = [
    "axis_lx_invert",
    "axis_ly_invert",
    "axis_rx_invert",
    "axis_ry_invert",
]
LEGACY_TRIGGER_FIELDS = ["axis_lt", "axis_rt"]
LEGACY_TRIGGER_INVERT_FIELDS = ["axis_lt_invert", "axis_rt_invert"]

EXT_TRIGGER_BUTTON_FIELDS = ["btn_lt", "btn_rt"]
EXT_STICK_MODE_FIELDS = [
    "axis_lx_mode",
    "axis_ly_mode",
    "axis_rx_mode",
    "axis_ry_mode",
]
EXT_TRIGGER_MODE_FIELDS = ["axis_lt_mode", "axis_rt_mode"]
EXT_DPAD_AXIS_FIELDS = ["dpad_axis_{}".format(d) for d in DPAD_DIRECTIONS]
EXT_DPAD_AXIS_MODE_FIELDS = [
    "dpad_axis_{}_mode".format(d) for d in DPAD_DIRECTIONS
]
EXT_DPAD_AXIS_INVERT_FIELDS = [
    "dpad_axis_{}_invert".format(d) for d in DPAD_DIRECTIONS
]
PADDLE_FIELDS = ["btn_paddle1", "btn_paddle2", "btn_paddle3", "btn_paddle4"]


def emit_typedef(output, schema, include_paddles):
    if schema == "extended":
        output.write("\n#define CONTROLLERDB_AXIS_FULL     0u\n")
        output.write("#define CONTROLLERDB_AXIS_POSITIVE 1u\n")
        output.write("#define CONTROLLERDB_AXIS_NEGATIVE 2u\n")

    output.write("\ntypedef struct {\n")
    output.write("    const char *guid;\n")
    output.write("    const char *name;\n")
    output.write("    uint16_t vid, pid, version;\n\n")

    output.write("    int8_t  btn_a, btn_b, btn_x, btn_y;\n")
    output.write("    int8_t  btn_back, btn_guide, btn_start;\n")
    output.write("    int8_t  btn_leftshoulder, btn_rightshoulder;\n")
    output.write("    int8_t  btn_leftstick, btn_rightstick;\n\n")

    output.write(
        "    int8_t  btn_dpad_up, btn_dpad_down, "
        "btn_dpad_left, btn_dpad_right;\n\n"
    )

    output.write("    int8_t  dpad_hat;\n")
    output.write(
        "    uint8_t dpad_hat_up, dpad_hat_right, "
        "dpad_hat_down, dpad_hat_left;\n\n"
    )

    output.write("    int8_t  axis_lx, axis_ly, axis_rx, axis_ry;\n")
    output.write(
        "    uint8_t axis_lx_invert, axis_ly_invert, "
        "axis_rx_invert, axis_ry_invert;\n\n"
    )

    output.write("    int8_t  axis_lt, axis_rt;\n")
    output.write("    uint8_t axis_lt_invert, axis_rt_invert;\n")

    if schema == "extended":
        output.write("\n    /* Extended schema additions. */\n")
        output.write("    int8_t  btn_lt, btn_rt;\n")
        output.write(
            "    uint8_t axis_lx_mode, axis_ly_mode, "
            "axis_rx_mode, axis_ry_mode;\n"
        )
        output.write("    uint8_t axis_lt_mode, axis_rt_mode;\n\n")

        output.write(
            "    int8_t  dpad_axis_up, dpad_axis_down, "
            "dpad_axis_left, dpad_axis_right;\n"
        )
        output.write(
            "    uint8_t dpad_axis_up_mode, dpad_axis_down_mode, "
            "dpad_axis_left_mode, dpad_axis_right_mode;\n"
        )
        output.write(
            "    uint8_t dpad_axis_up_invert, dpad_axis_down_invert, "
            "dpad_axis_left_invert, dpad_axis_right_invert;\n"
        )

        if include_paddles:
            output.write("\n")
            output.write(
                "    int8_t  btn_paddle1, btn_paddle2, "
                "btn_paddle3, btn_paddle4;\n"
            )

    output.write("} UsbGamepadMap;\n\n")


def emit_values(output, record, fields, prefix=""):
    output.write(prefix)
    output.write(", ".join(str(int(record[field])) for field in fields))


def platform_mode_description(preferred_platform, strict_platform):
    if preferred_platform is None:
        return "auto: Linux > Windows > Mac OS X > iOS > Android > unknown"
    if strict_platform:
        return "{} only (strict)".format(preferred_platform)
    return "{} preferred, automatic fallback".format(preferred_platform)


def write_c_header(
    entries,
    out_path,
    source,
    strings_mode,
    no_comments,
    schema,
    include_paddles,
    sort_mode,
    preferred_platform,
    strict_platform,
    max_buttons,
    max_axes,
    max_hats,
):
    paddles_note = " (+paddles)" if include_paddles else ""
    output_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(output_dir, exist_ok=True)

    fd, temporary_path = tempfile.mkstemp(
        prefix=".usb_controller_maps.", suffix=".tmp", dir=output_dir
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as output:
            output.write(
                C_HEADER_PREAMBLE.format(
                    generator_version=GENERATOR_VERSION,
                    src=source,
                    schema=schema,
                    paddles_note=paddles_note,
                    sort_mode=sort_mode,
                    platform_mode=platform_mode_description(
                        preferred_platform, strict_platform
                    ),
                    max_buttons=max_buttons,
                    max_axes=max_axes,
                    max_hats=max_hats,
                    guard=HEADER_GUARD,
                )
            )
            emit_typedef(output, schema, include_paddles)

            if strings_mode == "split":
                for index, entry in enumerate(entries):
                    output.write(
                        'CONTROLLERDB_STR(g_guid_{}, "{}");\n'.format(
                            index, c_escape(entry["guid"])
                        )
                    )
                    output.write(
                        'CONTROLLERDB_STR(g_name_{}, "{}");\n'.format(
                            index, c_escape(entry["name"])
                        )
                    )
                output.write("\n")

            output.write("CONTROLLERDB_ATTR\n")
            output.write("static const UsbGamepadMap kUsbGamepadMaps[] = {\n")

            for index, entry in enumerate(entries):
                record = entry["record"]
                if strings_mode == "split":
                    guid_expression = "g_guid_{}".format(index)
                    name_expression = "g_name_{}".format(index)
                else:
                    guid_expression = '"{}"'.format(c_escape(entry["guid"]))
                    name_expression = '"{}"'.format(c_escape(entry["name"]))

                output.write("  { ")
                output.write(
                    "{}, {}, 0x{:04X}, 0x{:04X}, 0x{:04X}, ".format(
                        guid_expression,
                        name_expression,
                        entry["vid"],
                        entry["pid"],
                        entry["version"],
                    )
                )
                emit_values(output, record, LEGACY_BUTTON_FIELDS)
                output.write(", ")
                emit_values(output, record, LEGACY_HAT_FIELDS)
                output.write(", ")
                emit_values(output, record, LEGACY_STICK_FIELDS)
                output.write(", ")
                emit_values(output, record, LEGACY_STICK_INVERT_FIELDS)
                output.write(", ")
                emit_values(output, record, LEGACY_TRIGGER_FIELDS)
                output.write(", ")
                emit_values(output, record, LEGACY_TRIGGER_INVERT_FIELDS)

                if schema == "extended":
                    output.write(", ")
                    emit_values(output, record, EXT_TRIGGER_BUTTON_FIELDS)
                    output.write(", ")
                    emit_values(output, record, EXT_STICK_MODE_FIELDS)
                    output.write(", ")
                    emit_values(output, record, EXT_TRIGGER_MODE_FIELDS)
                    output.write(", ")
                    emit_values(output, record, EXT_DPAD_AXIS_FIELDS)
                    output.write(", ")
                    emit_values(output, record, EXT_DPAD_AXIS_MODE_FIELDS)
                    output.write(", ")
                    emit_values(output, record, EXT_DPAD_AXIS_INVERT_FIELDS)
                    if include_paddles:
                        output.write(", ")
                        emit_values(output, record, PADDLE_FIELDS)

                output.write(" },")
                if not no_comments:
                    platform = entry["platform"] or "unknown"
                    output.write(
                        " /* {} [{}] */".format(
                            c_escape(entry["name"]), c_escape(platform)
                        )
                    )
                output.write("\n")

            output.write("};\n\nCONTROLLERDB_ATTR\n")
            output.write(
                "static const unsigned kUsbGamepadMapsCount = "
                "(unsigned)(sizeof(kUsbGamepadMaps) / "
                "sizeof(kUsbGamepadMaps[0]));\n"
            )
            output.write(C_HEADER_POSTAMBLE.format(guard=HEADER_GUARD))

        os.replace(temporary_path, out_path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise


# ---------------------------------------------------------------------------
# I/O and reporting
# ---------------------------------------------------------------------------


def read_from_url_or_file(source):
    if re.match(r"^https?://", source):
        with urlopen(source, timeout=30) as response:
            return response.read().decode("utf-8", errors="replace").splitlines()
    with open(source, "r", encoding="utf-8") as input_file:
        return input_file.read().splitlines()


def print_stats(stats, schema, warning_limit, quiet_warnings):
    print("Parsed USB entries:             {}".format(stats.parsed_usb), file=sys.stderr)
    print("Kept VID/PID/version entries:   {}".format(stats.kept), file=sys.stderr)
    print("Duplicate/platform variants:    {} removed".format(stats.duplicates_removed), file=sys.stderr)

    if stats.platform_skipped:
        print("Strict-platform groups skipped: {}".format(stats.platform_skipped), file=sys.stderr)
    if stats.platform_fallbacks:
        print("Platform fallbacks:             {}".format(stats.platform_fallbacks), file=sys.stderr)
    if stats.synthesized_dpad:
        print("Synthesized D-pads:             {}".format(stats.synthesized_dpad), file=sys.stderr)

    print("Selected platform variants:", file=sys.stderr)
    ordered_labels = ["linux", "windows", "mac os x", "ios", "android", "(unknown)"]
    displayed = set()
    for label in ordered_labels:
        count = stats.platform_winners.get(label, 0)
        if count:
            print("  {:12s} {}".format(label, count), file=sys.stderr)
            displayed.add(label)
    for label in sorted(set(stats.platform_winners) - displayed):
        print("  {:12s} {}".format(label, stats.platform_winners[label]), file=sys.stderr)

    if schema == "legacy":
        print("Dropped under legacy schema:", file=sys.stderr)
        print("  trigger-as-button:  {}".format(stats.dropped_trigger_button), file=sys.stderr)
        print("  half-axis trigger:  {}".format(stats.dropped_halfaxis_trigger), file=sys.stderr)
        print("  half-axis stick:    {}".format(stats.dropped_halfaxis_stick), file=sys.stderr)
        print("  dpad-as-axis:       {}".format(stats.dropped_dpad_axis), file=sys.stderr)
        print("  paddles:            {}".format(stats.dropped_paddles), file=sys.stderr)

    print("Warnings/issues:               {}".format(stats.issue_total), file=sys.stderr)
    for code, count in sorted(stats.issue_counts.items()):
        print("  {:31s} {}".format(code, count), file=sys.stderr)

    if stats.issue_total and not quiet_warnings and warning_limit > 0:
        print("Warning examples (max {}):".format(warning_limit), file=sys.stderr)
        for code, message in stats.issue_examples[:warning_limit]:
            print("  [{}] {}".format(code, message), file=sys.stderr)
        if len(stats.issue_examples) > warning_limit:
            print(
                "  ... {} more warning(s) omitted".format(
                    len(stats.issue_examples) - warning_limit
                ),
                file=sys.stderr,
            )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def nonnegative_int(value):
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def main():
    parser = argparse.ArgumentParser(
        description="Generate USB-only C maps from SDL_GameControllerDB."
    )
    parser.add_argument(
        "--url",
        default=DB_URL,
        help="Database URL or local file path (default: upstream database)",
    )
    parser.add_argument(
        "-o",
        "--out",
        default="usb_controller_maps.h",
        help="Output header path",
    )
    parser.add_argument(
        "--schema",
        choices=["legacy", "extended"],
        default="extended",
        help=(
            "Output structure schema (default: extended). Legacy keeps the "
            "old layout and drops unrepresentable bindings."
        ),
    )
    parser.add_argument(
        "--platform",
        choices=sorted(CLI_PLATFORM_MAP),
        default="auto",
        help=(
            "Preferred SDL platform variant (default: auto). A named platform "
            "falls back automatically unless --strict-platform is used."
        ),
    )
    parser.add_argument(
        "--strict-platform",
        action="store_true",
        help="Drop VID/PID/version groups that lack the requested platform",
    )
    parser.add_argument(
        "--paddles",
        action="store_true",
        help="Include paddle1..4 button fields; implies --schema extended",
    )
    parser.add_argument(
        "--strings",
        choices=["inline", "split"],
        default="split",
        help="String emission mode (default: split flash string section)",
    )
    parser.add_argument(
        "--sort",
        choices=["none", "name", "vidpid"],
        default="vidpid",
        help="Entry order (default: vidpid, including version)",
    )
    parser.add_argument(
        "--max-buttons",
        type=positive_int,
        default=DEFAULT_MAX_BUTTONS,
        help="Maximum button count supported by the consumer (default: 32)",
    )
    parser.add_argument(
        "--max-axes",
        type=positive_int,
        default=DEFAULT_MAX_AXES,
        help="Maximum axis count supported by the consumer (default: 8)",
    )
    parser.add_argument(
        "--max-hats",
        type=positive_int,
        default=DEFAULT_MAX_HATS,
        help="Maximum hat count supported by the consumer (default: 1)",
    )
    parser.add_argument(
        "--no-comments",
        action="store_true",
        help="Do not emit per-entry trailing comments",
    )
    parser.add_argument(
        "--no-synthesize-dpad",
        action="store_true",
        help=(
            "Disable D-pad synthesis from SDL output-side prefixes such as "
            "+leftx/-leftx/+lefty/-lefty"
        ),
    )
    parser.add_argument(
        "--warning-limit",
        type=nonnegative_int,
        default=40,
        help="Maximum detailed warning examples printed (default: 40)",
    )
    parser.add_argument(
        "--quiet-warnings",
        action="store_true",
        help="Print warning counts but no detailed examples",
    )
    parser.add_argument(
        "--fail-on-warning",
        action="store_true",
        help="Return exit status 2 when any selected mapping has warnings",
    )
    args = parser.parse_args()

    if args.max_buttons > 128 or args.max_axes > 128 or args.max_hats > 128:
        parser.error("max index counts must fit in int8_t fields (maximum 128)")

    if args.strict_platform and args.platform == "auto":
        parser.error("--strict-platform requires a named --platform")

    if args.paddles and args.schema == "legacy":
        print(
            "note: --paddles implies --schema extended; upgrading.",
            file=sys.stderr,
        )
        args.schema = "extended"

    preferred_platform = CLI_PLATFORM_MAP[args.platform]
    stats = Stats()

    try:
        lines = read_from_url_or_file(args.url)
    except (OSError, ValueError) as exc:
        print("Failed to read database: {}".format(exc), file=sys.stderr)
        return 1

    entries = parse_db_lines_usb(
        lines,
        stats,
        synthesize_dpad=not args.no_synthesize_dpad,
    )
    if not entries:
        print("No USB entries parsed.", file=sys.stderr)
        return 1

    entries = select_and_deduplicate(
        entries,
        stats,
        preferred_platform=preferred_platform,
        strict_platform=args.strict_platform,
        max_buttons=args.max_buttons,
        max_axes=args.max_axes,
        max_hats=args.max_hats,
    )
    if not entries:
        print("No entries remain after platform selection.", file=sys.stderr)
        return 1

    for entry in entries:
        validate_and_sanitize_entry(
            entry,
            max_buttons=args.max_buttons,
            max_axes=args.max_axes,
            max_hats=args.max_hats,
        )

    count_legacy_drops(entries, stats)
    finalize_selected_issues(entries, stats)

    if args.schema == "legacy":
        apply_legacy_filter(entries)
    else:
        apply_extended_filter(entries, args.paddles)

    entries = sort_entries(entries, args.sort)

    try:
        write_c_header(
            entries,
            out_path=args.out,
            source=args.url,
            strings_mode=args.strings,
            no_comments=args.no_comments,
            schema=args.schema,
            include_paddles=args.paddles,
            sort_mode=args.sort,
            preferred_platform=preferred_platform,
            strict_platform=args.strict_platform,
            max_buttons=args.max_buttons,
            max_axes=args.max_axes,
            max_hats=args.max_hats,
        )
    except OSError as exc:
        print("Failed to write {}: {}".format(args.out, exc), file=sys.stderr)
        return 1

    print_stats(
        stats,
        schema=args.schema,
        warning_limit=args.warning_limit,
        quiet_warnings=args.quiet_warnings,
    )
    print("OK -- generated {}".format(args.out), file=sys.stderr)

    if args.fail_on_warning and stats.issue_total:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
