
#include "../usb_controller_maps_tools.h"
#include "../usb_controller_maps.h"
#include "../debug.h"

// This is a custom SDL USB controller map entry for the
// Data Frog PC07 pair of wireless SNES-like controllers.
// This fixes the shoulder buttons.
static const UsbGamepadMap data_frog_pc07 =
  { "03000000100800000100000010010000", "DATA FROG PC07 Twin SNES Adapter",
    0x0810, 0x0001, 0x0110,
    2, 1, 3, 0,     // btn a, b, x, y
    8, -1, 9,       // btn back (select), guide, start
    4, 5,           // btn left shoulder, right shoulder
    -1, -1,         // btn left stick, right stick
    -1, -1, -1, -1, // dpad up, down, left, right
    0,              // dpad hat
    1, 2, 4, 8,     // dpad hat up, right, down, left
    0, 1, 3, 2,     // axis lx, ly, rx, ry
    0, 0, 0, 0,
    -1, -1,
    0, 0,
    -1, -1,       // btn lt, rt
    0, 0, 0, 0,
    0, 0,
    -1, -1, -1, -1,
    0, 0, 0, 0,
    0, 0, 0, 0
  };

const UsbGamepadMap *usb_controller_maps_get_data_frog_pc07(const UsbGamepadMap *map) {
  // check if map needs replacement
  if(map->vid == 0x0810 && map->pid == 0x0001)
    return &data_frog_pc07;

  return map;  // nothing to do, just return the original pointer
}

// Lookup if there is a map for current gamepad
const UsbGamepadMap *usb_controller_maps_find(uint16_t vid, uint16_t pid, int version_optional) {
  /*
   * Selection order:
   *   1. exact VID/PID/bcdDevice match,
   *   2. VID/PID entry with version == 0 (generic/default SDL entry),
   *   3. first VID/PID entry as a deterministic last-resort fallback.
   *
   * The old code only populated fallback when version_optional < 0.  The
   * normal caller always passes bcdDevice, so a missing exact version could
   * never fall back and the SDL map was silently disabled.
   */
  const UsbGamepadMap *first_vid_pid = NULL;
  const UsbGamepadMap *version_zero = NULL;

  for (size_t i = 0; i < kUsbGamepadMapsCount; i++)
  {
    const UsbGamepadMap *m = &kUsbGamepadMaps[i];

    if (m->vid != vid || m->pid != pid)
      continue;

    if (!first_vid_pid)
      first_vid_pid = m;

    if (!version_zero && m->version == 0)
      version_zero = m;

    if (version_optional >= 0 &&
        m->version == (uint16_t)version_optional)
      return m;
  }

  if (version_zero)
    return version_zero;

  return first_vid_pid;
}

void usb_controller_maps_show(const UsbGamepadMap *map) {
  usb_debugf("Gamepad map: %s (VID=%04x PID=%04x VER=%04x)",
	     map->name, map->vid, map->pid, map->version);
  
  char str0[8], str1[8], str2[8], str3[8];
  char str4[16], str5[16], str6[16], str7[16];
      
  if(map->btn_a>=0)    sprintf(str0, " A=%d", map->btn_a); else str0[0] = '\0';      
  if(map->btn_b>=0)    sprintf(str1, " B=%d", map->btn_b); else str1[0] = '\0';
  if(map->btn_x>=0)    sprintf(str2, " X=%d", map->btn_x); else str2[0] = '\0';
  if(map->btn_y>=0)    sprintf(str3, " Y=%d", map->btn_y); else str3[0] = '\0';            
  if(map->btn_back>=0) sprintf(str4, " SEL=%d", map->btn_back); else str4[0] = '\0';            
  if(map->btn_start>=0) sprintf(str5, " STA=%d", map->btn_start); else str5[0] = '\0';            
  if(map->btn_leftshoulder>=0) sprintf(str6, " LTRG=%d", map->btn_leftshoulder); else str6[0] = '\0';
  if(map->btn_rightshoulder>=0) sprintf(str7, " RTRG=%d", map->btn_rightshoulder); else str7[0] = '\0';
  usb_debugf("  buttons:%s%s%s%s%s%s%s%s", str0, str1, str2, str3, str4, str5, str6, str7);

  if(map->axis_lx>=0 || map->axis_ly>=0) {
    if(map->axis_lx>=0) sprintf(str0, " LX=%d", map->axis_lx); else str0[0] = '\0';      
    if(map->axis_ly>=0) sprintf(str1, " LY=%d", map->axis_ly); else str1[0] = '\0';      
    usb_debugf("  analogue axes:%s%s", str0, str1);
  }
  
  if(map->dpad_axis_up>=0 || map->dpad_axis_down>=0 ||
     map->dpad_axis_left>=0 || map->dpad_axis_right >= 0) {
    
    if(map->dpad_axis_up>=0)
      sprintf(str0, " U=%d", map->dpad_axis_up); else str0[0] = '\0';      
    if(map->dpad_axis_down>=0)
      sprintf(str1, " D=%d", map->dpad_axis_down); else str1[0] = '\0';      
    if(map->dpad_axis_left>=0)
      sprintf(str2, " L=%d", map->dpad_axis_left); else str2[0] = '\0';      
    if(map->dpad_axis_right>=0)
      sprintf(str3, " R=%d", map->dpad_axis_right); else str3[0] = '\0';      
    
    usb_debugf("  dpad axes:%s%s%s%s", str0, str1, str2, str3);
  }
}
  
