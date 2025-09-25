file(GLOB U8G2_SRC ../../u8g2/csrc/*.c)

idf_component_register(SRCS
	"../../main.c"
	"../../hid.c"
	"../../hidparser.c"
	"../../sysctrl.c"
	"../../sdc.c"
	"../../menu.c"
	"../../osd_u8g2.c"
	"../../inifile.c"
	"../../config.c"
        "../../at_wifi.c"
	"../../xml.c"
	"../mcu_hw.c"
	"../../puff.c"
	${U8G2_SRC}	
	../../u8g2/sys/bitmap/common/u8x8_d_bitmap.c

	INCLUDE_DIRS
	"."
	".."
	"../../u8g2/csrc"
)
