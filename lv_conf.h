/**
 * @file lv_conf.h
 * Configuration file for LVGL v8.3.x
 * Optimisé pour Waveshare ESP32-S3 800x480 RGB LCD
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/* Color depth: 1 (1 byte per pixel), 8 (RGB332), 16 (RGB565), 32 (ARGB8888) */
#define LV_COLOR_DEPTH 16

/* Swap the 2 bytes of RGB565 color. Useful if the display has an 8-bit interface (e.g. SPI) */
#define LV_COLOR_16_SWAP 0

/* Enable more complex drawing routines to manage screens transparency. */
#define LV_COLOR_SCREEN_TRANSP 0

/* Adjust color mix functions rounding. GPUs might calculate color mix differently. */
#define LV_COLOR_MIX_ROUND_OFS (LV_COLOR_DEPTH == 32 ? 0: 128)

/* Pre-multiply the alpha channel with the color. Performance gain but the color loses a bit of quality. */
#define LV_COLOR_CHROMA_KEY_HEX 0x00FF00         /* Pure green */

/*====================
   MEMORY SETTINGS
 *====================*/

/* Size of the memory available for `lv_mem_alloc()` in bytes (>= 2kB) */
/* IMPORTANT: Utiliser malloc pour profiter de la PSRAM sur ESP32-S3 */
#define LV_MEM_CUSTOM 1
#if LV_MEM_CUSTOM == 0
    /* Size of the memory which should be available for the memory manager (>= 2kB) */
    #define LV_MEM_SIZE (48U * 1024U)          /* [bytes] */

    /* Set an address for the memory pool instead of allocating it as a normal array. Can be in external SRAM too. */
    #define LV_MEM_ADR 0     /* 0: unused */

    /* Instead of an address give a memory allocator that will be called to get a memory pool for LVGL. */
    // #define LV_MEM_POOL_INCLUDE your_alloc_library  /* Uncomment if using */
    // #define LV_MEM_POOL_ALLOC   your_alloc          /* Uncomment if using */
#else       /* LV_MEM_CUSTOM */
    #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>   /* Header for the dynamic memory function */
    #define LV_MEM_CUSTOM_ALLOC   malloc
    #define LV_MEM_CUSTOM_FREE    free
    #define LV_MEM_CUSTOM_REALLOC realloc
#endif      /* LV_MEM_CUSTOM */

/* Number of the intermediate memory buffer used during rendering and other internal processing mechanisms. */
#define LV_MEM_BUF_MAX_NUM 16

/* Use the standard `memcpy` and `memset` instead of LVGL's own functions. (Might be faster). */
#define LV_MEMCPY_MEMSET_STD 0

/*====================
   HAL SETTINGS
 *====================*/

/* Default display refresh period. LVG will redraw changed areas with this period time */
/* 16ms = 60Hz, synchronisé avec le refresh rate de l'écran RGB pour éviter le tearing */
#define LV_DISP_DEF_REFR_PERIOD 16      /* [ms] */

/* Input device read period in milliseconds */
#define LV_INDEV_DEF_READ_PERIOD 30     /* [ms] */

/* Use a custom tick source that tells the elapsed time in milliseconds. */
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE "Arduino.h"         /* Header for the system time function */
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())    /* Expression evaluating to current system time in ms */
#endif   /* LV_TICK_CUSTOM */

/* Default Dot Per Inch. Used to initialize default sizes such as widgets sized, style paddings. */
#define LV_DPI_DEF 130     /* [px/inch] */

/*====================
   FEATURE CONFIGURATION
 *====================*/

/* Montserrat fonts with ASCII range and some symbols using bpp = 4 */
#define LV_FONT_MONTSERRAT_8     0
#define LV_FONT_MONTSERRAT_10    0
#define LV_FONT_MONTSERRAT_12    0
#define LV_FONT_MONTSERRAT_14    1
#define LV_FONT_MONTSERRAT_16    1
#define LV_FONT_MONTSERRAT_18    0
#define LV_FONT_MONTSERRAT_20    1
#define LV_FONT_MONTSERRAT_22    0
#define LV_FONT_MONTSERRAT_24    1
#define LV_FONT_MONTSERRAT_26    0
#define LV_FONT_MONTSERRAT_28    0
#define LV_FONT_MONTSERRAT_30    0
#define LV_FONT_MONTSERRAT_32    0
#define LV_FONT_MONTSERRAT_34    0
#define LV_FONT_MONTSERRAT_36    0
#define LV_FONT_MONTSERRAT_38    0
#define LV_FONT_MONTSERRAT_40    0
#define LV_FONT_MONTSERRAT_42    0
#define LV_FONT_MONTSERRAT_44    0
#define LV_FONT_MONTSERRAT_46    0
#define LV_FONT_MONTSERRAT_48    0

/* Demonstrate special features */
#define LV_FONT_MONTSERRAT_12_SUBPX      0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0  /* bpp = 3 */
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0  /* Hebrew, Arabic, Persian letters and all their forms */
#define LV_FONT_SIMSUN_16_CJK            0  /* 1000 most common CJK radicals */

/* Pixel perfect monospace fonts */
#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0

/* Optionally declare custom fonts here. */
#define LV_FONT_CUSTOM_DECLARE

/* Enable handling large font and/or fonts with a lot of characters. */
#define LV_FONT_FMT_TXT_LARGE 0

/* Enables/disables support for compressed fonts. */
#define LV_FONT_COMPRESSED 0

/* Enable subpixel rendering */
#define LV_FONT_SUBPX_BGR 0  /* Set the pixel order of the display. */

/* Enable drawing placeholders when glyph dsc is not found */
#define LV_USE_FONT_PLACEHOLDER 1

/* Select a character encoding for strings. */
#define LV_TXT_ENC LV_TXT_ENC_UTF8

/* Can break (wrap) texts on these chars */
#define LV_TXT_BREAK_CHARS " ,.;:-_"

/* If a word is at least this long, will break wherever "prettiest" */
#define LV_TXT_LINE_BREAK_LONG_LEN 0

/* Minimum number of characters in a long word to put on a line before a break. */
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3

/* Minimum number of characters in a long word to put on a line after a break. */
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

/* The control character to use for signalling text recoloring. */
#define LV_TXT_COLOR_CMD "#"

/* Support bidirectional texts. Allows mixing Left-to-Right and Right-to-Left texts. */
#define LV_USE_BIDI 0
#if LV_USE_BIDI
    /* Set the default direction. Supported values: `LV_BASE_DIR_LTR`, `LV_BASE_DIR_RTL`, `LV_BASE_DIR_AUTO` */
    #define LV_BIDI_BASE_DIR_DEF LV_BASE_DIR_AUTO
#endif

/* Enable Arabic/Persian processing */
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*====================
 * WIDGET USAGE
 *====================*/

/* Documentation of the widgets: https://docs.lvgl.io/latest/en/html/widgets/index.html */

#define LV_USE_ARC        1
#define LV_USE_BAR        1
#define LV_USE_BTN        1
#define LV_USE_BTNMATRIX  1
#define LV_USE_CANVAS     1
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1   /* Requires: lv_label */
#define LV_USE_IMG        1   /* Requires: lv_label */
#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION 1   /* Enable selecting text of the label */
    #define LV_LABEL_LONG_TXT_HINT 1    /* Store some extra info in labels to speed up drawing of very long texts */
#endif
#define LV_USE_LINE       1
#define LV_USE_ROLLER     1   /* Requires: lv_label */
#if LV_USE_ROLLER
    #define LV_ROLLER_INF_PAGES 7   /* Number of extra "pages" when the roller is infinite */
#endif
#define LV_USE_SLIDER     1   /* Requires: lv_bar */
#define LV_USE_SWITCH     1
#define LV_USE_TEXTAREA   1   /* Requires: lv_label */
#if LV_USE_TEXTAREA != 0
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500    /* ms */
#endif
#define LV_USE_TABLE      1

/*====================
 * EXTRA COMPONENTS
 *====================*/

/* Enable animations */
#define LV_USE_ANIMATION 1
#if LV_USE_ANIMATION
    /* Declare the type of the user data of animations (can be e.g. `void *`, `int`, `struct`) */
    typedef void * lv_anim_user_data_t;
#endif

/* Enable snapshot for screens */
#define LV_USE_SNAPSHOT 0

/* Enable image cache for read only images */
#define LV_IMG_CACHE_DEF_SIZE 0

/* Enable file system */
#define LV_USE_FS_STDIO 0
#define LV_USE_FS_POSIX 0
#define LV_USE_FS_WIN32 0
#define LV_USE_FS_FATFS 0

/*====================
 * THEMES
 *====================*/

/* A simple, impressive and very complete theme */
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
    /* 0: Light mode; 1: Dark mode */
    #define LV_THEME_DEFAULT_DARK 0

    /* 1: Enable grow on press */
    #define LV_THEME_DEFAULT_GROW 1

    /* Default transition time in [ms] */
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif

/* A very simple theme that is a good starting point for a custom theme */
#define LV_USE_THEME_BASIC 1

/* A theme designed for monochrome displays */
#define LV_USE_THEME_MONO 1

/*====================
 * LOGGING
 *====================*/

/* Enable the log module */
#define LV_USE_LOG 1
#if LV_USE_LOG
    /* How important log should be added:
     * LV_LOG_LEVEL_TRACE       A lot of logs to give detailed information
     * LV_LOG_LEVEL_INFO        Log important events
     * LV_LOG_LEVEL_WARN        Log if something unwanted happened but didn't cause a problem
     * LV_LOG_LEVEL_ERROR       Only critical issue, when the system may fail
     * LV_LOG_LEVEL_USER        Only logs added by the user
     * LV_LOG_LEVEL_NONE        Do not log anything */
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN

    /* 1: Print the log with 'printf'; 0: User need to register a callback with `lv_log_register_print_cb()` */
    #define LV_LOG_PRINTF 1

    /* Enable/disable LV_LOG_TRACE in modules that produces a huge number of logs */
    #define LV_LOG_TRACE_MEM        1
    #define LV_LOG_TRACE_TIMER      1
    #define LV_LOG_TRACE_INDEV      1
    #define LV_LOG_TRACE_DISP_REFR  1
    #define LV_LOG_TRACE_EVENT      1
    #define LV_LOG_TRACE_OBJ_CREATE 1
    #define LV_LOG_TRACE_LAYOUT     1
    #define LV_LOG_TRACE_ANIM       1
#endif

/*====================
 * ASSERTS
 *====================*/

/* Enable asserts if an operation is failed or an invalid data is found. */
#define LV_USE_ASSERT_NULL          1   /* Check if the parameter is NULL. (Very fast, recommended) */
#define LV_USE_ASSERT_MALLOC        1   /* Checks is the memory is successfully allocated or no. (Very fast, recommended) */
#define LV_USE_ASSERT_STYLE         0   /* Check if the styles are properly initialized. (Very fast, recommended) */
#define LV_USE_ASSERT_MEM_INTEGRITY 0   /* Check the integrity of `lv_mem` after critical operations. (Slow) */
#define LV_USE_ASSERT_OBJ           0   /* Check the object's type and existence (e.g. not deleted). (Slow) */

/* Add a custom handler when assert happens e.g. to restart the MCU */
#define LV_ASSERT_HANDLER_INCLUDE <stdint.h>
#define LV_ASSERT_HANDLER while(1);   /* Halt by default */

/*====================
 * OTHERS
 *====================*/

/* 1: Show CPU usage and FPS count */
#define LV_USE_PERF_MONITOR 0
#if LV_USE_PERF_MONITOR
    #define LV_USE_PERF_MONITOR_POS LV_ALIGN_BOTTOM_RIGHT
#endif

/* 1: Show the used memory and the memory fragmentation
 * Requires LV_MEM_CUSTOM = 0 */
#define LV_USE_MEM_MONITOR 0
#if LV_USE_MEM_MONITOR
    #define LV_USE_MEM_MONITOR_POS LV_ALIGN_BOTTOM_LEFT
#endif

/* Set the working memory size for malloc/free */
#define LV_MEM_SIZE (48U * 1024U)

/* Garbage Collector settings */
#define LV_ENABLE_GC 0
#if LV_ENABLE_GC != 0
    #define LV_GC_INCLUDE "gc.h"           /* Include Garbage Collector related things */
#endif /* LV_ENABLE_GC */

/*====================
 * COMPILER SETTINGS
 *====================*/

/* For big endian systems set to 1 */
#define LV_BIG_ENDIAN_SYSTEM 0

/* Define a custom attribute to `lv_tick_inc` function */
#define LV_ATTRIBUTE_TICK_INC

/* Define a custom attribute to `lv_timer_handler` function */
#define LV_ATTRIBUTE_TIMER_HANDLER

/* Define a custom attribute to `lv_disp_flush_ready` function */
#define LV_ATTRIBUTE_FLUSH_READY

/* Required alignment size for buffers */
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 1

/* Will be added where memories needs to be aligned (with -Os data might not be aligned to boundary by default). */
#define LV_ATTRIBUTE_MEM_ALIGN

/* Attribute to mark large constant arrays for example font's bitmaps */
#define LV_ATTRIBUTE_LARGE_CONST

/* Compiler prefix for a big array declaration in RAM */
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY

/* Place performance critical functions into a faster memory (e.g RAM) */
#define LV_ATTRIBUTE_FAST_MEM

/* Prefix variables that are used in GPU accelerated operations, often these need to be placed in RAM sections that are DMA accessible */
#define LV_ATTRIBUTE_DMA

/* Export integer constant to binding. This macro is used with constants in the form of LV_<CONST> that should also appear on LVGL binding API such as Micropython. */
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning /* The default value just prevents GCC warning */

/* Extend the default -32k..32k coordinate range to -4M..4M by using int32_t for coordinates instead of int16_t */
#define LV_USE_LARGE_COORD 0

/*====================
 * DEMO USAGE
 *====================*/

/* Show some widget. It might be required to increase `LV_MEM_SIZE` */
#define LV_USE_DEMO_WIDGETS 0
#if LV_USE_DEMO_WIDGETS
#define LV_DEMO_WIDGETS_SLIDESHOW 0
#endif

/* Demonstrate the usage of encoder and keyboard */
#define LV_USE_DEMO_KEYPAD_AND_ENCODER 0

/* Benchmark your system */
#define LV_USE_DEMO_BENCHMARK 0
#if LV_USE_DEMO_BENCHMARK
/* Use RGB565A8 images with 16 bit color depth instead of ARGB8565 */
#define LV_DEMO_BENCHMARK_RGB565A8 0
#endif

/* Stress test for LVGL */
#define LV_USE_DEMO_STRESS 0

/* Music player demo */
#define LV_USE_DEMO_MUSIC 0
#if LV_USE_DEMO_MUSIC
    #define LV_DEMO_MUSIC_SQUARE    0
    #define LV_DEMO_MUSIC_LANDSCAPE 0
    #define LV_DEMO_MUSIC_ROUND     0
    #define LV_DEMO_MUSIC_LARGE     0
    #define LV_DEMO_MUSIC_AUTO_PLAY 0
#endif

#endif /* LV_CONF_H */
