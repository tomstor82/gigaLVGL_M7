/*******************************************************************************
 * Size: 20 px
 * Bpp: 4
 * Opts: --bpp 4 --size 20 --no-compress --font Montserrat-Regular.ttf --symbols W-0123456789 --format lvgl -o Montserrat20_0_9_W_minus.c
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "/home/tom/Arduino/libraries/lvgl/lvgl.h"
#endif

#ifndef MONTSERRAT20_0_9_W_MINUS
#define MONTSERRAT20_0_9_W_MINUS 1
#endif

#if MONTSERRAT20_0_9_W_MINUS

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+002D "-" */
    0x12, 0x22, 0x22, 0x20, 0x9f, 0xff, 0xff, 0xf1,
    0x9f, 0xff, 0xff, 0xf1,

    /* U+0030 "0" */
    0x0, 0x18, 0xdf, 0xfc, 0x60, 0x0, 0x2, 0xef,
    0xff, 0xff, 0xfb, 0x0, 0xc, 0xfe, 0x61, 0x28,
    0xff, 0x80, 0x3f, 0xf4, 0x0, 0x0, 0x8f, 0xf0,
    0x8f, 0xd0, 0x0, 0x0, 0x2f, 0xf4, 0xbf, 0x90,
    0x0, 0x0, 0xe, 0xf7, 0xcf, 0x80, 0x0, 0x0,
    0xc, 0xf8, 0xcf, 0x80, 0x0, 0x0, 0xc, 0xf8,
    0xbf, 0x90, 0x0, 0x0, 0xe, 0xf7, 0x8f, 0xd0,
    0x0, 0x0, 0x2f, 0xf4, 0x3f, 0xf4, 0x0, 0x0,
    0x8f, 0xf0, 0xc, 0xfe, 0x51, 0x17, 0xff, 0x80,
    0x2, 0xef, 0xff, 0xff, 0xfb, 0x0, 0x0, 0x18,
    0xdf, 0xfc, 0x60, 0x0,

    /* U+0031 "1" */
    0x6f, 0xff, 0xfa, 0x6f, 0xff, 0xfa, 0x2, 0x2a,
    0xfa, 0x0, 0x9, 0xfa, 0x0, 0x9, 0xfa, 0x0,
    0x9, 0xfa, 0x0, 0x9, 0xfa, 0x0, 0x9, 0xfa,
    0x0, 0x9, 0xfa, 0x0, 0x9, 0xfa, 0x0, 0x9,
    0xfa, 0x0, 0x9, 0xfa, 0x0, 0x9, 0xfa, 0x0,
    0x9, 0xfa,

    /* U+0032 "2" */
    0x0, 0x7d, 0xff, 0xb3, 0x0, 0x1d, 0xff, 0xff,
    0xff, 0x50, 0xbf, 0xf7, 0x35, 0xdf, 0xf0, 0x1a,
    0x30, 0x0, 0x2f, 0xf3, 0x0, 0x0, 0x0, 0x1f,
    0xf3, 0x0, 0x0, 0x0, 0x7f, 0xe0, 0x0, 0x0,
    0x4, 0xff, 0x50, 0x0, 0x0, 0x4f, 0xf8, 0x0,
    0x0, 0x4, 0xff, 0x90, 0x0, 0x0, 0x4f, 0xf9,
    0x0, 0x0, 0x5, 0xff, 0x90, 0x0, 0x0, 0x5f,
    0xfc, 0x44, 0x44, 0x43, 0xdf, 0xff, 0xff, 0xff,
    0xfb, 0xdf, 0xff, 0xff, 0xff, 0xfb,

    /* U+0033 "3" */
    0x7, 0xff, 0xff, 0xff, 0xfb, 0x0, 0x7f, 0xff,
    0xff, 0xff, 0xb0, 0x1, 0x22, 0x22, 0xdf, 0xe2,
    0x0, 0x0, 0x0, 0xbf, 0xe2, 0x0, 0x0, 0x0,
    0xbf, 0xe2, 0x0, 0x0, 0x0, 0x9f, 0xfe, 0xa4,
    0x0, 0x0, 0xa, 0xff, 0xff, 0xf6, 0x0, 0x0,
    0x10, 0x14, 0xdf, 0xf1, 0x0, 0x0, 0x0, 0x1,
    0xff, 0x50, 0x0, 0x0, 0x0, 0xd, 0xf6, 0x2,
    0x10, 0x0, 0x1, 0xff, 0x40, 0xbe, 0x84, 0x36,
    0xdf, 0xd0, 0xe, 0xff, 0xff, 0xff, 0xf3, 0x0,
    0x6, 0xcf, 0xfd, 0x91, 0x0,

    /* U+0034 "4" */
    0x0, 0x0, 0x7, 0xff, 0x20, 0x0, 0x0, 0x2,
    0xff, 0x80, 0x0, 0x0, 0x0, 0xbf, 0xd0, 0x0,
    0x0, 0x0, 0x5f, 0xf3, 0x0, 0x0, 0x0, 0x1e,
    0xf9, 0x0, 0x0, 0x0, 0xa, 0xfe, 0x11, 0x65,
    0x0, 0x4, 0xff, 0x50, 0x4f, 0xf0, 0x0, 0xdf,
    0xb0, 0x4, 0xff, 0x0, 0x4f, 0xff, 0xff, 0xff,
    0xff, 0xa4, 0xff, 0xff, 0xff, 0xff, 0xfa, 0x2,
    0x22, 0x22, 0x6f, 0xf2, 0x10, 0x0, 0x0, 0x4,
    0xff, 0x0, 0x0, 0x0, 0x0, 0x4f, 0xf0, 0x0,
    0x0, 0x0, 0x4, 0xff, 0x0,

    /* U+0035 "5" */
    0x5f, 0xff, 0xff, 0xff, 0xb0, 0x5f, 0xff, 0xff,
    0xff, 0xb0, 0x5f, 0xd3, 0x33, 0x33, 0x20, 0x5f,
    0xd0, 0x0, 0x0, 0x0, 0x5f, 0xd0, 0x0, 0x0,
    0x0, 0x5f, 0xed, 0xff, 0xc5, 0x0, 0x3f, 0xff,
    0xff, 0xff, 0x90, 0x6, 0x51, 0x3, 0xbf, 0xf3,
    0x0, 0x0, 0x0, 0xe, 0xf8, 0x0, 0x0, 0x0,
    0xc, 0xf9, 0x5, 0x0, 0x0, 0x1f, 0xf7, 0x9f,
    0xc7, 0x57, 0xef, 0xf1, 0x8f, 0xff, 0xff, 0xff,
    0x40, 0x4, 0xae, 0xfe, 0xa2, 0x0,

    /* U+0036 "6" */
    0x0, 0x7, 0xdf, 0xeb, 0x50, 0x0, 0x1d, 0xff,
    0xff, 0xff, 0x90, 0xb, 0xff, 0x73, 0x37, 0xd2,
    0x3, 0xff, 0x40, 0x0, 0x0, 0x0, 0x7f, 0xc0,
    0x1, 0x10, 0x0, 0xa, 0xf9, 0x6d, 0xff, 0xd5,
    0x0, 0xcf, 0xef, 0xff, 0xff, 0xf6, 0xc, 0xff,
    0xc3, 0x2, 0xbf, 0xf0, 0xbf, 0xf1, 0x0, 0x0,
    0xef, 0x48, 0xfd, 0x0, 0x0, 0xc, 0xf5, 0x4f,
    0xf2, 0x0, 0x0, 0xef, 0x30, 0xdf, 0xd5, 0x24,
    0xcf, 0xe0, 0x3, 0xef, 0xff, 0xff, 0xf3, 0x0,
    0x2, 0x9e, 0xfe, 0xa2, 0x0,

    /* U+0037 "7" */
    0x1f, 0xff, 0xff, 0xff, 0xff, 0xc1, 0xff, 0xff,
    0xff, 0xff, 0xfc, 0x1f, 0xf3, 0x33, 0x35, 0xff,
    0x91, 0xff, 0x10, 0x0, 0xaf, 0xf1, 0x1, 0x10,
    0x0, 0x2f, 0xf9, 0x0, 0x0, 0x0, 0x9, 0xff,
    0x10, 0x0, 0x0, 0x1, 0xff, 0x90, 0x0, 0x0,
    0x0, 0x9f, 0xf1, 0x0, 0x0, 0x0, 0x1f, 0xf9,
    0x0, 0x0, 0x0, 0x9, 0xff, 0x10, 0x0, 0x0,
    0x1, 0xff, 0x90, 0x0, 0x0, 0x0, 0x8f, 0xf1,
    0x0, 0x0, 0x0, 0x1f, 0xf9, 0x0, 0x0, 0x0,
    0x8, 0xff, 0x10, 0x0, 0x0,

    /* U+0038 "8" */
    0x0, 0x4b, 0xef, 0xda, 0x20, 0x0, 0x9f, 0xff,
    0xef, 0xff, 0x50, 0x3f, 0xf7, 0x0, 0x1b, 0xfe,
    0x4, 0xfe, 0x0, 0x0, 0x3f, 0xf0, 0xe, 0xf6,
    0x0, 0xa, 0xfb, 0x0, 0x3f, 0xfe, 0xdf, 0xfd,
    0x10, 0xa, 0xff, 0xff, 0xff, 0xf6, 0x6, 0xff,
    0x92, 0x3, 0xcf, 0xf1, 0xbf, 0xb0, 0x0, 0x1,
    0xff, 0x7d, 0xf8, 0x0, 0x0, 0xd, 0xf8, 0xbf,
    0xc0, 0x0, 0x1, 0xff, 0x75, 0xff, 0xb3, 0x15,
    0xdf, 0xf1, 0x9, 0xff, 0xff, 0xff, 0xf5, 0x0,
    0x5, 0xbe, 0xfe, 0xa2, 0x0,

    /* U+0039 "9" */
    0x0, 0x7d, 0xfe, 0xb5, 0x0, 0x0, 0xcf, 0xff,
    0xff, 0xf8, 0x0, 0x8f, 0xf7, 0x23, 0xaf, 0xf3,
    0xd, 0xf6, 0x0, 0x0, 0xcf, 0xa0, 0xff, 0x30,
    0x0, 0x7, 0xff, 0xe, 0xf6, 0x0, 0x0, 0xbf,
    0xf1, 0x9f, 0xf6, 0x23, 0xaf, 0xff, 0x21, 0xdf,
    0xff, 0xff, 0xdf, 0xf2, 0x1, 0x9e, 0xfd, 0x73,
    0xff, 0x10, 0x0, 0x0, 0x0, 0x6f, 0xe0, 0x0,
    0x0, 0x0, 0xd, 0xf9, 0x0, 0xba, 0x42, 0x5c,
    0xff, 0x20, 0x3f, 0xff, 0xff, 0xff, 0x50, 0x0,
    0x29, 0xef, 0xea, 0x30, 0x0,

    /* U+0057 "W" */
    0x9f, 0xe0, 0x0, 0x0, 0xb, 0xfb, 0x0, 0x0,
    0x0, 0xff, 0x83, 0xff, 0x40, 0x0, 0x0, 0xff,
    0xf0, 0x0, 0x0, 0x5f, 0xf3, 0xe, 0xfa, 0x0,
    0x0, 0x5f, 0xff, 0x50, 0x0, 0xb, 0xfd, 0x0,
    0x8f, 0xf0, 0x0, 0xa, 0xff, 0xf9, 0x0, 0x1,
    0xff, 0x70, 0x3, 0xff, 0x50, 0x0, 0xef, 0xcf,
    0xe0, 0x0, 0x6f, 0xf2, 0x0, 0xd, 0xfa, 0x0,
    0x4f, 0xf3, 0xff, 0x30, 0xb, 0xfc, 0x0, 0x0,
    0x7f, 0xf0, 0x9, 0xfc, 0xc, 0xf8, 0x1, 0xff,
    0x70, 0x0, 0x2, 0xff, 0x50, 0xdf, 0x70, 0x7f,
    0xd0, 0x7f, 0xf1, 0x0, 0x0, 0xc, 0xfb, 0x2f,
    0xf2, 0x2, 0xff, 0x2c, 0xfc, 0x0, 0x0, 0x0,
    0x7f, 0xf8, 0xfd, 0x0, 0xe, 0xf9, 0xff, 0x60,
    0x0, 0x0, 0x1, 0xff, 0xff, 0x80, 0x0, 0x9f,
    0xff, 0xf1, 0x0, 0x0, 0x0, 0xc, 0xff, 0xf3,
    0x0, 0x4, 0xff, 0xfb, 0x0, 0x0, 0x0, 0x0,
    0x6f, 0xfe, 0x0, 0x0, 0xf, 0xff, 0x50, 0x0,
    0x0, 0x0, 0x1, 0xff, 0x90, 0x0, 0x0, 0xaf,
    0xf0, 0x0, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 152, .box_w = 8, .box_h = 3, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 12, .adv_w = 220, .box_w = 12, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 96, .adv_w = 122, .box_w = 6, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 138, .adv_w = 188, .box_w = 10, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 208, .adv_w = 185, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 285, .adv_w = 183, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 362, .adv_w = 185, .box_w = 10, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 432, .adv_w = 198, .box_w = 11, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 509, .adv_w = 181, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 586, .adv_w = 204, .box_w = 11, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 663, .adv_w = 198, .box_w = 11, .box_h = 14, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 740, .adv_w = 335, .box_w = 21, .box_h = 14, .ofs_x = 0, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
    0xa, 0xb, 0xc, 0x2a
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 45, .range_length = 43, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 12, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Pair left and right glyphs for kerning*/
static const uint8_t kern_pair_glyph_ids[] =
{
    1, 3,
    1, 4,
    1, 5,
    1, 9,
    1, 12,
    4, 1,
    6, 3,
    6, 9,
    9, 1,
    9, 6,
    12, 1,
    12, 6
};

/* Kerning between the respective left and right glyphs
 * 4.4 format which needs to scaled with `kern_scale`*/
static const int8_t kern_pair_values[] =
{
    -9, -6, -6, -6, -6, -4, -4, -4,
    -8, -6, -6, -4
};

/*Collect the kern pair's data in one place*/
static const lv_font_fmt_txt_kern_pair_t kern_pairs =
{
    .glyph_ids = kern_pair_glyph_ids,
    .values = kern_pair_values,
    .pair_cnt = 12,
    .glyph_ids_size = 0
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = &kern_pairs,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t Montserrat20_0_9_W_minus = {
#else
lv_font_t Montserrat20_0_9_W_minus = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 14,          /*The maximum line height required by the font*/
    .base_line = 0,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if MONTSERRAT20_0_9_W_MINUS*/

