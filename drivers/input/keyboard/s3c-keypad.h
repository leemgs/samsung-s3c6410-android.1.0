/*
 *	
 * $Id: s3c-keypad.h,v 1.5 2008/05/23 06:22:53 dark0351 Exp $
 */

#ifndef _S3C_KEYPAD_H_
#define _S3C_KEYPAD_H_

static void __iomem *key_base;

#if defined(CONFIG_CPU_S3C6400) || defined (CONFIG_CPU_S3C6410)
#define KEYPAD_COLUMNS	8
#define KEYPAD_ROWS	8
#define MAX_KEYPAD_NR	64	/* 8*8 */
#define MAX_KEYMASK_NR	32

int keypad_keycode[] = {
		1, 2, KEY_1, KEY_Q, KEY_A, 6, 7, KEY_LEFT,
		9, 10, KEY_2, KEY_W, KEY_S, KEY_Z, KEY_RIGHT, 16,
		17, 18, KEY_3, KEY_E, KEY_D, KEY_X, 23, KEY_UP,
		25, 26, KEY_4, KEY_R, KEY_F, KEY_C, 31, 32,
		33, KEY_O, KEY_5, KEY_T, KEY_G, KEY_V, KEY_DOWN, KEY_BACKSPACE,
		KEY_P, KEY_0, KEY_6, KEY_Y, KEY_H, KEY_SPACE, 47, 48,
		KEY_M, KEY_L, KEY_7, KEY_U, KEY_J, KEY_N, 55, KEY_ENTER,
		KEY_LEFTSHIFT, KEY_9, KEY_8, KEY_I, KEY_K, KEY_B, 63, KEY_COMMA
	};


#define KEYPAD_DELAY		(50)
#define KEYPAD_ROW_GPIOCON	S3C_GPK1CON
#define KEYPAD_ROW_GPIOPUD	S3C_GPKPU
#define KEYPAD_COL_GPIOCON	S3C_GPL0CON
#define KEYPAD_COL_GPIOPUD	S3C_GPLPU

#define KEYPAD_ROW_GPIO_SET	\
	((readl(KEYPAD_ROW_GPIOCON) & ~(0xfffffff)) | 0x33333333)
	
#define KEYPAD_COL_GPIO_SET	\
	((readl(KEYPAD_COL_GPIOCON) & ~(0xfffffff)) | 0x33333333)

#define KEYPAD_ROW_GPIOPUD_DIS	(readl(KEYPAD_ROW_GPIOPUD) & ~(0xffff<<16))
#define KEYPAD_COL_GPIOPUD_DIS	(readl(KEYPAD_COL_GPIOPUD) & ~0xffff)

#define	KEYIFCOL_CLEAR		(readl(key_base+S3C_KEYIFCOL) & ~0xffff)
#define	KEYIFCON_CLEAR		(readl(key_base+S3C_KEYIFCON) & ~0x1f)
#define KEYIFFC_DIV		(readl(key_base+S3C_KEYIFFC) | 0x1)

#else
#error "Not supported S3C Configuration!!"
#endif

struct s3c_keypad {
	struct input_dev *dev;
	int nr_rows;	
	int no_cols;
	int total_keys; 
	int keycodes[MAX_KEYPAD_NR];
};


#endif				/* _S3C_KEYIF_H_ */
