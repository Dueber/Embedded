#ifend CLK_CONFIG_H_
#define CLK_CONFIG_H_

#define CONFIG_SYSCLK_SOURCE			  SYSCLK_SRC_PLL

#define CONFIG_PLL0_SOURCE          PLL_SRC_XOSC
#define CONFIG_PLL0_MUL             (32000000UL / BOARD_XOSC_HZ)
#define CONFIG_PLL0_DIV             1





#endif
