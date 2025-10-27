#ifndef STM32F446XX_H
#define STM32F446XX_H

#include <stdint.h>


/************************************/
/* Base addresses of memory regions */
/************************************/
#define FLASH_BASE_ADDR             (0x08000000U)
#define SYSMEM_BASE_ADDR            (0x1FFF0000U)
#define SRAM1_BASE_ADDR             (0x20000000U)
#define SRAM2_BASE_ADDR             (0x2001C000U)
#define SRAM_BASE_ADDR              (SRAM1_BASE_ADDR)



/***************************/
/* Base addresses of Buses */
/***************************/
#define AHB1_BASE_ADDR              (0x40020000U)
#define AHB2_BASE_ADDR				(0x50000000U)
#define APB1_BASE_ADDR              (0x40000000U)
#define APB2_BASE_ADDR				(0x40010000U)
#define AHB3_BASE_ADDR				(0xA0001000U)

/****************************/
/* core processor registers */
/****************************/


/**********************************/
/* AHB1 peripheral base addresses */
/**********************************/
#define RCC_BASE_ADDR               ((AHB1_BASE_ADDR) + (0x3800U))
#define GPIOA_BASE_ADDR				((AHB1_BASE_ADDR) + (0x0000U))
#define GPIOB_BASE_ADDR				((AHB1_BASE_ADDR) + (0x0400U))
#define GPIOC_BASE_ADDR				((AHB1_BASE_ADDR) + (0x0800U))
#define GPIOD_BASE_ADDR				((AHB1_BASE_ADDR) + (0x0C00U))
#define GPIOE_BASE_ADDR				((AHB1_BASE_ADDR) + (0x1000U))
#define GPIOF_BASE_ADDR				((AHB1_BASE_ADDR) + (0x1400U))
#define GPIOG_BASE_ADDR				((AHB1_BASE_ADDR) + (0x1800U))
#define GPIOH_BASE_ADDR				((AHB1_BASE_ADDR) + (0x1C00U))


/******************************/
/* AHB1 peripheral structures */
/******************************/
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t _RESERVED1;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t _RESERVED2[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t _RESERVED3;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t _RESERVED4[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint32_t _RESERVED5;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t _RESERVED6[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t _RESERVED7[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t PLLSAICFGR;
    volatile uint32_t DCKCFGR;
    volatile uint32_t CKGATENR;
    volatile uint32_t CDKCFGR2;
} RCC_regdef_t;


typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDER;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_regdef_t;



/***********************************************/
/* AHB1 register structure pointer definitions */
/***********************************************/
#define RCC             ((RCC_regdef_t *const)RCC_BASE_ADDR);
#define GPIOA           ((GPIO_regdef_t *const)GPIOA_BASE_ADDR);
#define GPIOB           ((GPIO_regdef_t *const)GPIOB_BASE_ADDR);
#define GPIOC           ((GPIO_regdef_t *const)GPIOC_BASE_ADDR);
#define GPIOD           ((GPIO_regdef_t *const)GPIOD_BASE_ADDR);
#define GPIOE           ((GPIO_regdef_t *const)GPIOE_BASE_ADDR);
#define GPIOF           ((GPIO_regdef_t *const)GPIOF_BASE_ADDR);
#define GPIOG           ((GPIO_regdef_t *const)GPIOG_BASE_ADDR);
#define GPIOH           ((GPIO_regdef_t *const)GPIOH_BASE_ADDR);

/**************************/
/* AHB1 peripheral macros */
/**************************/

/* @GPIO_MODER */
#define GPIO_MODER_MODER0           (0UL)
#define GPIO_MODER_MODER1           (1UL)
#define GPIO_MODER_MODER2           (2UL)
#define GPIO_MODER_MODER3           (3UL)
#define GPIO_MODER_MODER4           (4UL)
#define GPIO_MODER_MODER5           (5UL)
#define GPIO_MODER_MODER6           (6UL)
#define GPIO_MODER_MODER7           (7UL)
#define GPIO_MODER_MODER8           (8UL)
#define GPIO_MODER_MODER9           (9UL)
#define GPIO_MODER_MODER10          (10UL)
#define GPIO_MODER_MODER11          (11UL)
#define GPIO_MODER_MODER12          (12UL)
#define GPIO_MODER_MODER13          (13UL)
#define GPIO_MODER_MODER14          (14UL)
#define GPIO_MODER_MODER15          (15UL)

/* @GPIO_MODER_MODER */
#define GPIO_MODER_MODER_IP         (0UL)
#define GPIO_MODER_MODER_OP         (1UL)
#define GPIO_MODER_MODER_AF         (2UL)
#define GPIO_MODER_MODER_AN         (3UL)

/* @GPIO_OTYPER */
#define GPIO_OTYPER_OT0              (0UL)
#define GPIO_OTYPER_OT1              (1UL)
#define GPIO_OTYPER_OT2              (2UL)
#define GPIO_OTYPER_OT3              (3UL)
#define GPIO_OTYPER_OT4              (4UL)
#define GPIO_OTYPER_OT5              (5UL)
#define GPIO_OTYPER_OT6              (6UL)
#define GPIO_OTYPER_OT7              (7UL)
#define GPIO_OTYPER_OT8              (8UL)
#define GPIO_OTYPER_OT9              (9UL)
#define GPIO_OTYPER_OT10            (10UL)
#define GPIO_OTYPER_OT11            (11UL)
#define GPIO_OTYPER_OT12            (12UL)
#define GPIO_OTYPER_OT13            (13UL)
#define GPIO_OTYPER_OT14            (14UL)
#define GPIO_OTYPER_OT15            (15UL)

/* @GPIO_OTYPER_OT */
#define GPIO_OTYPER_OT_LOW          (0UL)
#define GPIO_OTYPER_OT_MED          (1UL)
#define GPIO_OTYPER_OT_FAST         (2UL)
#define GPIO_OTYPER_OT_HIGH         (3UL)

/* AHB2 Peripheral base addresses */

/* AHB3 Peripheral base addresses */

/* APB1 Peripheral base addresses */


/**********************************/
/* APB2 Peripheral base addresses */
/**********************************/
#define SPI1_BASE_ADDR              ((APB2_BASE_ADDR) + 0x3000)
#define SPI4_BASE_ADDR              ((APB2_BASE_ADDR) + 0x3400)
#define EXTI_BASE_ADDR              ((APB2_BASE_ADDR) + 0x3C00)
#define SYSCFG_BASE_ADDR            ((APB2_BASE_ADDR) + 0x3800)





#endif
