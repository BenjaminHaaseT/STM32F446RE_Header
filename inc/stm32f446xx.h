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
    volatile uint32_t AFR[2];
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

/* @RCC_AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN         (0UL)
#define RCC_AHB1ENR_GPIOBEN         (1UL)
#define RCC_AHB1ENR_GPIOCEN         (2UL)
#define RCC_AHB1ENR_GPIODEN         (3UL)
#define RCC_AHB1ENR_GPIOEEN         (4UL)
#define RCC_AHB1ENR_GPIOFEN         (5UL)
#define RCC_AHB1ENR_GPIOGEN         (6UL)
#define RCC_AHB1ENR_GPIOHEN         (7UL)
#define RCC_AHB1ENR_CRCEN           (12UL)
#define RCC_AHB1ENR_BKPSRAMEN       (18UL)
#define RCC_AHB1ENR_DMA1EN          (21UL)
#define RCC_AHB1ENR_DMA2EN          (22UL)
#define RCC_AHB1ENR_OTGHSEN         (29UL)
#define RCC_AHB1ENR_OTGHSULPIEN     (30UL)

/* @RCC_AHB2ENR */
#define RCC_AHB2ENR_DCMIEN          (0UL)
#define RCC_AHB2ENR_OTGFSEN         (7UL)

/* @RCC_AHB3ENR */
#define RCC_AHB3ENR_FMCEN           (0UL)
#define RCC_AHB3ENR_QSPIEN          (1UL)

/* @RCC_APB1ENR */
#define RCC_APB1ENR_TIM2EN           (0UL)
#define RCC_APB1ENR_TIM3EN           (1UL)
#define RCC_APB1ENR_TIM4EN           (2UL)
#define RCC_APB1ENR_TIM5EN           (3UL)
#define RCC_APB1ENR_TIM6EN           (4UL)
#define RCC_APB1ENR_TIM7EN           (5UL)
#define RCC_APB1ENR_TIM12EN          (6UL)
#define RCC_APB1ENR_TIM13EN          (7UL)
#define RCC_APB1ENR_TIM14EN          (8UL)
#define RCC_APB1ENR_WWDGEN           (11UL)
#define RCC_APB1ENR_SPI2EN           (14UL)
#define RCC_APB1ENR_SPI3EN           (15UL)
#define RCC_APB1ENR_SPDIFRXEN        (16UL)
#define RCC_APB1ENR_USART2EN         (17UL)
#define RCC_APB1ENR_USART3EN         (18UL)
#define RCC_APB1ENR_UART4EN          (19UL)
#define RCC_APB1ENR_UART5EN          (20UL)
#define RCC_APB1ENR_I2C1EN           (21UL)
#define RCC_APB1ENR_I2C2EN           (22UL)
#define RCC_APB1ENR_I2C3EN           (23UL)
#define RCC_APB1ENR_FMPI2C1EN        (24UL)
#define RCC_APB1ENR_CAN1EN           (25UL)
#define RCC_APB1ENR_CAN2EN           (26UL)
#define RCC_APB1ENR_CECEN            (27UL)
#define RCC_APB1ENR_PWREN            (28UL)
#define RCC_APB1ENR_DACEN            (29UL)

/* @RCC_APB2ENR */
#define RCC_APB2ENR_TIM1EN           (0UL)
#define RCC_APB2ENR_TIM8EN           (1UL)
#define RCC_APB2ENR_USART1EN         (4UL)
#define RCC_APB2ENR_USART6EN         (5UL)
#define RCC_APB2ENR_ADC1EN           (8UL)
#define RCC_APB2ENR_ADC2EN           (9UL)
#define RCC_APB2ENR_ADC3EN           (10UL)
#define RCC_APB2ENR_SDIOEN           (11UL)
#define RCC_APB2ENR_SPI1EN           (12UL)
#define RCC_APB2ENR_SPI4EN           (13UL)
#define RCC_APB2ENR_SYSCFGEN         (14UL)
#define RCC_APB2ENR_TIM9EN           (16UL)
#define RCC_APB2ENR_TIM10EN          (17UL)
#define RCC_APB2ENR_TIM11EN          (18UL)
#define RCC_APB2ENR_SAI1EN           (22UL)
#define RCC_APB2ENR_SAI2EN           (23UL)

/* @GPIO_PIN */
#define GPIO_PIN0                   (0UL)
#define GPIO_PIN1                   (1UL)
#define GPIO_PIN2                   (2UL)
#define GPIO_PIN3                   (3UL)
#define GPIO_PIN4                   (4UL)
#define GPIO_PIN5                   (5UL)
#define GPIO_PIN6                   (6UL)
#define GPIO_PIN7                   (7UL)
#define GPIO_PIN8                   (8UL)
#define GPIO_PIN9                   (9UL)
#define GPIO_PIN10                  (10UL)
#define GPIO_PIN11                  (11UL)
#define GPIO_PIN12                  (12UL)
#define GPIO_PIN13                  (13UL)
#define GPIO_PIN14                  (14UL)
#define GPIO_PIN15                  (15UL)

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

/* @GPIO_PUPDR */
#define GPIO_PUPDR_PUPDR0           (0UL)
#define GPIO_PUPDR_PUPDR1           (2UL)
#define GPIO_PUPDR_PUPDR2           (4UL)
#define GPIO_PUPDR_PUPDR3           (6UL)
#define GPIO_PUPDR_PUPDR4           (8UL)
#define GPIO_PUPDR_PUPDR5           (10UL)
#define GPIO_PUPDR_PUPDR6           (12UL)
#define GPIO_PUPDR_PUPDR7           (14UL)
#define GPIO_PUPDR_PUPDR8           (16UL)
#define GPIO_PUPDR_PUPDR9           (18UL)
#define GPIO_PUPDR_PUPDR10          (20UL)
#define GPIO_PUPDR_PUPDR11          (22UL)
#define GPIO_PUPDR_PUPDR12          (24UL)
#define GPIO_PUPDR_PUPDR13          (26UL)
#define GPIO_PUPDR_PUPDR14          (28UL)
#define GPIO_PUPDR_PUPDR15          (30UL)

/* @GPIO_IDR */
#define GPIO_IDR_IDR0               (0UL)
#define GPIO_IDR_IDR1               (1UL)
#define GPIO_IDR_IDR2               (2UL)
#define GPIO_IDR_IDR3               (3UL)
#define GPIO_IDR_IDR4               (4UL)
#define GPIO_IDR_IDR5               (5UL)
#define GPIO_IDR_IDR6               (6UL)
#define GPIO_IDR_IDR7               (7UL)
#define GPIO_IDR_IDR8               (8UL)
#define GPIO_IDR_IDR9               (9UL)
#define GPIO_IDR_IDR10              (10UL)
#define GPIO_IDR_IDR11              (11UL)
#define GPIO_IDR_IDR12              (12UL)
#define GPIO_IDR_IDR13              (13UL)
#define GPIO_IDR_IDR14              (14UL)
#define GPIO_IDR_IDR15              (15UL)

/* @GPIO_ODR */
#define GPIO_ODR_ODR0               (0UL)
#define GPIO_ODR_ODR1               (1UL)
#define GPIO_ODR_ODR2               (2UL)
#define GPIO_ODR_ODR3               (3UL)
#define GPIO_ODR_ODR4               (4UL)
#define GPIO_ODR_ODR5               (5UL)
#define GPIO_ODR_ODR6               (6UL)
#define GPIO_ODR_ODR7               (7UL)
#define GPIO_ODR_ODR8               (8UL)
#define GPIO_ODR_ODR9               (9UL)
#define GPIO_ODR_ODR10              (10UL)
#define GPIO_ODR_ODR11              (11UL)
#define GPIO_ODR_ODR12              (12UL)
#define GPIO_ODR_ODR13              (13UL)
#define GPIO_ODR_ODR14              (14UL)
#define GPIO_ODR_ODR15              (15UL)

/* @GPIO_BSRR */
#define GPIO_BSRR_BS0               (0UL)
#define GPIO_BSRR_BS1               (1UL)
#define GPIO_BSRR_BS2               (2UL)
#define GPIO_BSRR_BS3               (3UL)
#define GPIO_BSRR_BS4               (4UL)
#define GPIO_BSRR_BS5               (5UL)
#define GPIO_BSRR_BS6               (6UL)
#define GPIO_BSRR_BS7               (7UL)
#define GPIO_BSRR_BS8               (8UL)
#define GPIO_BSRR_BS9               (9UL)
#define GPIO_BSRR_BS10              (10UL)
#define GPIO_BSRR_BS11              (11UL)
#define GPIO_BSRR_BS12              (12UL)
#define GPIO_BSRR_BS13              (13UL)
#define GPIO_BSRR_BS14              (14UL)
#define GPIO_BSRR_BS15              (15UL)

#define GPIO_BSRR_BR0               (16UL)
#define GPIO_BSRR_BR1               (17UL)
#define GPIO_BSRR_BR2               (18UL)
#define GPIO_BSRR_BR3               (19UL)
#define GPIO_BSRR_BR4               (20UL)
#define GPIO_BSRR_BR5               (21UL)
#define GPIO_BSRR_BR6               (22UL)
#define GPIO_BSRR_BR7               (23UL)
#define GPIO_BSRR_BR8               (24UL)
#define GPIO_BSRR_BR9               (25UL)
#define GPIO_BSRR_BR10              (26UL)
#define GPIO_BSRR_BR11              (27UL)
#define GPIO_BSRR_BR12              (28UL)
#define GPIO_BSRR_BR13              (29UL)
#define GPIO_BSRR_BR14              (30UL)
#define GPIO_BSRR_BR15              (31UL)

/* @GPIO_LCKR */
#define GPIO_LCKR_LCKK0             (0UL)
#define GPIO_LCKR_LCKK1             (1UL)
#define GPIO_LCKR_LCKK2             (2UL)
#define GPIO_LCKR_LCKK3             (3UL)
#define GPIO_LCKR_LCKK4             (4UL)
#define GPIO_LCKR_LCKK5             (5UL)
#define GPIO_LCKR_LCKK6             (6UL)
#define GPIO_LCKR_LCKK7             (7UL)
#define GPIO_LCKR_LCKK8             (8UL)
#define GPIO_LCKR_LCKK9             (9UL)
#define GPIO_LCKR_LCKK10            (10UL)
#define GPIO_LCKR_LCKK11            (11UL)
#define GPIO_LCKR_LCKK12            (12UL)
#define GPIO_LCKR_LCKK13            (13UL)
#define GPIO_LCKR_LCKK14            (14UL)
#define GPIO_LCKR_LCKK15            (15UL)
#define GPIO_LCKR_LCKK16            (16UL)

/* @GPIO_AFR_IDX */
#define GPIO_AFR_IDX0               ((GPIO_PIN0) / 8)
#define GPIO_AFR_IDX1               ((GPIO_PIN1) / 8)
#define GPIO_AFR_IDX2               ((GPIO_PIN2) / 8)
#define GPIO_AFR_IDX3               ((GPIO_PIN3) / 8)
#define GPIO_AFR_IDX4               ((GPIO_PIN4) / 8)
#define GPIO_AFR_IDX5               ((GPIO_PIN5) / 8)
#define GPIO_AFR_IDX6               ((GPIO_PIN6) / 8)
#define GPIO_AFR_IDX7               ((GPIO_PIN7) / 8)
#define GPIO_AFR_IDX8               ((GPIO_PIN8) / 8)
#define GPIO_AFR_IDX9               ((GPIO_PIN9) / 8)
#define GPIO_AFR_IDX10              ((GPIO_PIN10) / 8)
#define GPIO_AFR_IDX11              ((GPIO_PIN11) / 8)
#define GPIO_AFR_IDX12              ((GPIO_PIN12) / 8)
#define GPIO_AFR_IDX13              ((GPIO_PIN13) / 8)
#define GPIO_AFR_IDX14              ((GPIO_PIN14) / 8)
#define GPIO_AFR_IDX15              ((GPIO_PIN15) / 8)

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
