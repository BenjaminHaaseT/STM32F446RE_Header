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


/*************************************/
/* core processor register addresses */
/*************************************/
#define NVIC_ISER_BASE_ADDR         (0xE000E100UL)
#define NVIC_IPR_BASE_ADDR          (0xE000E400UL)


/************************************/
/* core processor register pointers */
/************************************/
#define NVIC_ISER                   ((volatile uint32_t *)NVIC_ISER_BASE_ADDR)
#define NVIC_IPR                    ((volatile uint8_t *)NVIC_IPR_BASE_ADDR)


/***********************************/
/* core processor macros and types */
/***********************************/
/* @NVIC_IPR_IPREN */
#define NVIC_IPR_IPREN              (4UL)

/* @IRQn_t */
typedef uint8_t IRQn_t;

#define WWDG_IRQn                           ((IRQn_t)0)
#define PVD_IRQn                            ((IRQn_t)1)
#define TAMP_STAMP_IRQn                     ((IRQn_t)2)
#define RTC_WKUP_IRQn                       ((IRQn_t)3)
#define FLASH_IRQn                          ((IRQn_t)4)
#define RCC_IRQn                            ((IRQn_t)5)
#define EXTI0_IRQn                          ((IRQn_t)6)
#define EXTI1_IRQn                          ((IRQn_t)7)
#define EXTI2_IRQn                          ((IRQn_t)8)
#define EXTI3_IRQn                          ((IRQn_t)9)
#define EXTI4_IRQn                          ((IRQn_t)10)
#define DMA1_Stream0_IRQn                   ((IRQn_t)11)
#define DMA1_Stream1_IRQn                   ((IRQn_t)12)
#define DMA1_Stream2_IRQn                   ((IRQn_t)13)
#define DMA1_Stream3_IRQn                   ((IRQn_t)14)
#define DMA1_Stream4_IRQn                   ((IRQn_t)15)
#define DMA1_Stream5_IRQn                   ((IRQn_t)16)
#define DMA1_Stream6_IRQn                   ((IRQn_t)17)
#define ADC_IRQn                            ((IRQn_t)18)
#define CAN1_TX_IRQn                        ((IRQn_t)19)
#define CAN1_RX0_IRQn                       ((IRQn_t)20)
#define CAN1_RX1_IRQn                       ((IRQn_t)21)
#define CAN1_SCE_IRQn                       ((IRQn_t)22)
#define EXTI9_5_IRQn                        ((IRQn_t)23)
#define TIM1_BRK_TIM9_IRQn                  ((IRQn_t)24)
#define TIM1_UP_TIM10_IRQn                  ((IRQn_t)25)
#define TIM1_TRG_COM_TIM11_IRQn             ((IRQn_t)26)
#define TIM1_CC_IRQn                        ((IRQn_t)27)
#define TIM2_IRQn                           ((IRQn_t)28)
#define TIM3_IRQn                           ((IRQn_t)29)
#define TIM4_IRQn                           ((IRQn_t)30)
#define I2C1_EV_IRQn                        ((IRQn_t)31)
#define I2C1_ER_IRQn                        ((IRQn_t)32)
#define I2C2_EV_IRQn                        ((IRQn_t)33)
#define I2C2_ER_IRQn                        ((IRQn_t)34)
#define SPI1_IRQn                           ((IRQn_t)35)
#define SPI2_IRQn                           ((IRQn_t)36)
#define USART1_IRQn                         ((IRQn_t)37)
#define USART2_IRQn                         ((IRQn_t)38)
#define USART3_IRQn                         ((IRQn_t)39)
#define EXTI15_10_IRQn                      ((IRQn_t)40)
#define RTC_Alarm_IRQn                      ((IRQn_t)41)
#define OT_FS_WKUP_IRQn                     ((IRQn_t)42)
#define TIM8_BRK_TIM12_IRQn                 ((IRQn_t)43)
#define TIM8_UP_TIM13_IRQn                  ((IRQn_t)44)
#define TIM8_TRG_COM_TIM14_IRQn             ((IRQn_t)45)
#define TIM8_CC_IRQn                        ((IRQn_t)46)
#define DMA1_Stream7_IRQn                   ((IRQn_t)47)
#define FMC_IRQn                            ((IRQn_t)48)
#define SDIO_IRQn                           ((IRQn_t)49)
#define TIM5_IRQn                           ((IRQn_t)50)
#define SPI3_IRQn                           ((IRQn_t)51)
#define UART4_IRQn                          ((IRQn_t)52)
#define UART5_IRQn                          ((IRQn_t)53)
#define TIM6_DAC_IRQn                       ((IRQn_t)54)
#define TIM7_IRQn                           ((IRQn_t)55)
#define DMA2_Stream0_IRQn                   ((IRQn_t)56)
#define DMA2_Stream1_IRQn                   ((IRQn_t)57)
#define DMA2_Stream2_IRQn                   ((IRQn_t)58)
#define DMA2_Stream3_IRQn                   ((IRQn_t)59)
#define DMA2_Stream4_IRQn                   ((IRQn_t)60)
#define CAN2_TX_IRQn                        ((IRQn_t)63)
#define CAN2_RX0_IRQn                       ((IRQn_t)64)
#define CAN2_RX1_IRQn                       ((IRQn_t)65)
#define CAN2_SCE_IRQn                       ((IRQn_t)66)
#define OTG_FS_IRQn                         ((IRQn_t)67)
#define DMA2_Stream5_IRQn                   ((IRQn_t)68)
#define DMA2_Stream6_IRQn                   ((IRQn_t)69)
#define DMA2_Stream7_IRQn                   ((IRQn_t)70)
#define USART6_IRQn                         ((IRQn_t)71)
#define I2C3_EV_IRQn                        ((IRQn_t)72)
#define I2C3_ER_IRQn                        ((IRQn_t)73)
#define OTG_HS_EP1_OUT_IRQn                 ((IRQn_t)74)
#define OTG_HS_EP1_IN_IRQn                  ((IRQn_t)75)
#define OTG_HS_WKUP_IRQn                    ((IRQn_t)76)
#define OTG_HS_IRQn                         ((IRQn_t)77)
#define DCMI_IRQn                           ((IRQn_t)78)
#define FPU_IRQn                            ((IRQn_t)81)
#define SPI4_IRQn                           ((IRQn_t)84)
#define SAI1_IRQn                           ((IRQn_t)87)
#define SAI2_IRQn                           ((IRQn_t)91)
#define QuadSPI_IRQn                        ((IRQn_t)92)
#define HDMI_CEC_IRQn                       ((IRQn_t)93)
#define SPDIF_RX_IRQn                       ((IRQn_t)94)
#define FMPI2C1_IRQn                        ((IRQn_t)95)
#define FMPIWC1_ERR_IRQn                    ((IRQn_t)96)


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
    volatile uint32_t OSPEEDR;
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
#define RCC             ((RCC_regdef_t *const)RCC_BASE_ADDR)
#define GPIOA           ((GPIO_regdef_t *const)GPIOA_BASE_ADDR)
#define GPIOB           ((GPIO_regdef_t *const)GPIOB_BASE_ADDR)
#define GPIOC           ((GPIO_regdef_t *const)GPIOC_BASE_ADDR)
#define GPIOD           ((GPIO_regdef_t *const)GPIOD_BASE_ADDR)
#define GPIOE           ((GPIO_regdef_t *const)GPIOE_BASE_ADDR)
#define GPIOF           ((GPIO_regdef_t *const)GPIOF_BASE_ADDR)
#define GPIOG           ((GPIO_regdef_t *const)GPIOG_BASE_ADDR)
#define GPIOH           ((GPIO_regdef_t *const)GPIOH_BASE_ADDR)

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
#define GPIO_MODER_MODER1           (2UL)
#define GPIO_MODER_MODER2           (4UL)
#define GPIO_MODER_MODER3           (6UL)
#define GPIO_MODER_MODER4           (8UL)
#define GPIO_MODER_MODER5           (10UL)
#define GPIO_MODER_MODER6           (12UL)
#define GPIO_MODER_MODER7           (14UL)
#define GPIO_MODER_MODER8           (16UL)
#define GPIO_MODER_MODER9           (18UL)
#define GPIO_MODER_MODER10          (20UL)
#define GPIO_MODER_MODER11          (22UL)
#define GPIO_MODER_MODER12          (24UL)
#define GPIO_MODER_MODER13          (26UL)
#define GPIO_MODER_MODER14          (28UL)
#define GPIO_MODER_MODER15          (30UL)

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
#define GPIO_OTYPER_OT_PP			(0UL)
#define GPIO_OTYPER_OT_OD			(1UL)

/* @GPIO_OSPEEDR */
#define GPIO_OSPEEDR_OSPEEDR0		(0UL)
#define GPIO_OSPEEDR_OSPEEDR1		(2UL)
#define GPIO_OSPEEDR_OSPEEDR2		(4UL)
#define GPIO_OSPEEDR_OSPEEDR3		(6UL)
#define GPIO_OSPEEDR_OSPEEDR4		(8UL)
#define GPIO_OSPEEDR_OSPEEDR5		(10UL)
#define GPIO_OSPEEDR_OSPEEDR6		(12UL)
#define GPIO_OSPEEDR_OSPEEDR7		(14UL)
#define GPIO_OSPEEDR_OSPEEDR8		(16UL)
#define GPIO_OSPEEDR_OSPEEDR9		(18UL)
#define GPIO_OSPEEDR_OSPEEDR10		(20UL)
#define GPIO_OSPEEDR_OSPEEDR11		(22UL)
#define GPIO_OSPEEDR_OSPEEDR12		(24UL)
#define GPIO_OSPEEDR_OSPEEDR13		(26UL)
#define GPIO_OSPEEDR_OSPEEDR14		(28UL)
#define GPIO_OSPEEDR_OSPEEDR15		(30UL)

/* @GPIO_OSPEEDR_OSPEEDR */
#define GPIO_OSPEEDR_OSPEEDR_LOW          (0UL)
#define GPIO_OSPEEDR_OSPEEDR_MED          (1UL)
#define GPIO_OSPEEDR_OSPEEDR_FAST         (2UL)
#define GPIO_OSPEEDR_OSPEEDRT_HIGH         (3UL)

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

/* @GPIO_PUPDR_PUPDR */
#define GPIO_PUPDR_PUPDR_NONE		(0x0UL)
#define GPIO_PUPDR_PUPDR_PU			(0x1UL)
#define GPIO_PUPDR_PUPDR_PD			(0x2UL)

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

/* @GPIO_AFR */
#define GPIO_AFR_AFR0				((0 % 8) * 4)
#define GPIO_AFR_AFR1				((1 % 8) * 4)
#define GPIO_AFR_AFR2				((2 % 8) * 4)
#define GPIO_AFR_AFR3				((3 % 8) * 4)
#define GPIO_AFR_AFR4				((4 % 8) * 4)
#define GPIO_AFR_AFR5				((5 % 8) * 4)
#define GPIO_AFR_AFR6				((6 % 8) * 4)
#define GPIO_AFR_AFR7				((7 % 8) * 4)
#define GPIO_AFR_AFR8				((8 % 8) * 4)
#define GPIO_AFR_AFR9				((9 % 8) * 4)
#define GPIO_AFR_AFR10				((10 % 8) * 4)
#define GPIO_AFR_AFR11				((11 % 8) * 4)
#define GPIO_AFR_AFR12				((12 % 8) * 4)
#define GPIO_AFR_AFR13				((13 % 8) * 4)
#define GPIO_AFR_AFR14				((14 % 8) * 4)
#define GPIO_AFR_AFR15				((15 % 8) * 4)

/* @GPIO_AFR_AFR */
#define GPIO_AFR_AFR_AF0			(0UL)
#define GPIO_AFR_AFR_AF1			(1UL)
#define GPIO_AFR_AFR_AF2			(2UL)
#define GPIO_AFR_AFR_AF3			(3UL)
#define GPIO_AFR_AFR_AF4			(4UL)
#define GPIO_AFR_AFR_AF5			(5UL)
#define GPIO_AFR_AFR_AF6			(6UL)
#define GPIO_AFR_AFR_AF7			(7UL)
#define GPIO_AFR_AFR_AF8			(8UL)
#define GPIO_AFR_AFR_AF9			(9UL)
#define GPIO_AFR_AFR_AF10			(10UL)
#define GPIO_AFR_AFR_AF11			(11UL)
#define GPIO_AFR_AFR_AF12			(12UL)
#define GPIO_AFR_AFR_AF13			(13UL)
#define GPIO_AFR_AFR_AF14			(14UL)
#define GPIO_AFR_AFR_AF15			(15UL)


/* AHB2 Peripheral base addresses */

/* AHB3 Peripheral base addresses */


/**********************************/
/* APB1 Peripheral base addresses */
/**********************************/
#define TIM2_BASE_ADDR              ((APB1_BASE_ADDR) + 0x0)
#define TIM3_BASE_ADDR				((APB1_BASE_ADDR) + 0x0400)
#define TIM4_BASE_ADDR				((APB1_BASE_ADDR) + 0x0800)
#define TIM5_BASE_ADDR              ((APB1_BASE_ADDR) + 0x0C00)


/***************************************/
/* APB1 peripheral register structures */
/***************************************/
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t _RESERVED1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t _RESERVED2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
    volatile uint32_t OR;
} TIM2X5_regdef_t;


/***********************************************/
/* AHB1 register structure pointer definitions */
/***********************************************/
#define TIM2					((TIM2X5_regdef_t *)TIM2_BASE_ADDR)
#define TIM3					((TIM2X5_regdef_t *)TIM3_BASE_ADDR)
#define TIM4					((TIM2X5_regdef_t *)TIM4_BASE_ADDR)
#define TIM5					((TIM2X5_regdef_t *)TIM5_BASE_ADDR)

/**************************/
/* AHB1 peripheral macros */
/**************************/
/* @TIM2X5_CR1 */
#define TIM2X5_CR1_CEN              (0UL)
#define TIM2X5_CR1_UDIS             (1UL)
#define TIM2X5_CR1_URS              (2UL)
#define TIM2X5_CR1_OPM              (3UL)
#define TIM2X5_CR1_DIR              (4UL)
#define TIM2X5_CR1_CMS              (5UL)
#define TIM2X5_CR1_ARPE             (7UL)
#define TIM2X5_CR1_CKD              (8UL)

/* @TIM2X5_CR2 */
#define TIM2X5_CR2_CCDS             (3UL)
#define TIM2X5_CR2_MMS              (4UL)
#define TIM2X5_CR2_TI1S             (7UL)

/* @TIM2X5_SR */
#define TIM2X5_SR_UIF               (0UL)
#define TIM2X5_SR_CC1IF             (1UL)
#define TIM2X5_SR_CC2IF             (2UL)
#define TIM2X5_SR_CC3IF             (3UL)
#define TIM2X5_SR_CC4IF             (4UL)
#define TIM2X5_SR_TIF               (6UL)
#define TIM2X5_SR_CC10F             (9UL)
#define TIM2X5_SR_CC20F             (10UL)
#define TIM2X5_SR_CC30F             (11UL)
#define TIM2X5_SR_CC40F             (12UL)

/* @TIM2X5_CCMR1 */
#define TIM2X5_CCMR1_CC1S           (0UL)
#define TIM2X5_CCMR1_OC1FE          (2UL)
#define TIM2X5_CCMR1_OC1PE          (3UL)
#define TIM2X5_CCMR1_OC1M           (4UL)
#define TIM2X5_CCMR1_OC1CE          (7UL)
#define TIM2X5_CCMR1_CC2S           (8UL)
#define TIM2X5_CCMR1_OC2FE          (10UL)
#define TIM2X5_CCMR1_OC2PE          (11UL)
#define TIM2X5_CCMR1_OC2M           (12UL)
#define TIM2X5_CCMR1_OC2CE          (15UL)
#define TIM2X5_CCMR1_IC1PSC         (2UL)
#define TIM2X5_CCMR1_IC1F           (4UL)
#define TIM2X5_CCMR1_IC2PSC         (10UL)
#define TIM2X5_CCMR1_IC2F           (12UL)

/* @TIM2X5_CCMR1_CC1S */
#define TIM2X5_CCMR1_CC1S_OP         (0x0UL)
#define TIM2X5_CCMR1_CC1S_IP_TI1     (0x1UL)
#define TIM2X5_CCMR1_CC1S_IP_TI2     (0x2UL)
#define TIM2X5_CCMR1_CC1S_IP_TRC     (0x3UL)

/* @TIM2X5_CCMR1_OC1M */
#define TIM2X5_CCMR1_OC1M_FROZEN     (0x0UL)
#define TIM2X5_CCMR1_OC1M_SET        (0x1UL)
#define TIM2X5_CCMR1_OC1M_CLEAR      (0x2UL)
#define TIM2X5_CCMR1_OC1M_TOGGLE     (0x3UL)
#define TIM2X5_CCMR1_OC1M_FRC_CLEAR  (0x4UL)
#define TIM2X5_CCMR1_OC1M_FRC_SET    (0x5UL)
#define TIM2X5_CCMR1_OC1M_PWM_MODE1  (0x6UL)
#define TIM2X5_CCMR1_OC1M_PWM_MODE2  (0x7UL)

/* @TIM2X5_CCMR1_CC2S */
#define TIM2X5_CCMR1_CC2S_OP         (0x0UL)
#define TIM2X5_CCMR1_CC2S_IP_TI1     (0x1UL)
#define TIM2X5_CCMR1_CC2S_IP_TI2     (0x2UL)
#define TIM2X5_CCMR1_CC2S_IP_TRC     (0x3UL)

/* @TIM2X5_CCMR1_OC2M */
#define TIM2X5_CCMR1_OC2M_FROZEN     (0x0UL)
#define TIM2X5_CCMR1_OC2M_SET        (0x1UL)
#define TIM2X5_CCMR1_OC2M_CLEAR      (0x2UL)
#define TIM2X5_CCMR1_OC2M_TOGGLE     (0x3UL)
#define TIM2X5_CCMR1_OC2M_FRC_CLEAR  (0x4UL)
#define TIM2X5_CCMR1_OC2M_FRC_SET    (0x5UL)
#define TIM2X5_CCMR1_OC2M_PWM_MODE1  (0x6UL)
#define TIM2X5_CCMR1_OC2M_PWM_MODE2  (0x7UL)

/* @TIM2X5_CCMR1_IC1PSC */
#define TIM2X5_CCMR1_IC1PSC_NONE     (0UL)
#define TIM2X5_CCMR1_IC1PSC_2        (1UL)
#define TIM2X5_CCMR1_IC1PSC_4        (2UL)
#define TIM2X5_CCMR1_IC1PSC_8        (3UL)

/* @TIM2X5_CCMR1_IC1F */
#define TIM2X5_CCMR1_IC1F_NONE          (0UL)
#define TIM2X5_CCMR1_IC1F_N2            (1UL)
#define TIM2X5_CCMR1_IC1F_N4            (2UL)
#define TIM2X5_CCMR1_IC1F_N8            (3UL)
#define TIM2X5_CCMR1_IC1F_DIV2N6        (4UL)
#define TIM2X5_CCMR1_IC1F_DIV2N8        (5UL)
#define TIM2X5_CCMR1_IC1F_DIV4N6        (6UL)
#define TIM2X5_CCMR1_IC1F_DIV4N8        (7UL)
#define TIM2X5_CCMR1_IC1F_DIV8N6        (8UL)
#define TIM2X5_CCMR1_IC1F_DIV8N8        (9UL)
#define TIM2X5_CCMR1_IC1F_DIV16N5       (10UL)
#define TIM2X5_CCMR1_IC1F_DIV16N6       (11UL)
#define TIM2X5_CCMR1_IC1F_DIV16N8       (12UL)
#define TIM2X5_CCMR1_IC1F_DIV32N5       (13UL)
#define TIM2X5_CCMR1_IC1F_DIV32N6       (14UL)
#define TIM2X5_CCMR1_IC1F_DIV32N8       (15UL)

/* @TIM2X5_CCMR1_IC2PSC */
#define TIM2X5_CCMR1_IC2PSC_NONE     (0UL)
#define TIM2X5_CCMR1_IC2PSC_2        (1UL)
#define TIM2X5_CCMR1_IC2PSC_4        (2UL)
#define TIM2X5_CCMR1_IC2PSC_8        (3UL)

/* @TIM2X5_CCMR1_IC2F */
#define TIM2X5_CCMR1_IC2F_NONE          (0UL)
#define TIM2X5_CCMR1_IC2F_N2            (1UL)
#define TIM2X5_CCMR1_IC2F_N4            (2UL)
#define TIM2X5_CCMR1_IC2F_N8            (3UL)
#define TIM2X5_CCMR1_IC2F_DIV2N6        (4UL)
#define TIM2X5_CCMR1_IC2F_DIV2N8        (5UL)
#define TIM2X5_CCMR1_IC2F_DIV4N6        (6UL)
#define TIM2X5_CCMR1_IC2F_DIV4N8        (7UL)
#define TIM2X5_CCMR1_IC2F_DIV8N6        (8UL)
#define TIM2X5_CCMR1_IC2F_DIV8N8        (9UL)
#define TIM2X5_CCMR1_IC2F_DIV16N5       (10UL)
#define TIM2X5_CCMR1_IC2F_DIV16N6       (11UL)
#define TIM2X5_CCMR1_IC2F_DIV16N8       (12UL)
#define TIM2X5_CCMR1_IC2F_DIV32N5       (13UL)
#define TIM2X5_CCMR1_IC2F_DIV32N6       (14UL)
#define TIM2X5_CCMR1_IC2F_DIV32N8       (15UL)

/* @TIM2X5_CCMR2 */
#define TIM2X5_CCMR2_CC3S           (0UL)
#define TIM2X5_CCMR2_OC3FE          (2UL)
#define TIM2X5_CCMR2_OC3PE          (3UL)
#define TIM2X5_CCMR2_OC3M           (4UL)
#define TIM2X5_CCMR2_OC3CE          (7UL)
#define TIM2X5_CCMR2_CC4S           (8UL)
#define TIM2X5_CCMR2_OC4FE          (10UL)
#define TIM2X5_CCMR2_OC4PE          (11UL)
#define TIM2X5_CCMR2_OC4M           (12UL)
#define TIM2X5_CCMR2_OC4CE          (15UL)
#define TIM2X5_CCMR2_IC3PSC         (2UL)
#define TIM2X5_CCMR2_IC3F           (4UL)
#define TIM2X5_CCMR2_IC4PSC         (10UL)
#define TIM2X5_CCMR2_IC4F           (12UL)

/* @TIM2X5_CCMR2_CC3S */
#define TIM2X5_CCR2_CC3S_OP         (0x0UL)
#define TIM2X5_CCR2_CC3S_IP_TI3     (0x1UL)
#define TIM2X5_CCR2_CC3S_IP_TI4     (0x2UL)
#define TIM2X5_CCR2_CC3S_IP_TRC     (0x3UL)

/* @TIM2X5_CCMR2_OC3M */
#define TIM2X5_CCMR2_OC3M_FROZEN     (0x0UL)
#define TIM2X5_CCMR2_OC3M_SET        (0x1UL)
#define TIM2X5_CCMR2_OC3M_CLEAR      (0x2UL)
#define TIM2X5_CCMR2_OC3M_TOGGLE     (0x3UL)
#define TIM2X5_CCMR2_OC3M_FRC_CLEAR  (0x4UL)
#define TIM2X5_CCMR2_OC3M_FRC_SET    (0x5UL)
#define TIM2X5_CCMR2_OC3M_PWM_MODE1  (0x6UL)
#define TIM2X5_CCMR2_OC3M_PWM_MODE2  (0x7UL)

/* @TIM2X5_CCMR2_CC4S */
#define TIM2X5_CCMR2_CC4S_OP         (0x0UL)
#define TIM2X5_CCMR2_CC4S_IP_TI4     (0x1UL)
#define TIM2X5_CCMR2_CC4S_IP_TI3     (0x2UL)
#define TIM2X5_CCMR2_CC4S_IP_TRC     (0x3UL)

/* @TIM2X5_CCMR2_OC4M */
#define TIM2X5_CCMR2_OC4M_FROZEN     (0x0UL)
#define TIM2X5_CCMR2_OC4M_SET        (0x1UL)
#define TIM2X5_CCMR2_OC4M_CLEAR      (0x2UL)
#define TIM2X5_CCMR2_OC4M_TOGGLE     (0x3UL)
#define TIM2X5_CCMR2_OC4M_FRC_CLEAR  (0x4UL)
#define TIM2X5_CCMR2_OC4M_FRC_SET    (0x5UL)
#define TIM2X5_CCMR2_OC4M_PWM_MODE1  (0x6UL)
#define TIM2X5_CCMR2_OC4M_PWM_MODE2  (0x7UL)

/* @TIM2X5_CCR2_IC3PSC */
#define TIM2X5_CCR2_IC3PSC_NONE     (0UL)
#define TIM2X5_CCR2_IC3PSC_2        (1UL)
#define TIM2X5_CCR2_IC3PSC_4        (2UL)
#define TIM2X5_CCR2_IC3PSC_8        (3UL)

/* @TIM2X5_CCMR2_IC1F */
#define TIM2X5_CCMR2_IC3F_NONE          (0UL)
#define TIM2X5_CCMR2_IC3F_N2            (1UL)
#define TIM2X5_CCMR2_IC3F_N4            (2UL)
#define TIM2X5_CCMR2_IC3F_N8            (3UL)
#define TIM2X5_CCMR2_IC3F_DIV2N6        (4UL)
#define TIM2X5_CCMR2_IC3F_DIV2N8        (5UL)
#define TIM2X5_CCMR2_IC3F_DIV4N6        (6UL)
#define TIM2X5_CCMR2_IC3F_DIV4N8        (7UL)
#define TIM2X5_CCMR2_IC3F_DIV8N6        (8UL)
#define TIM2X5_CCMR2_IC3F_DIV8N8        (9UL)
#define TIM2X5_CCMR2_IC3F_DIV16N5       (10UL)
#define TIM2X5_CCMR2_IC3F_DIV16N6       (11UL)
#define TIM2X5_CCMR2_IC3F_DIV16N8       (12UL)
#define TIM2X5_CCMR2_IC3F_DIV32N5       (13UL)
#define TIM2X5_CCMR2_IC3F_DIV32N6       (14UL)
#define TIM2X5_CCMR2_IC3F_DIV32N8       (15UL)

/* @TIM2X5_CCMR2_IC4PSC */
#define TIM2X5_CCMR2_IC4PSC_NONE     (0UL)
#define TIM2X5_CCMR2_IC4PSC_2        (1UL)
#define TIM2X5_CCMR2_IC4PSC_4        (2UL)
#define TIM2X5_CCMR2_IC4PSC_8        (3UL)

/* @TIM2X5_CCMR2_IC4F */
#define TIM2X5_CCMR2_IC4F_NONE          (0UL)
#define TIM2X5_CCMR2_IC4F_N2            (1UL)
#define TIM2X5_CCMR2_IC4F_N4            (2UL)
#define TIM2X5_CCMR2_IC4F_N8            (3UL)
#define TIM2X5_CCMR2_IC4F_DIV2N6        (4UL)
#define TIM2X5_CCMR2_IC4F_DIV2N8        (5UL)
#define TIM2X5_CCMR2_IC4F_DIV4N6        (6UL)
#define TIM2X5_CCMR2_IC4F_DIV4N8        (7UL)
#define TIM2X5_CCMR2_IC4F_DIV8N6        (8UL)
#define TIM2X5_CCMR2_IC4F_DIV8N8        (9UL)
#define TIM2X5_CCMR2_IC4F_DIV16N5       (10UL)
#define TIM2X5_CCMR2_IC4F_DIV16N6       (11UL)
#define TIM2X5_CCMR2_IC4F_DIV16N8       (12UL)
#define TIM2X5_CCMR2_IC4F_DIV32N5       (13UL)
#define TIM2X5_CCMR2_IC4F_DIV32N6       (14UL)
#define TIM2X5_CCMR2_IC4F_DIV32N8       (15UL)

/* @TIM2X5_CCER */
#define TIM2X5_CCER_CC1E                 (0UL)
#define TIM2X5_CCER_CC1P                 (1UL)
#define TIM2X5_CCER_CC1NP                (3UL)
#define TIM2X5_CCER_CC2E                (4UL)
#define TIM2X5_CCER_CC2P                (5UL)
#define TIM2X5_CCER_CC2NP               (7UL)
#define TIM2X5_CCER_CC3E                (8UL)
#define TIM2X5_CCER_CC3P                (9UL)
#define TIM2X5_CCER_CC3NP               (11UL)
#define TIM2X5_CCER_CC4E                (12UL)
#define TIM2X5_CCER_CC4P                (13UL)
#define TIM2X5_CCER_CC4NP               (15UL)

/* @TIM2X5_TIM2_OR */
#define TIM2X5_TIM2_OR_ITR1_RMP          (10UL)

/* @TIM2X5_TIM5_OR */
#define TIM2X5_TIM2_OR_TI4_RMP           (6UL)


/**********************************/
/* APB2 Peripheral base addresses */
/**********************************/
#define SPI1_BASE_ADDR              ((APB2_BASE_ADDR) + 0x3000)
#define SPI4_BASE_ADDR              ((APB2_BASE_ADDR) + 0x3400)
#define EXTI_BASE_ADDR              ((APB2_BASE_ADDR) + 0x3C00)
#define SYSCFG_BASE_ADDR            ((APB2_BASE_ADDR) + 0x3800)


/****************************/
/* APB2 register structures */
/****************************/
typedef struct
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    volatile uint32_t CMPCR;
    volatile uint32_t CFGR;
} SYSCFG_regdef_t;


typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_regdef_t;


/***********************************************/
/* AHB2 register structure pointer definitions */
/***********************************************/
#define SYSCFG                  ((SYSCFG_regdef_t *)SYSCFG_BASE_ADDR)
#define EXTI                    ((EXTI_regdef_t *)EXTI_BASE_ADDR)


/**************************/
/* AHB2 peripheral macros */
/**************************/
/* @SYSCFG_EXTICR_IDX */
#define SYSCFG_EXTICR_IDX0          (0 / 4)
#define SYSCFG_EXTICR_IDX1          (1 / 4)
#define SYSCFG_EXTICR_IDX2          (2 / 4)
#define SYSCFG_EXTICR_IDX3          (3 / 4)
#define SYSCFG_EXTICR_IDX4          (4 / 4)
#define SYSCFG_EXTICR_IDX5          (5 / 4)
#define SYSCFG_EXTICR_IDX6          (6 / 4)
#define SYSCFG_EXTICR_IDX7          (7 / 4)
#define SYSCFG_EXTICR_IDX8          (8 / 4)
#define SYSCFG_EXTICR_IDX9          (9 / 4)
#define SYSCFG_EXTICR_IDX10         (10 / 4)
#define SYSCFG_EXTICR_IDX11         (11 / 4)
#define SYSCFG_EXTICR_IDX12         (12 / 4)
#define SYSCFG_EXTICR_IDX13         (13 / 4)
#define SYSCFG_EXTICR_IDX14         (14 / 4)
#define SYSCFG_EXTICR_IDX15         (15 / 4)

/* @SYSCFG_EXTICR */
#define SYSCFG_EXTICR_EXTI0         ((0 % 4) * 4)
#define SYSCFG_EXTICR_EXTI1         ((1 % 4) * 4)
#define SYSCFG_EXTICR_EXTI2         ((2 % 4) * 4)
#define SYSCFG_EXTICR_EXTI3         ((3 % 4) * 4)
#define SYSCFG_EXTICR_EXTI4         ((4 % 4) * 4)
#define SYSCFG_EXTICR_EXTI5         ((5 % 4) * 4)
#define SYSCFG_EXTICR_EXTI6         ((6 % 4) * 4)
#define SYSCFG_EXTICR_EXTI7         ((7 % 4) * 4)
#define SYSCFG_EXTICR_EXTI8         ((8 % 4) * 4)
#define SYSCFG_EXTICR_EXTI9         ((9 % 4) * 4)
#define SYSCFG_EXTICR_EXTI10        ((10 % 4) * 4)
#define SYSCFG_EXTICR_EXTI11        ((11 % 4) * 4)
#define SYSCFG_EXTICR_EXTI12        ((12 % 4) * 4)
#define SYSCFG_EXTICR_EXTI13        ((13 % 4) * 4)
#define SYSCFG_EXTICR_EXTI14        ((14 % 4) * 4)
#define SYSCFG_EXTICR_EXTI15        ((15 % 4) * 4)

/* @SYSCFG_EXTICR_EXTI */
#define SYSCFG_EXTICR_EXTI_PA           (0UL)
#define SYSCFG_EXTICR_EXTI_PB           (1UL)
#define SYSCFG_EXTICR_EXTI_PC           (2UL)
#define SYSCFG_EXTICR_EXTI_PD           (3UL)
#define SYSCFG_EXTICR_EXTI_PE           (4UL)
#define SYSCFG_EXTICR_EXTI_PF           (5UL)
#define SYSCFG_EXTICR_EXTI_PG           (6UL)
#define SYSCFG_EXTICR_EXTI_PH           (7UL)

/* @EXTI_IMR */
#define EXTI_IMR_MR0                    (0UL)
#define EXTI_IMR_MR1                    (1UL)
#define EXTI_IMR_MR2                    (2UL)
#define EXTI_IMR_MR3                    (3UL)
#define EXTI_IMR_MR4                    (4UL)
#define EXTI_IMR_MR5                    (5UL)
#define EXTI_IMR_MR6                    (6UL)
#define EXTI_IMR_MR7                    (7UL)
#define EXTI_IMR_MR8                    (8UL)
#define EXTI_IMR_MR9                    (9UL)
#define EXTI_IMR_MR10                   (10UL)
#define EXTI_IMR_MR11                   (11UL)
#define EXTI_IMR_MR12                   (12UL)
#define EXTI_IMR_MR13                   (13UL)
#define EXTI_IMR_MR14                   (14UL)
#define EXTI_IMR_MR15                   (15UL)

/* @EXTI_EMR */
#define EXTI_EMR_MR0                    (0UL)
#define EXTI_EMR_MR1                    (1UL)
#define EXTI_EMR_MR2                    (2UL)
#define EXTI_EMR_MR3                    (3UL)
#define EXTI_EMR_MR4                    (4UL)
#define EXTI_EMR_MR5                    (5UL)
#define EXTI_EMR_MR6                    (6UL)
#define EXTI_EMR_MR7                    (7UL)
#define EXTI_EMR_MR8                    (8UL)
#define EXTI_EMR_MR9                    (9UL)
#define EXTI_EMR_MR10                   (10UL)
#define EXTI_EMR_MR11                   (11UL)
#define EXTI_EMR_MR12                   (12UL)
#define EXTI_EMR_MR13                   (13UL)
#define EXTI_EMR_MR14                   (14UL)
#define EXTI_EMR_MR15                   (15UL)

/* @EXTI_RTSR */
#define EXTI_RTSR_TR0                   (0UL)
#define EXTI_RTSR_TR1                   (1UL)
#define EXTI_RTSR_TR2                   (2UL)
#define EXTI_RTSR_TR3                   (3UL)
#define EXTI_RTSR_TR4                   (4UL)
#define EXTI_RTSR_TR5                   (5UL)
#define EXTI_RTSR_TR6                   (6UL)
#define EXTI_RTSR_TR7                   (7UL)
#define EXTI_RTSR_TR8                   (8UL)
#define EXTI_RTSR_TR9                   (9UL)
#define EXTI_RTSR_TR10                  (10UL)
#define EXTI_RTSR_TR11                  (11UL)
#define EXTI_RTSR_TR12                  (12UL)
#define EXTI_RTSR_TR13                  (13UL)
#define EXTI_RTSR_TR14                  (14UL)
#define EXTI_RTSR_TR15                  (15UL)
#define EXTI_RTSR_TR16                  (16UL)
#define EXTI_RTSR_TR17                  (17UL)
#define EXTI_RTSR_TR18                  (18UL)
#define EXTI_RTSR_TR20                  (20UL)
#define EXTI_RTSR_TR21                  (21UL)
#define EXTI_RTSR_TR22                  (22UL)

/* @EXTI_FTSR */
#define EXTI_FTSR_TR0                   (0UL)
#define EXTI_FTSR_TR1                   (1UL)
#define EXTI_FTSR_TR2                   (2UL)
#define EXTI_FTSR_TR3                   (3UL)
#define EXTI_FTSR_TR4                   (4UL)
#define EXTI_FTSR_TR5                   (5UL)
#define EXTI_FTSR_TR6                   (6UL)
#define EXTI_FTSR_TR7                   (7UL)
#define EXTI_FTSR_TR8                   (8UL)
#define EXTI_FTSR_TR9                   (9UL)
#define EXTI_FTSR_TR10                  (10UL)
#define EXTI_FTSR_TR11                  (11UL)
#define EXTI_FTSR_TR12                  (12UL)
#define EXTI_FTSR_TR13                  (13UL)
#define EXTI_FTSR_TR14                  (14UL)
#define EXTI_FTSR_TR15                  (15UL)
#define EXTI_FTSR_TR16                  (16UL)
#define EXTI_FTSR_TR17                  (17UL)
#define EXTI_FTSR_TR18                  (18UL)
#define EXTI_FTSR_TR20                  (20UL)
#define EXTI_FTSR_TR21                  (21UL)
#define EXTI_FTSR_TR22                  (22UL)

/* @EXTI_SWIER */
#define EXTI_SWIER_SWIER0               (0UL)
#define EXTI_SWIER_SWIER1               (1UL)
#define EXTI_SWIER_SWIER2               (2UL)
#define EXTI_SWIER_SWIER3               (3UL)
#define EXTI_SWIER_SWIER4               (4UL)
#define EXTI_SWIER_SWIER5               (5UL)
#define EXTI_SWIER_SWIER6               (6UL)
#define EXTI_SWIER_SWIER7               (7UL)
#define EXTI_SWIER_SWIER8               (8UL)
#define EXTI_SWIER_SWIER9               (9UL)
#define EXTI_SWIER_SWIER10              (10UL)
#define EXTI_SWIER_SWIER11              (11UL)
#define EXTI_SWIER_SWIER12              (12UL)
#define EXTI_SWIER_SWIER13              (13UL)
#define EXTI_SWIER_SWIER14              (14UL)
#define EXTI_SWIER_SWIER15              (15UL)
#define EXTI_SWIER_SWIER16              (16UL)
#define EXTI_SWIER_SWIER17              (17UL)
#define EXTI_SWIER_SWIER18              (18UL)
#define EXTI_SWIER_SWIER19              (19UL)
#define EXTI_SWIER_SWIER20              (20UL)
#define EXTI_SWIER_SWIER21              (21UL)
#define EXTI_SWIER_SWIER22              (22UL)

/* @EXTI_PR */
#define EXTI_PR_PR0               (0UL)
#define EXTI_PR_PR1               (1UL)
#define EXTI_PR_PR2               (2UL)
#define EXTI_PR_PR3               (3UL)
#define EXTI_PR_PR4               (4UL)
#define EXTI_PR_PR5               (5UL)
#define EXTI_PR_PR6               (6UL)
#define EXTI_PR_PR7               (7UL)
#define EXTI_PR_PR8               (8UL)
#define EXTI_PR_PR9               (9UL)
#define EXTI_PR_PR10              (10UL)
#define EXTI_PR_PR11              (11UL)
#define EXTI_PR_PR12              (12UL)
#define EXTI_PR_PR13              (13UL)
#define EXTI_PR_PR14              (14UL)
#define EXTI_PR_PR15              (15UL)
#define EXTI_PR_PR16              (16UL)
#define EXTI_PR_PR17              (17UL)
#define EXTI_PR_PR18              (18UL)
#define EXTI_PR_PR19              (19UL)
#define EXTI_PR_PR20              (20UL)
#define EXTI_PR_PR21              (21UL)
#define EXTI_PR_PR22              (22UL)

#define SET 			(1UL)

#endif        // STM32F446XX_H
