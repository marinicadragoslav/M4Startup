/*
 * stm32f407.h
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

#ifndef STM32F407_H
#define STM32F407_H

#include <stdint.h>

/* memory base addresses */
#define FLASH_BASEADDR        0x08000000U
#define SRAM1_BASEADDR        0x20000000U                   /* 112 Kb (112 * 1024 = 0x1C000 bytes in hex) */
#define SRAM2_BASEADDR        (SRAM1_BASEADDR + 0x1C000U)   /* follows SRAM1, 16 Kb */
#define ROM_BASEADDR          0x1FFF0000U                   /* system memory, 30 Kb */
#define SRAM                  SRAM1_BASEADDR
#define ROM                   ROM_BASEADDR

/* AHBx and APBx peripheral base addresses */
#define APB1PERIPH_BASEADDR   0x40000000U                   /* APB1 Bus base addr */
#define APB2PERIPH_BASEADDR   0x40010000U                   /* APB2 Bus base addr */
#define AHB1PERIPH_BASEADDR   0x40020000U                   /* AHB1 Bus base addr */
#define AHB2PERIPH_BASEADDR   0x50000000U                   /* AHB2 Bus base addr */
#define PERIPH_BASEADDR       APB1PERIPH_BASEADDR

/* base addresses of peripherals hanging on AHB1 bus */
#define GPIOA_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR        (AHB1PERIPH_BASEADDR + 0x2000U)

/* RCC base address - also on AHB1*/
#define RCC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x3800U)
#define FLASH_IF_BASEADDR     (AHB1PERIPH_BASEADDR + 0x3C00U)

/* base addresses of peripherals hanging on APB1 bus */
#define I2C1_BASEADDR         (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR         (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR         (APB1PERIPH_BASEADDR + 0x5C00U)
#define SPI2_BASEADDR         (APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR         (APB1PERIPH_BASEADDR + 0x3C00U)
#define USART2_BASEADDR       (APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR       (APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR        (APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR        (APB1PERIPH_BASEADDR + 0x5000U)

/* base addresses of peripherals hanging on APB2 bus */
#define SPI1_BASEADDR         (APB2PERIPH_BASEADDR + 0x3000U)
#define USART1_BASEADDR       (APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR       (APB2PERIPH_BASEADDR + 0x1400U)
#define EXTI_BASEADDR         (APB2PERIPH_BASEADDR + 0x3C00U)
#define SYSCFG_BASEADDR       (APB2PERIPH_BASEADDR + 0x3800U)

/* NVIC ISERx base addresses */
#define NVIC_ISER0            ((volatile uint32_t*) 0xE000E100U)
#define NVIC_ISER1            ((volatile uint32_t*) 0xE000E104U)
#define NVIC_ISER2            ((volatile uint32_t*) 0xE000E108U)
#define NVIC_ISER3            ((volatile uint32_t*) 0xE000E10CU)

/* NVIC ICERx base addresses */
#define NVIC_ICER0            ((volatile uint32_t*) 0xE000E180U)
#define NVIC_ICER1            ((volatile uint32_t*) 0xE000E184U)
#define NVIC_ICER2            ((volatile uint32_t*) 0xE000E188U)
#define NVIC_ICER3            ((volatile uint32_t*) 0xE000E18CU)

/* base addresses for system control registers */
#define SCS_BASE              (0xE000E000UL)
#define SCB_BASE              (SCS_BASE +  0x0D00U)
#define SYSTICK_BASE          (SCS_BASE +  0x0010U)

/* NVIC Priority registers base addresses */
#define NVIC_PRI_BASE_ADDR    ((volatile uint32_t *) 0xE000E400)

/* NVIC - number of priority bits (MSB) implemented for each of the four 8-bit sections of any NVIC_IPRx register */
#define NVIC_PRIO_BITS        4U

/* IRQ Priority: MCU specific, size given by NVIC_PRIO_BIT. Lower number = Higher priority */
#define NVIC_IRQ_PRIO0        0U
#define NVIC_IRQ_PRIO1        1U
#define NVIC_IRQ_PRIO2        2U
#define NVIC_IRQ_PRIO3        3U
#define NVIC_IRQ_PRIO4        4U
#define NVIC_IRQ_PRIO5        5U
#define NVIC_IRQ_PRIO6        6U
#define NVIC_IRQ_PRIO7        7U
#define NVIC_IRQ_PRIO8        8U
#define NVIC_IRQ_PRIO9        9U
#define NVIC_IRQ_PRIO10       10U
#define NVIC_IRQ_PRIO11       11U
#define NVIC_IRQ_PRIO12       12U
#define NVIC_IRQ_PRIO13       13U
#define NVIC_IRQ_PRIO14       14U
#define NVIC_IRQ_PRIO15       15U
#define NVIC_IRQ_INVALID_PRIO 16U /* used for checking valid prio values */

/* IRQ_Numbers */
#define IRQ_NUM_SYSTICK       (-1)
#define IRQ_NUM_EXTI_0        6U
#define IRQ_NUM_EXTI_1        7U
#define IRQ_NUM_EXTI_2        8U
#define IRQ_NUM_EXTI_3        9U
#define IRQ_NUM_EXTI_4        10U
#define IRQ_NUM_EXTI_5_9      23U
#define IRQ_NUM_EXTI_10_15    40U

/* structure defining the registers of SysTick */
typedef struct
{
  volatile uint32_t CTRL;      /* Offset: 0x000 (R/W)  SysTick Control and Status Register */
  volatile uint32_t LOAD;      /* Offset: 0x004 (R/W)  SysTick Reload Value Register */
  volatile uint32_t VAL;       /* Offset: 0x008 (R/W)  SysTick Current Value Register */
  volatile uint32_t CALIB;     /* Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_RegDef_t;

/* define a pointer to the systick registers area */
#define SYSTICK               ((SysTick_RegDef_t*)SYSTICK_BASE)

/* structure defining the registers of a GPIO peripheral */
typedef struct
{
   volatile uint32_t MODER;      /* GPIO port mode register (GPIOx_MODER).                   Offset 0x00.         */
   volatile uint32_t OTYPER;     /* GPIO port output type register (GPIOx_OTYPER).           Offset 0x04.         */
   volatile uint32_t OSPEEDR;    /* GPIO port output speed register (GPIOx_OSPEEDR).         Offset 0x08.         */
   volatile uint32_t PUPDR;      /* GPIO port pull-up/pull-down register (GPIOx_PUPDR).      Offset 0x0C.         */
   volatile uint32_t IDR;        /* GPIO port input data register (GPIOx_IDR).               Offset 0x10.         */
   volatile uint32_t ODR;        /* GPIO port output data register (GPIOx_ODR).              Offset 0x14.         */
   volatile uint32_t BSRR;       /* GPIO port bit set/reset register (GPIOx_BSRR).           Offset 0x18.         */
   volatile uint32_t LCKR;       /* GPIO port configuration lock register (GPIOx_LCKR).      Offset 0x1C.         */
   volatile uint32_t AFR[2];     /* GPIO alternate function reg (GPIOx_AFRL + GPIOx_AFRH).   Offset 0x20 - 0x24.  */
}GPIO_PortRegDef_t;

/* GPIO_Ports */
#define GPIOA                 ((GPIO_PortRegDef_t*) GPIOA_BASEADDR)
#define GPIOB                 ((GPIO_PortRegDef_t*) GPIOB_BASEADDR)
#define GPIOC                 ((GPIO_PortRegDef_t*) GPIOC_BASEADDR)
#define GPIOD                 ((GPIO_PortRegDef_t*) GPIOD_BASEADDR)
#define GPIOE                 ((GPIO_PortRegDef_t*) GPIOE_BASEADDR)
#define GPIOF                 ((GPIO_PortRegDef_t*) GPIOF_BASEADDR)
#define GPIOG                 ((GPIO_PortRegDef_t*) GPIOG_BASEADDR)
#define GPIOH                 ((GPIO_PortRegDef_t*) GPIOH_BASEADDR)
#define GPIOI                 ((GPIO_PortRegDef_t*) GPIOI_BASEADDR)

/* structure defining the registers of an SPI peripheral */
typedef struct
{
   volatile uint32_t CR1;        /* TODO */
   volatile uint32_t CR2;        /* TODO */
   volatile uint32_t SR;         /* TODO */
   volatile uint32_t DR;         /* TODO */
   volatile uint32_t CRCPR;      /* TODO */
   volatile uint32_t RXCRCR;     /* TODO */
   volatile uint32_t TXCRCR;     /* TODO */
   volatile uint32_t I2SCFGR;    /* TODO */
   volatile uint32_t I2SPR;      /* TODO */
}SPI_RegDef_t;

/* define pointers to the SPI peripherals */
#define SPI1                  ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2                  ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3                  ((SPI_RegDef_t*) SPI3_BASEADDR)

/* structure defining the registers of the RCC */
typedef struct
{
   volatile uint32_t CR;         /* RCC clock control register (RCC_CR).                            Offset 0x00.  */
   volatile uint32_t PLLCFGR;    /* RCC PLL configuration register  (RCC_PLLCFGR).                  Offset 0x04.  */
   volatile uint32_t CFGR;       /* RCC clock configuration register (RCC_CFGR).                    Offset 0x08.  */
   volatile uint32_t CIR;        /* RCC clock interrupt register (RCC_CIR).                         Offset 0x0C.  */
   volatile uint32_t AHB1RSTR;   /* RCC AHB1 peripheral reset register (RCC_AHB1RSTR).              Offset 0x10.  */
   volatile uint32_t AHB2RSTR;   /* RCC AHB2 peripheral reset register (RCC_AHB2RSTR).              Offset 0x14.  */
   volatile uint32_t AHB3RSTR;   /* RCC AHB3 peripheral reset register (RCC_AHB3RSTR).              Offset 0x18.  */
   uint32_t RESERVED0;           /* reserved, 0x1C;                                                               */
   volatile uint32_t APB1RSTR;   /* RCC APB1 peripheral reset register (RCC_APB1RSTR).              Offset 0x20.  */
   volatile uint32_t APB2RSTR;   /* RCC APB2 peripheral reset register (RCC_APB2RSTR).              Offset 0x24.  */
   uint32_t RESERVED1[2];        /* reserved, 0x28, 0x2C;                                                         */
   volatile uint32_t AHB1ENR;    /* RCC AHB1 peripheral clock enable register (RCC_AHB1ENR).        Offset 0x30.  */
   volatile uint32_t AHB2ENR;    /* RCC AHB2 peripheral clock enable register (RCC_AHB2ENR).        Offset 0x34.  */
   volatile uint32_t AHB3ENR;    /* RCC AHB3 peripheral clock enable register (RCC_AHB3ENR).        Offset 0x38.  */
   uint32_t RESERVED2;           /* reserved, 0x3C;                                                               */
   volatile uint32_t APB1ENR;    /* RCC APB1 peripheral clock enable register (RCC_APB1ENR).        Offset 0x40.  */
   volatile uint32_t APB2ENR;    /* RCC APB2 peripheral clock enable register (RCC_APB2ENR).        Offset 0x44.  */
   uint32_t RESERVED3[2];        /* reserved, 0x48, 0x4C;                                                         */
   volatile uint32_t AHB1LPENR;  /* RCC AHB1 periph clock enable in low power mode (RCC_AHB1LPENR). Offset 0x50.  */
   volatile uint32_t AHB2LPENR;  /* RCC AHB2 periph clock enable in low power mode (RCC_AHB2LPENR). Offset 0x54.  */
   volatile uint32_t AHB3LPENR;  /* RCC AHB3 periph clock enable in low power mode (RCC_AHB3LPENR). Offset 0x58.  */
   uint32_t RESERVED4;           /* reserved, 0x5C;                                                               */
   volatile uint32_t APB1LPENR;  /* RCC APB1 periph clock enable in low power mode (RCC_APB1LPENR). Offset 0x60.  */
   volatile uint32_t APB2LPENR;  /* RCC APB2 periph clock enable in low power mode (RCC_APB2LPENR). Offset 0x64.  */
   uint32_t RESERVED5[2];        /* reserved, 0x68, 0x6C;                                                         */
   volatile uint32_t BDCR;       /* RCC Backup domain control register (RCC_BDCR).                  Offset 0x70.  */
   volatile uint32_t CSR;        /* RCC clock control & status register (RCC_CSR).                  Offset 0x74.  */
   uint32_t RESERVED6[2];        /* reserved, 0x78, 0x7C;                                                         */
   volatile uint32_t SSCGR;      /* RCC spread spectrum clock generation register (RCC_SSCGR).      Offset 0x80.  */
   volatile uint32_t PLLI2SCFGR; /* RCC PLLI2S configuration register (RCC_PLLI2SCFGR).             Offset 0x84.  */
   volatile uint32_t PLLSAICFGR; /* RCC PLLSAI configuration register (RCC_PLLSAICFGR).             Offset 0x88.  */
   volatile uint32_t DCKCFGR;    /* RRCC Dedicated Clock Configuration Register (RCC_DCKCFGR).      Offset 0x8C.  */
}RCC_RegDef_t;

/* define a pointer of type to the RCC */
#define RCC                   ((RCC_RegDef_t*) RCC_BASEADDR)

/* structure defining the registers of the EXTI */
typedef struct
{
   volatile uint32_t IMR;         /* Interrupt mask register (EXTI_IMR)                Offset 0x00.  */
   volatile uint32_t EMR;         /* Event mask register (EXTI_EMR)                    Offset 0x04.  */
   volatile uint32_t RTSR;        /* Rising trigger selection register (EXTI_RTSR)     Offset 0x08.  */
   volatile uint32_t FTSR;        /* Falling trigger selection register (EXTI_FTSR)    Offset 0x0C.  */
   volatile uint32_t SWIER;       /* Software interrupt event register (EXTI_SWIER)    Offset 0x10.  */
   volatile uint32_t PR;          /* Pending register (EXTI_PR)                        Offset 0x14.  */
}EXTI_RegDef_t;

/* define a pointer to EXTI */
#define EXTI                  ((EXTI_RegDef_t*) EXTI_BASEADDR)

/* structure defining the registers of the Flash Interface*/
typedef struct
{
  volatile uint32_t ACR;      /* FLASH access control register,   offset: 0x00 */
  volatile uint32_t KEYR;     /* FLASH key register,              offset: 0x04 */
  volatile uint32_t OPTKEYR;  /* FLASH option key register,       offset: 0x08 */
  volatile uint32_t SR;       /* FLASH status register,           offset: 0x0C */
  volatile uint32_t CR;       /* FLASH control register,          offset: 0x10 */
  volatile uint32_t OPTCR;    /* FLASH option control register,   offset: 0x14 */
  volatile uint32_t OPTCR1;   /* FLASH option control register 1, offset: 0x18 */
} FLASH_IF_RegDef_t;

/* Define a pointer to Flash Interface registers */
#define FLASH_IF              ((FLASH_IF_RegDef_t*)FLASH_IF_BASEADDR)

/* structure defining the registers of the SYSCFG */
typedef struct
{
   volatile uint32_t MEMRMP;     /* SYSCFG memory remap register (SYSCFG_MEMRMP).                   Offset 0x00.  */
   volatile uint32_t PMC;        /* SYSCFG peripheral mode configuration register (SYSCFG_PMC).     Offset 0x04.  */
   volatile uint32_t EXTICR1;    /* SYSCFG external interrupt configuration reg 1 (SYSCFG_EXTICR1). Offset 0x08.  */
   volatile uint32_t EXTICR2;    /* SYSCFG external interrupt configuration reg 2 (SYSCFG_EXTICR2). Offset 0x0C.  */
   volatile uint32_t EXTICR3;    /* SYSCFG external interrupt configuration reg 3 (SYSCFG_EXTICR3). Offset 0x10.  */
   volatile uint32_t EXTICR4;    /* SYSCFG external interrupt configuration reg 4 (SYSCFG_EXTICR4). Offset 0x14.  */
   uint32_t RESERVED0[2];        /* reserved, 0x18, 0x1C;                                                         */
   volatile uint32_t CMPCR;      /* Compensation cell control register (SYSCFG_CMPCR).              Offset 0x20.  */
}SYSCFG_RegDef_t;

/* define a pointer to SYSCFG */
#define SYSCFG                ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/* structure defining the registers of System Control Block (SCB). */
typedef struct
{
   volatile uint32_t CPUID;       /* Offset: 0x000 (R/ )  CPUID Base Register */
   volatile uint32_t ICSR;        /* Offset: 0x004 (R/W)  Interrupt Control and State Register */
   volatile uint32_t VTOR;        /* Offset: 0x008 (R/W)  Vector Table Offset Register */
   volatile uint32_t AIRCR;       /* Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
   volatile uint32_t SCR;         /* Offset: 0x010 (R/W)  System Control Register */
   volatile uint32_t CCR;         /* Offset: 0x014 (R/W)  Configuration Control Register */
   volatile uint8_t  SHP[12U];    /* Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
   volatile uint32_t SHCSR;       /* Offset: 0x024 (R/W)  System Handler Control and State Register */
   volatile uint32_t CFSR;        /* Offset: 0x028 (R/W)  Configurable Fault Status Register */
   volatile uint32_t HFSR;        /* Offset: 0x02C (R/W)  HardFault Status Register */
   volatile uint32_t DFSR;        /* Offset: 0x030 (R/W)  Debug Fault Status Register */
   volatile uint32_t MMFAR;       /* Offset: 0x034 (R/W)  MemManage Fault Address Register */
   volatile uint32_t BFAR;        /* Offset: 0x038 (R/W)  BusFault Address Register */
   volatile uint32_t AFSR;        /* Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
   volatile uint32_t PFR[2U];     /* Offset: 0x040 (R/ )  Processor Feature Register */
   volatile uint32_t DFR;         /* Offset: 0x048 (R/ )  Debug Feature Register */
   volatile uint32_t ADR;         /* Offset: 0x04C (R/ )  Auxiliary Feature Register */
   volatile uint32_t MMFR[4U];    /* Offset: 0x050 (R/ )  Memory Model Feature Register */
   volatile uint32_t ISAR[5U];    /* Offset: 0x060 (R/ )  Instruction Set Attributes Register */
   uint32_t RESERVED0[5U];
   volatile uint32_t CPACR;       /* Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_RegDef_t;

/* define a pointer to the SCB register area */
#define SCB                 ((SCB_RegDef_t*)SCB_BASE)

/* macros to enable the clock for I2Cx */
#define I2C1_PCLK_EN()        (RCC->APB1ENR |= (1U << 21))
#define I2C2_PCLK_EN()        (RCC->APB1ENR |= (1U << 22))
#define I2C3_PCLK_EN()        (RCC->APB1ENR |= (1U << 23))

/* macros to enable the clock for SPIx */
#define SPI1_PCLK_EN()        (RCC->APB2ENR |= (1U << 12))
#define SPI2_PCLK_EN()        (RCC->APB1ENR |= (1U << 14))
#define SPI3_PCLK_EN()        (RCC->APB1ENR |= (1U << 15))
#define SPI4_PCLK_EN()        (RCC->APB1ENR |= (1U << 13))

/* macros to enable the clock for USARTx/UARTx */
#define USART1_PCLK_EN()      (RCC->APB2ENR |= (1U << 4))
#define USART2_PCLK_EN()      (RCC->APB1ENR |= (1U << 17))
#define USART3_PCLK_EN()      (RCC->APB1ENR |= (1U << 18))
#define UART4_PCLK_EN()       (RCC->APB1ENR |= (1U << 19))
#define UART5_PCLK_EN()       (RCC->APB1ENR |= (1U << 20))
#define USART6_PCLK_EN()      (RCC->APB2ENR |= (1U << 5))

/* macro to enable the clock for SYSCFG */
#define SYSCFG_PCLK_EN()      (RCC->APB2ENR |= (1U << 14))

/* macros to disable the clock for I2Cx */
#define I2C1_PCLK_DI()        (RCC->APB1ENR &= ~(1U << 21))
#define I2C2_PCLK_DI()        (RCC->APB1ENR &= ~(1U << 22))
#define I2C3_PCLK_DI()        (RCC->APB1ENR &= ~(1U << 23))

/* macros to disable the clock for SPIx */
#define SPI1_PCLK_DI()        (RCC->APB2ENR &= ~(1U << 12))
#define SPI2_PCLK_DI()        (RCC->APB1ENR &= ~(1U << 14))
#define SPI3_PCLK_DI()        (RCC->APB1ENR &= ~(1U << 15))
#define SPI4_PCLK_DI()        (RCC->APB1ENR &= ~(1U << 13))

/* macros to disable the clock for USARTx/UARTx */
#define USART1_PCLK_DI()      (RCC->APB2ENR &= ~(1U << 4))
#define USART2_PCLK_DI()      (RCC->APB1ENR &= ~(1U << 17))
#define USART3_PCLK_DI()      (RCC->APB1ENR &= ~(1U << 18))
#define UART4_PCLK_DI()       (RCC->APB1ENR &= ~(1U << 19))
#define UART5_PCLK_DI()       (RCC->APB1ENR &= ~(1U << 20))
#define USART6_PCLK_DI()      (RCC->APB2ENR &= ~(1U << 5))

/* macro to disable the clock for SYSCFG */
#define SYSCFG_PCLK_DI()      (RCC->APB2ENR &= ~(1U << 14))

/* register-related definitions */
#define SYSTICK_RELOAD_SHIFT           0U
#define SYSTICK_RELOAD_MASK            (0xFFFFFFU << SYSTICK_RELOAD_SHIFT)
#define SYSTICK_CTRL_CLKSOURCE_SHIFT   2U
#define SYSTICKCTRL_CLKSOURCE_MASK     (1U << SYSTICK_CTRL_CLKSOURCE_SHIFT)
#define SYSTICK_CTRL_TICKINT_SHIFT     1U 
#define SYSTICK_CTRL_TICKINT_MASK      (1UL << SYSTICK_CTRL_TICKINT_SHIFT)
#define SYSTICK_CTRL_ENABLE_SHIFT      0U
#define SYSTICK_CTRL_ENABLE_MASK       (1U << SYSTICK_CTRL_ENABLE_SHIFT)

#define FLASH_ACR_LATENCY_SHIFT        0U
#define FLASH_ACR_LATENCY_MASK         (7U << FLASH_ACR_LATENCY_SHIFT)

#define RCC_CR_HSEON_SHIFT             16U
#define RCC_CR_HSEON_MASK              (1U << RCC_CR_HSEON_SHIFT)
#define RCC_CR_HSERDY_SHIFT            17U
#define RCC_CR_HSERDY_MASK             (1U << RCC_CR_HSERDY_SHIFT)
#define RCC_CR_PLLON_SHIFT             24U
#define RCC_CR_PLLON_MASK              (0x1U << RCC_CR_PLLON_SHIFT)
#define RCC_CR_PLLRDY_SHIFT            25U
#define RCC_CR_PLLRDY_MASK             (0x1U << RCC_CR_PLLRDY_SHIFT)

#define RCC_PLLCFGR_PLLM_SHIFT         0U
#define RCC_PLLCFGR_PLLM_MASK          (0x3FU << RCC_PLLCFGR_PLLM_SHIFT)
#define RCC_PLLCFGR_PLLN_SHIFT         6U
#define RCC_PLLCFGR_PLLN_MASK          (0x1FFU << RCC_PLLCFGR_PLLN_SHIFT)
#define RCC_PLLCFGR_PLLP_SHIFT         16U
#define RCC_PLLCFGR_PLLP_MASK          (0x3U << RCC_PLLCFGR_PLLP_SHIFT)
#define RCC_PLLCFGR_PLLSRC_SHIFT       22U
#define RCC_PLLCFGR_PLLSRC_MASK        (1U << RCC_PLLCFGR_PLLSRC_SHIFT)

#define RCC_CFGR_HPRE_SHIFT            4U
#define RCC_CFGR_HPRE_MSK              (0xFU << RCC_CFGR_HPRE_SHIFT)
#define RCC_CFGR_SW_SHIFT              0U 
#define RCC_CFGR_SW_MASK               (0x3U << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SWS_SHIFT             2U
#define RCC_CFGR_SWS_MASK              (0x3U << RCC_CFGR_SWS_SHIFT)
#define RCC_CFGR_PPRE1_SHIFT           (10U)
#define RCC_CFGR_PPRE1_MASK            (0x7U << RCC_CFGR_PPRE1_SHIFT)
#define RCC_CFGR_PPRE2_SHIFT           (13U)
#define RCC_CFGR_PPRE2_MASK            (0x7U << RCC_CFGR_PPRE2_SHIFT)

#endif /* STM32F407_H */
