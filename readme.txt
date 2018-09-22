Description
===========

    This repos contains the porting code for popular MCUs. It's based on Ayla MCU
demo code and will keep upgrade to follow Ayla releases. User can design their own
projects by referring on this code to speed up the development.

Structure
=========

    [root]
    +---MCU_driver_porting
    |   |
    |   +---1.2                         Ayla MCU demo v1.2
    |   |
    |   +---1.7                         Ayla MCU demo v1.7
    |   |
    |   +---common                      Common functions.
    |   |                               User can build it by Visual Stuido.
    |   +---img_pkg                     OTA image generater.
    |   |
    |   +---KSDK                        MKL16Z128 support files from NXP. Not included.
    |   |
    |   +---STM32F0xx_StdPeriph_Driver  STM32F0xx support files from ST. Not included.
    |   |
    |   +---STM32F30x_StdPeriph_Driver  STM32f30x support files from ST. Not included.
    |
    +---SEGGER_RTT                      RTT(Real Time Transfer for embedded targets)
                                        from SEGGER. It's optional. User can mask it by
                                        removing pre-define symbol RTT in project option.
                                        Not included.

    Note: Please build projects using Keil 5.


Build projects
==============

    Ayla MCU demo code v1.2
    -----------------------

    MKL16Z128
    loader  [root]\MCU_driver_porting\1.2\loader.uvprojx
    demo    [root]\MCU_driver_porting\1.2\demo.uvprojx

    Ayla MCU demo code v1.7
    -----------------------

    STM32F303
    loader  [root]\MCU_driver_porting\1.7\proj\stm32f3\keil\loader@stm32f3.uvproj
    demo    [root]\MCU_driver_porting\1.7\proj\stm32f3\keil\demo@stm32f3.uvproj

    STM32F072
    loader  [root]\MCU_driver_porting\1.7\proj\stm32f3\keil\loader@stm32f0.uvproj
    demo    [root]\MCU_driver_porting\1.7\proj\stm32f3\keil\demo@stm32f0.uvproj

    MKL16Z128
    loader  [root]\MCU_driver_porting\1.7\proj\stm32f3\keil\loader@mkl16z.uvproj
    demo    [root]\MCU_driver_porting\1.7\proj\stm32f3\keil\demo@mkl16z.uvproj

Hardware connection
===================

      Ayla     BM-09A     STM32F3Discovery   STM32F303 STM32F072 MKL16Z128
    ----------------------------------------------------------------------
    SPI_SSN  SPI_SSN(22)  MICRO_SPI_SSN(35)    PB12      PA15      PD4
    SPI_SCK  SPI_SCK(24)  MICRO_SPI_SCK(36)    PB13      PB3       PD5
    SPI_MISO SPI_MISO(25) MICRO_SPI_MISO(37)   PB14      PB4       PD7
    SPI_MOSI SPI_MOSI(23) MICRO_SPI_MOSI(38)   PB15      PB5       PD6
    INTR_N   GPIO_4(4)    MICRO_GPIO_4(34)     PB11      PB6       PC1
    READY_N  GPIO_5(3)    MICRO_GPIO_5(33)     PB10      PB7       PC4
    RESET_N  RST_N(26)    MICRO_RST_N(21)      PB0       PB8       PC7
    WAKEUP   WKUP(27)     MICRO_WKUP(23)       PB2       PB9       PB0
    GND      GND(45)      GND(49~52)

    Note: The test enviroment is built by MCU and Wi-Fi module. The Wi-Fi module
          is come from Ayla Devkit. The MCUs are come from all kinds of develop
          boards from MCU companys as below.

    STM32F303 - STM32F3discovery(MB1035B)
    STM32F072 - STM32F0discovery(MB1076C)
    MKL16Z128 - FRDM-KL26Z
