
const stm32_tim_pin_list_type chip_tim1 [] = {
    { TIM1  ,GPIOA, GPIO_PIN_6  ,TIM_BKIN   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_12 ,TIM_BKIN   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_15 ,TIM_BKIN   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_8  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_9  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_7  ,TIM_CH1N   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_13 ,TIM_CH1N   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_8  ,TIM_CH1N   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_9  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_11 ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOB, GPIO_PIN_0  ,TIM_CH2N   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_14 ,TIM_CH2N   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_10 ,TIM_CH2N   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_10 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_13 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOB, GPIO_PIN_1  ,TIM_CH3N   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_15 ,TIM_CH3N   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_12 ,TIM_CH3N   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_11 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_14 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_12 ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_7  ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
}; 

const stm32_tim_pin_list_type chip_tim2 [] = {
    { TIM2  ,GPIOA, GPIO_PIN_0  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOA, GPIO_PIN_15 ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1}, 
    { TIM2  ,GPIOA, GPIO_PIN_1  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOB, GPIO_PIN_3  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1}, 
    { TIM2  ,GPIOA, GPIO_PIN_2  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOB, GPIO_PIN_10 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_2}, 
    { TIM2  ,GPIOA, GPIO_PIN_3  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOB, GPIO_PIN_11 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_2}, 
    { TIM2  ,GPIOA, GPIO_PIN_0  ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOA, GPIO_PIN_15 ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1}, 
}; 

const stm32_tim_pin_list_type chip_tim3 [] = {
    { TIM3  ,GPIOA, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_4  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM3_PARTIAL}, 
    { TIM3  ,GPIOC, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOA, GPIO_PIN_7  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_5  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM3_PARTIAL}, 
    { TIM3  ,GPIOC, GPIO_PIN_7  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_0  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOC, GPIO_PIN_8  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_1  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOC, GPIO_PIN_9  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOD, GPIO_PIN_2  ,TIM_ETR    , AF_NO_REMAP    }, 
}; 

const stm32_tim_pin_list_type chip_tim4 [] = {
    { TIM4  ,GPIOB, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_12 ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOB, GPIO_PIN_7  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_13 ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOB, GPIO_PIN_8  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_14 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOB, GPIO_PIN_9  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_15 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOE, GPIO_PIN_0  ,TIM_ETR    , AF_NO_REMAP    }, 
}; 

const stm32_tim_pin_list_type chip_tim5 [] = {
}; 

const stm32_tim_pin_list_type chip_tim6 [] = {
}; 

const stm32_tim_pin_list_type chip_tim7 [] = {
}; 

const stm32_tim_pin_list_type chip_tim8 [] = {
}; 

const stm32_tim_pin_list_type chip_tim9 [] = {
}; 

const stm32_tim_pin_list_type chip_tim10 [] = {
}; 

const stm32_tim_pin_list_type chip_tim11 [] = {
}; 

const stm32_tim_pin_list_type chip_tim12 [] = {
}; 

const stm32_tim_pin_list_type chip_tim13 [] = {
}; 

const stm32_tim_pin_list_type chip_tim14 [] = {
}; 

const stm32_tim_pin_list_type chip_tim15 [] = {
    { TIM15 ,GPIOA, GPIO_PIN_9  ,TIM_BKIN   , AF_NO_REMAP    }, 
    { TIM15 ,GPIOA, GPIO_PIN_2  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM15_DISABLE}, 
    { TIM15 ,GPIOB, GPIO_PIN_14 ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM15_ENABLE}, 
    { TIM15 ,GPIOB, GPIO_PIN_15 ,TIM_CH1N   , AF_NO_REMAP    }, 
    { TIM15 ,GPIOA, GPIO_PIN_3  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM15_DISABLE}, 
    { TIM15 ,GPIOB, GPIO_PIN_15 ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM15_ENABLE}, 
}; 

const stm32_tim_pin_list_type chip_tim16 [] = {
    { TIM16 ,GPIOB, GPIO_PIN_5  ,TIM_BKIN   , AF_NO_REMAP    }, 
    { TIM16 ,GPIOA, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM16_ENABLE}, 
    { TIM16 ,GPIOB, GPIO_PIN_8  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM16_DISABLE}, 
    { TIM16 ,GPIOB, GPIO_PIN_6  ,TIM_CH1N   , AF_NO_REMAP    }, 
}; 

const stm32_tim_pin_list_type chip_tim17 [] = {
    { TIM17 ,GPIOA, GPIO_PIN_10 ,TIM_BKIN   , AF_NO_REMAP    }, 
    { TIM17 ,GPIOA, GPIO_PIN_7  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM17_ENABLE}, 
    { TIM17 ,GPIOB, GPIO_PIN_9  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM17_DISABLE}, 
    { TIM17 ,GPIOB, GPIO_PIN_7  ,TIM_CH1N   , AF_NO_REMAP    }, 
}; 

const stm32_tim_pin_list_type chip_tim18 [] = {
}; 

const stm32_tim_pin_list_type chip_tim [] = {
    { TIM1  ,GPIOA, GPIO_PIN_6  ,TIM_BKIN   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_12 ,TIM_BKIN   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_15 ,TIM_BKIN   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_8  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_9  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_7  ,TIM_CH1N   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_13 ,TIM_CH1N   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_8  ,TIM_CH1N   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_9  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_11 ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOB, GPIO_PIN_0  ,TIM_CH2N   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_14 ,TIM_CH2N   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_10 ,TIM_CH2N   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_10 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_13 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOB, GPIO_PIN_1  ,TIM_CH3N   , AF__HAL_AFIO_REMAP_TIM1_PARTIAL}, 
    { TIM1  ,GPIOB, GPIO_PIN_15 ,TIM_CH3N   , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_12 ,TIM_CH3N   , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_11 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_14 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM1  ,GPIOA, GPIO_PIN_12 ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM1_DISABLE}, 
    { TIM1  ,GPIOE, GPIO_PIN_7  ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM1_ENABLE}, 
    { TIM2  ,GPIOA, GPIO_PIN_0  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOA, GPIO_PIN_15 ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1}, 
    { TIM2  ,GPIOA, GPIO_PIN_1  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOB, GPIO_PIN_3  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1}, 
    { TIM2  ,GPIOA, GPIO_PIN_2  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOB, GPIO_PIN_10 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_2}, 
    { TIM2  ,GPIOA, GPIO_PIN_3  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOB, GPIO_PIN_11 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_2}, 
    { TIM2  ,GPIOA, GPIO_PIN_0  ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM2_DISABLE}, 
    { TIM2  ,GPIOA, GPIO_PIN_15 ,TIM_ETR    , AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1}, 
    { TIM3  ,GPIOA, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_4  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM3_PARTIAL}, 
    { TIM3  ,GPIOC, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOA, GPIO_PIN_7  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_5  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM3_PARTIAL}, 
    { TIM3  ,GPIOC, GPIO_PIN_7  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_0  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOC, GPIO_PIN_8  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOB, GPIO_PIN_1  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM3_DISABLE}, 
    { TIM3  ,GPIOC, GPIO_PIN_9  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM3_ENABLE}, 
    { TIM3  ,GPIOD, GPIO_PIN_2  ,TIM_ETR    , AF_NO_REMAP    }, 
    { TIM4  ,GPIOB, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_12 ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOB, GPIO_PIN_7  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_13 ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOB, GPIO_PIN_8  ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_14 ,TIM_CH3    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOB, GPIO_PIN_9  ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM4_DISABLE}, 
    { TIM4  ,GPIOD, GPIO_PIN_15 ,TIM_CH4    , AF__HAL_AFIO_REMAP_TIM4_ENABLE}, 
    { TIM4  ,GPIOE, GPIO_PIN_0  ,TIM_ETR    , AF_NO_REMAP    }, 
    { TIM15 ,GPIOA, GPIO_PIN_9  ,TIM_BKIN   , AF_NO_REMAP    }, 
    { TIM15 ,GPIOA, GPIO_PIN_2  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM15_DISABLE}, 
    { TIM15 ,GPIOB, GPIO_PIN_14 ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM15_ENABLE}, 
    { TIM15 ,GPIOB, GPIO_PIN_15 ,TIM_CH1N   , AF_NO_REMAP    }, 
    { TIM15 ,GPIOA, GPIO_PIN_3  ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM15_DISABLE}, 
    { TIM15 ,GPIOB, GPIO_PIN_15 ,TIM_CH2    , AF__HAL_AFIO_REMAP_TIM15_ENABLE}, 
    { TIM16 ,GPIOB, GPIO_PIN_5  ,TIM_BKIN   , AF_NO_REMAP    }, 
    { TIM16 ,GPIOA, GPIO_PIN_6  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM16_ENABLE}, 
    { TIM16 ,GPIOB, GPIO_PIN_8  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM16_DISABLE}, 
    { TIM16 ,GPIOB, GPIO_PIN_6  ,TIM_CH1N   , AF_NO_REMAP    }, 
    { TIM17 ,GPIOA, GPIO_PIN_10 ,TIM_BKIN   , AF_NO_REMAP    }, 
    { TIM17 ,GPIOA, GPIO_PIN_7  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM17_ENABLE}, 
    { TIM17 ,GPIOB, GPIO_PIN_9  ,TIM_CH1    , AF__HAL_AFIO_REMAP_TIM17_DISABLE}, 
    { TIM17 ,GPIOB, GPIO_PIN_7  ,TIM_CH1N   , AF_NO_REMAP    }, 
}; 
