/**
 * Ciastkolog.pl (https://github.com/ciastkolog)
 * 
*/
/**
 * The MIT License (MIT)
 * Copyright (c) 2016 sheinz (https://github.com/sheinz)
 */
#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 
O sensor BMP280 apresenta a mesma construção do sensor BME280, com a diferença sendo que o chip do segundo também realiza medição de umidade.
Sendo assim, só é possível a identificação entre um e outro a partir da leitura de endereços.
Esta biblioteca faz a implementação das funções para ambos os sensores, simultaneamente.
Caso o chip seja o BMP280, só haverá leitura de pressão e temperatura, mesmo que a função seja parametrizada com umidade.
*/
#define BMP280_I2C_ADDRESS_0  0x76
#define BMP280_I2C_ADDRESS_1  0x77

#define BMP280_CHIP_ID  0x58 /* BMP280 has chip-id 0x58 */
#define BME280_CHIP_ID  0x60 /* BME280 has chip-id 0x60 */

// modos de operação do sensor
typedef enum {
    BMP280_MODE_SLEEP = 0,
    BMP280_MODE_FORCED = 1,
    BMP280_MODE_NORMAL = 3
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

// configuração de oversampling da pressão
typedef enum {
    BMP280_SKIPPED = 0,          /* no measurement  */
    BMP280_ULTRA_LOW_POWER = 1,  /* oversampling x1 */
    BMP280_LOW_POWER = 2,        /* oversampling x2 */
    BMP280_STANDARD = 3,         /* oversampling x4 */
    BMP280_HIGH_RES = 4,         /* oversampling x8 */
    BMP280_ULTRA_HIGH_RES = 5    /* oversampling x16 */
} BMP280_Oversampling;

// intervalo de tempo entre medidas
typedef enum {
    BMP280_STANDBY_05 = 0,      /* stand by time 0.5ms */
    BMP280_STANDBY_62 = 1,      /* stand by time 62.5ms */
    BMP280_STANDBY_125 = 2,     /* stand by time 125ms */
    BMP280_STANDBY_250 = 3,     /* stand by time 250ms */
    BMP280_STANDBY_500 = 4,     /* stand by time 500ms */
    BMP280_STANDBY_1000 = 5,    /* stand by time 1s */
    BMP280_STANDBY_2000 = 6,    /* stand by time 2s BMP280, 10ms BME280 */
    BMP280_STANDBY_4000 = 7,    /* stand by time 4s BMP280, 20ms BME280 */
} BMP280_StandbyTime;

// criação de estrutura para os parâmetros do sensor
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;

// criação de estrutura para o sensor
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    uint16_t addr;

    I2C_HandleTypeDef* i2c;

    bmp280_params_t params;

    uint8_t  id;        /* Chip ID */

} BMP280_HandleTypedef;

// inicialização dos parâmetros como default
void bmp280_init_default_params(bmp280_params_t *params);

// inicialização do sensor
bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params);

// inicializa as medidas em modo forçado
bool bmp280_force_measurement(BMP280_HandleTypedef *dev);

// função que verifica se o sensor está realizando medidas
bool bmp280_is_measuring(BMP280_HandleTypedef *dev);

// leitura dos dados compensados
bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature,
                       uint32_t *pressure, uint32_t *humidity);

// leitura dos dados em medidas reais (temperatura em °C, pressão em Pa)
bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature,
                       float *pressure, float *humidity);


#endif  // __BMP280_H__
