# Predictive Maintainace  using ADXL345 and STM32

>
    Code Originally written by Mohammed Faariq Siddiqui
                        using STM32CubeIDE.
>

This project makes an approach towards Predictive maintainence using **STM32**  _(Specifically using STM32F103RB)_. The code in this repository is the C Code _(in the [main.c file](https://github.com/Faariq-Siddiqui/ADXL345_Internship_Project/blob/main/STM32%20ADXL345/main.c#L34))_  

Given below is a helpful table to select values for ADXL345.

| G Value | Hex Value |  Bit Resolution |   Type    |
|:-------:|:---------:|:---------------:|:---------:|
| 02 G    |   0x00    | 10 Bit          | Not Full  |
| 04 G    |   0x01    | 10 Bit          | Not Full  |
| 08 G    |   0x02    | 10 Bit          | Not Full  |
| 16 G    |   0x03    | 10 Bit          | Not Full  |
|    -    |    -      |   -             |    -      |
| 02 G    |   0x08    | 10 Bit          |  Full     |
| 04 G    |   0x09    | 11 Bit          |  Full     |
| 08 G    |   0x0A    | 12 Bit          |  Full     |
| 16 G    |   0x0B    | 13 Bit          |  Full     |

---
### Omit this portion:

```C
#define OUTPUT_AS_CSV 2
```

```C
case OUTPUT_AS_CSV:
  				printf("%d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f\r\n", numberOfReadings, timeInMilliSeconds,
  				            sensorData.reg_x, sensorData.reg_y, sensorData.reg_z, output_in_G.reg_x, output_in_G.reg_y,output_in_G.reg_z,
  				            output_in_MetersPerSecondSquared.reg_x, output_in_MetersPerSecondSquared.reg_y, output_in_MetersPerSecondSquared.reg_z,
  				            total_value_in_G, total_value_in_MeterPerSecondSquare);

  				HAL_Delay(DELAY_PER_CYCLE);
  				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  				HAL_Delay(DELAY_PER_CYCLE);
  				break;
```

That is because this is an extra piece of code which I wrote to make my task easier to process the data directly in Microsoft Excel. 
