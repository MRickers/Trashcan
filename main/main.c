#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "queue/ld_queue.h"
#include "vl53l0x.h"

/*
void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    // New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
    

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);

}

VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    int i;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
        printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
        		(FixPoint1616_t)(1.5*0.023*65536));
    }


    //  Step  4 : Test ranging mode
     

    if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<10;i++){
            printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

            print_pal_error(Status);
            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
            		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

            printf("RANGE IGNORE THRESHOLD: %f\n\n", (float)LimitCheckCurrent/65536.0);


            if (Status != VL53L0X_ERROR_NONE) break;

            printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);


        }
    }
    return Status;
}


#define SDA_PIN     21
#define SCL_PIN     22
static char tag[] = "i2cscanner";

void task_i2cscanner(void *ignore) {
	ESP_LOGD(tag, ">> i2cScanner");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	esp_err_t error = i2c_param_config(I2C_NUM_0, &conf);
    printf("param_config=%d\n", error);
	error = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    printf("install=%d\n", error);

	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (int i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 );// expect ack
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}
*/
void app_main() {
    int16_t data[32] = {0x00};

    VL53L0x_SetDebugLevel(DEBUG_INFO);
    VL53L0x_Init(1, 21, 22);
    VL53L0x_GetData(&data[0], 32);

    printf("\n\n");
    for(int i=0;i<32;i++) {
        printf("1.  %d\n", data[i]);
    }
    fflush(stdout);
    /*
    VL53L0X_Dev_t dev = {0x00};
    VL53L0X_Error Status = {0x00};
    VL53L0X_DeviceInfo_t dev_info = {0x00};
    i2c_port_t i2c_master_port = I2C_NUM_1;
    i2c_config_t conf={0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;
    
    esp_err_t esp_err = i2c_param_config(i2c_master_port, &conf); 
    printf("param_config=%d\n", esp_err);
    ESP_ERROR_CHECK(esp_err);
    esp_err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    printf("driver_install=%d\n", esp_err);
    ESP_ERROR_CHECK(esp_err);

    dev.i2c_address = 0x29;
    dev.i2c_port_num = I2C_NUM_1;



    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(&dev); // Data initialization
        print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(&dev, &dev_info);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X_GetDeviceInfo:\n");
        printf("Device Name : %s\n", dev_info.Name);
        printf("Device Type : %s\n", dev_info.Type);
        printf("Device ID : %s\n", dev_info.ProductId);
        printf("ProductRevisionMajor : %d\n", dev_info.ProductRevisionMajor);
        printf("ProductRevisionMinor : %d\n", dev_info.ProductRevisionMinor);

        if ((dev_info.ProductRevisionMinor != 1) && (dev_info.ProductRevisionMinor != 1)) {
        	printf("Error expected cut 1.1 but found cut %d.%d\n",
        			dev_info.ProductRevisionMajor, dev_info.ProductRevisionMinor);
        	Status = VL53L0X_ERROR_NOT_SUPPORTED;
        }
        fflush(stdout);
    }


    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(&dev); // Device Initialization
        // StaticInit will set interrupt by default
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
		printf ("Call of VL53L0X_StartMeasurement\n");
		Status = VL53L0X_StartMeasurement(&dev);
		print_pal_error(Status);
    }
    VL53L0X_RangingMeasurementData_t pRangingMeasurementData = {0x00};
    if(Status == VL53L0X_ERROR_NONE)
    {
        uint32_t measurement;
        uint32_t no_of_measurements = 32;

        uint16_t* pResults = (uint16_t*)malloc(sizeof(uint16_t) * no_of_measurements);

        for(measurement=0; measurement<no_of_measurements; measurement++)
        {

            Status = WaitMeasurementDataReady(&dev);

            if(Status == VL53L0X_ERROR_NONE)
            {
                Status = VL53L0X_GetRangingMeasurementData(&dev, &pRangingMeasurementData);

                *(pResults + measurement) = pRangingMeasurementData.RangeMilliMeter;
                printf("In loop measurement %d: %d\n", measurement, pRangingMeasurementData.RangeMilliMeter);

                // Clear the interrupt
                VL53L0X_ClearInterruptMask(&dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
                VL53L0X_PollingDelay(&dev);
            } else {
                break;
            }
        }

        if(Status == VL53L0X_ERROR_NONE)
        {
            for(measurement=0; measurement<no_of_measurements; measurement++)
            {
                printf("measurement %d: %d\n", measurement, *(pResults + measurement));
            }
        }

        free(pResults);
    }
*/
    vTaskDelay(5000/portTICK_PERIOD_MS);
    esp_restart();
}

/*
void app_main() {
    VL53L0X_Version_t version = {0x00};
    VL53L0X_Error error = {0x00};
    VL53L0X_Dev_t dev = {0x00};

    printf("Hello from app_main\n");
    vTaskDelay(1000/portTICK_PERIOD_MS);

    error = VL53L0X_GetVersion(&version);

    if(error == VL53L0X_ERROR_NONE) {
        printf("Version is %d.%d.%d\n", version.major, version.minor, version.revision);
    }else {
        printf("Retrieving version failed\n");
    }
    
    printf("Init i2c\n");

    i2c_port_t i2c_master_port = I2C_NUM_1;
    i2c_config_t conf={0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;
    
    esp_err_t esp_err = i2c_param_config(i2c_master_port, &conf); 
    printf("param_config=%d\n", esp_err);
    ESP_ERROR_CHECK(esp_err);
    esp_err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    printf("driver_install=%d\n", esp_err);
    ESP_ERROR_CHECK(esp_err);

    dev.i2c_address = 0x29;
    dev.i2c_port_num = I2C_NUM_1;


    printf ("Call of VL53L0X_DataInit\n");
    error = VL53L0X_StaticInit(&dev); // Data initialization
    print_pal_error(error);


    if(error == VL53L0X_ERROR_NONE) {
        
        for(int i=5;i>0;i--) {
            error = rangingTest(&dev);
            if(error == VL53L0X_ERROR_NONE) {
                
                printf("Success\n");

            }else {
                printf("Measurement error: %d\n ", error);
            }
            fflush(stdout);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }else {
        printf("Init failed\n");
    }
    
    vTaskDelay(1000/portTICK_PERIOD_MS);
    esp_restart();
} 
*/