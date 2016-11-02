F3_TARGETS  += $(TARGET)
FEATURES    = VCP SDCARD HIGHEND

TARGET_SRC = \
            drivers/accgyro_adxl345.c \
            drivers/accgyro_bma280.c \
            drivers/accgyro_l3gd20.c \
            drivers/accgyro_l3g4200d.c \
            drivers/accgyro_lsm303dlhc.c \
            drivers/compass_hmc5883l.c \
            drivers/accgyro_adxl345.c \
            drivers/accgyro_bma280.c \
            drivers/accgyro_mma845x.c \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu3050.c \
            drivers/accgyro_mpu6050.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_spi_mpu9250.c \
            drivers/barometer_bmp085.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_ms5611.c \
            drivers/compass_ak8963.c \
            drivers/compass_ak8975.c \
            drivers/compass_hmc5883l.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c \
            io/osd.c \
            io/canvas.c \
            io/cms.c \
            io/cms_imu.c \
            io/cms_blackbox.c \
            io/cms_vtx.c \
            io/cms_ledstrip.c

