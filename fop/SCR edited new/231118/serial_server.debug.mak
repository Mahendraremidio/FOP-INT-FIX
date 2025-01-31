###########################################################
# Makefile generated by xIDE for uEnergy                   
#                                                          
# Project: serial_server
# Configuration: Debug
# Generated: Thu May 14 16:38:58 2020
#                                                          
# WARNING: Do not edit this file. Any changes will be lost 
#          when the project is rebuilt.                    
#                                                          
###########################################################

XIDE_PROJECT=serial_server
XIDE_CONFIG=Debug
OUTPUT=serial_server
OUTDIR=C:/Users/Remidio/Desktop/SCR edited new/231118
DEFS=

OUTPUT_TYPE=0
USE_FLASH=0
ERASE_NVM=1
CSFILE_CSR101x_A05=serial_server_csr101x_A05.keyr
MASTER_DB=app_gatt_db.db
LIBPATHS=
INCPATHS=
STRIP_SYMBOLS=0
OTAU_BOOTLOADER=1
OTAU_CSFILE=bootloader.keyr
OTAU_NAME=OTA Update
OTAU_SECRET=00112233445566778899aabbccddeeff

DBS=\
\
      gap_service_db.db\
      app_gatt_db.db\
      battery_service_db.db\
      dev_info_service_db.db\
      gatt_service_db.db\
      serial_service_db.db\
      csr_ota_db.db

INPUTS=\
      pio_ctrlr_code.asm\
      uart_interface.c\
      battery_service.c\
      nvm_access.c\
      dev_info_service.c\
      hw_access.c\
      serial_server.c\
      serial_gatt.c\
      serial_service.c\
      byte_queue.c\
      csr_ota_service.c\
      gatt_service.c\
      vf_pwm.c\
      i2c_comms.c\
      $(DBS)

KEYR=\
      serial_server_csr101x_A05.keyr\
      bootloader.keyr

# Project-specific options
hw_version=0

-include serial_server.mak
include $(SDK)/genmakefile.uenergy
