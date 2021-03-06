# Copyright (C) 2012 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# This file sets variables that control the way modules are built
# thorughout the system. It should not be used to conditionally
# disable makefiles (the proper mechanism to control what gets
# included in a build is to use PRODUCT_PACKAGES in a product
# definition file).
#

# WARNING: This line must come *before* including the proprietary
# variant, so that it gets overwritten by the parent (which goes
# against the traditional rules of inheritance).

# Vendor stuff
# inherit from the proprietary version
-include vendor/samsung/ancora_tmo/BoardConfigVendor.mk

# create the folder /usr to prevent the build from failing
$(shell mkdir -p $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/)

# Platform
TARGET_BOOTLOADER_BOARD_NAME := ancora_tmo
TARGET_OTA_ASSERT_DEVICE := ancora_tmo,SGH-T679,ancora
TARGET_BOARD_PLATFORM := msm7x30
TARGET_BOARD_PLATFORM_GPU := qcom-adreno200
TARGET_NO_BOOTLOADER := true
TARGET_NO_RADIOIMAGE := true
TARGET_NO_INITLOGO := true

# Architecture
TARGET_ARCH := arm
TARGET_ARCH_VARIANT := armv7-a-neon
TARGET_CPU_VARIANT := scorpion
TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi
TARGET_ARCH_LOWMEM := true

# Kernel
BOARD_KERNEL_BASE := 0x00400000
BOARD_KERNEL_PAGESIZE := 2048
TARGET_KERNEL_SOURCE := kernel/samsung/ancora_tmo
TARGET_KERNEL_CONFIG := ancora_tmo_defconfig

# Wifi related defines
BOARD_HAVE_SAMSUNG_WIFI          := true
WIFI_BAND                        := 802_11_ABG
WPA_SUPPLICANT_VERSION           := VER_0_8_X
BOARD_WPA_SUPPLICANT_DRIVER      := NL80211
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_bcmdhd
BOARD_HOSTAPD_DRIVER             := NL80211
BOARD_HOSTAPD_PRIVATE_LIB        := lib_driver_cmd_bcmdhd
BOARD_WLAN_DEVICE                := bcmdhd
WIFI_DRIVER_MODULE_PATH          := "/system/lib/modules/dhd.ko"
WIFI_DRIVER_MODULE_NAME          := "dhd"
WIFI_DRIVER_MODULE_ARG           := "firmware_path=/vendor/firmware/fw_bcmdhd.bin nvram_path=/vendor/firmware/nvram_net.txt"
WIFI_DRIVER_MODULE_AP_ARG        := "firmware_path=/vendor/firmware/fw_bcmdhd_apsta.bin nvram_path=/vendor/firmware/nvram_net.txt"
WIFI_DRIVER_FW_PATH_PARAM        := "/sys/module/dhd/parameters/firmware_path"
WIFI_DRIVER_FW_PATH_STA          := "/vendor/firmware/fw_bcmdhd.bin"
WIFI_DRIVER_FW_PATH_AP           := "/vendor/firmware/fw_bcmdhd_apsta.bin"

# Bluetooth
BOARD_HAVE_BLUETOOTH := true
BOARD_HAVE_BLUETOOTH_BCM := true
BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR := device/samsung/ancora_tmo/bluetooth
BOARD_BLUEDROID_VENDOR_CONF := device/samsung/ancora_tmo/bluetooth/vnd_ancora.txt

#RIL
BOARD_MOBILEDATA_INTERFACE_NAME = "pdp0"
BOARD_RIL_CLASS := ../../../device/samsung/ancora_tmo/ril/
BOARD_USES_LEGACY_RIL := true

# GPS
BOARD_USES_QCOM_GPS := true
BOARD_VENDOR_QCOM_GPS_LOC_API_HARDWARE := msm7x30
BOARD_VENDOR_QCOM_GPS_LOC_API_AMSS_VERSION := 50000

# Graphics
BOARD_EGL_CFG := device/samsung/ancora_tmo/configs/egl.cfg
TARGET_USES_C2D_COMPOSITION := true
USE_OPENGL_RENDERER := true

# Memory allocation
TARGET_USES_ION := true
BOARD_NEEDS_MEMORYHEAPPMEM := true

# QCOM
COMMON_GLOBAL_CFLAGS += -DQCOM_HARDWARE
COMMON_GLOBAL_CFLAGS += -DBINDER_COMPAT
COMMON_GLOBAL_CFLAGS += -DQCOM_BSP
COMMON_GLOBAL_CFLAGS += -DWITH_QCOM_VOIP_OVER_MVS
BOARD_USES_QCOM_HARDWARE := true
BOARD_VENDOR_QCOM_AMSS_VERSION := 6225
TARGET_QCOM_DISPLAY_VARIANT := caf
TARGET_QCOM_MEDIA_VARIANT := caf
TARGET_QCOM_AUDIO_VARIANT := caf
TARGET_ENABLE_QC_AV_ENHANCEMENTS := true
TARGET_USES_QCOM_BSP := true
TARGET_USE_QCOM_BIONIC_OPTIMIZATION := true

# Use insecure heap
TARGET_DISPLAY_INSECURE_MM_HEAP := true

# Power HAL
TARGET_PROVIDES_POWERHAL := true

# Lights HAL
TARGET_PROVIDES_LIBLIGHT := true

# healthd HAL
BOARD_HAL_STATIC_LIBRARIES := libhealthd.qcom

# Camera stuff
COMMON_GLOBAL_CFLAGS += -DSAMSUNG_CAMERA_LEGACY
TARGET_RELEASE_CPPFLAGS += -DNEEDS_VECTORIMPL_SYMBOLS
USE_CAMERA_STUB := true
BOARD_USES_LEGACY_OVERLAY := true
USE_DEVICE_SPECIFIC_CAMERA := true

# Audio
BOARD_HAVE_SAMSUNG_AUDIO := true
BOARD_USES_QCOM_AUDIO_LPA := true
BOARD_USES_LEGACY_ALSA_AUDIO := true
BOARD_USES_QCOM_AUDIO_RESETALL := true
BOARD_USES_QCOM_AUDIO_VOIPMUTE := true

# Sensors
BOARD_USE_LEGACY_SENSORS_FUSION := false


# FM Radio
BOARD_HAVE_FM_RADIO := true
BOARD_GLOBAL_CFLAGS += -DHAVE_FM_RADIO
BOARD_FM_DEVICE := si4709

# USB mass storage
TARGET_USE_CUSTOM_LUN_FILE_PATH := /sys/devices/platform/msm_hsusb/gadget/lun%d/file
# Enable below line if compiling for a recovery version before 6.0.1.2
#BOARD_UMS_LUNFILE := /sys/devices/platform/msm_hsusb/gadget/lun%d/file
# End recovery stuff

# Partitions
# For the Exhibit II, these are calculated from /proc/partitions
BOARD_BOOTIMAGE_PARTITION_SIZE := 5767168
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 7864320
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 1163919360
BOARD_USERDATAIMAGE_PARTITION_SIZE := 1832894464
BOARD_CACHEIMAGE_PARTITION_SIZE := 560988160
BOARD_FLASH_BLOCK_SIZE := 4096

# Target filesystem
TARGET_USERIMAGES_USE_EXT4 := true
TARGET_USERIMAGES_USE_F2FS := true

# Vold stuff
BOARD_VOLD_EMMC_SHARES_DEV_MAJOR := true
BOARD_VOLD_MAX_PARTITIONS := 28

# Custom recovery files
TARGET_RECOVERY_DEVICE_DIRS += device/samsung/ancora_tmo
TARGET_RECOVERY_INITRC := device/samsung/ancora_tmo/recovery/init.recovery.rc
TARGET_RECOVERY_FSTAB := device/samsung/ancora_tmo/rootdir/fstab.qcom
BOARD_CUSTOM_RECOVERY_KEYMAPPING := ../../device/samsung/ancora_tmo/recovery/recovery_keys.c
COMMON_GLOBAL_CFLAGS += -DNO_SECURE_DISCARD

# TWRP recovery stuff
BOARD_HAS_SDCARD_INTERNAL := true
BOARD_USES_MMCUTILS := true
BOARD_HAS_NO_MISC_PARTITION := true
BOARD_HAS_NO_SELECT_BUTTON := true
DEVICE_RESOLUTION := 480x800
TW_TARGET_USES_QCOM_BSP := true
TW_SCREEN_BLANK_ON_BOOT := true
TW_BRIGHTNESS_PATH := /sys/devices/platform/msm_fb.196609/leds/lcd-backlight/brightness
TW_MAX_BRIGHTNESS := 255
TW_CUSTOM_POWER_BUTTON := 107
BOARD_USE_CUSTOM_RECOVERY_FONT:= \"roboto_10x18.h\"
RECOVERY_SDCARD_ON_DATA := true
TW_EXTERNAL_STORAGE_PATH := "/external_sd"
TW_EXTERNAL_STORAGE_MOUNT_POINT := "external_sd"
TW_NO_REBOOT_BOOTLOADER := true
TW_HAS_DOWNLOAD_MODE := true
TW_USE_MODEL_HARDWARE_ID_FOR_DEVICE_ID := true
TW_INCLUDE_FB2PNG := true
TARGET_RECOVERY_PIXEL_FORMAT := "RGBX_8888"
RECOVERY_VARIANT := twrp

# Bootanimation
TARGET_BOOTANIMATION_PRELOAD := true
TARGET_BOOTANIMATION_TEXTURE_CACHE := true
TARGET_BOOTANIMATION_USE_RGB565 := true

# SELinux
BOARD_SEPOLICY_DIRS += \
    device/samsung/ancora_tmo/sepolicy

BOARD_SEPOLICY_UNION += \
    file_contexts \
    property_contexts \
    bridge.te \
    camera.te \
    device.te \
    dhcp.te \
    domain.te \
    file.te \
    healthd.te \
    init.te \
    mac_update.te \
    mediaserver.te \
    rild.te \
    rmt.te \
    surfaceflinger.te \
    system.te \
    tee.te \
    ueventd.te \
    wpa_supplicant.te
