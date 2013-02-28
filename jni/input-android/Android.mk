LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := input-android
#LOCAL_ARM_MODE := arm

CORE_PATH := ../core

LOCAL_C_INCLUDES := $(LOCAL_PATH)/$(CORE_PATH)/src/api

LOCAL_SRC_FILES := plugin.c

LOCAL_CFLAGS := -DNO_ASM -DANDROID

ifeq ($(TARGET_ARCH_ABI), armeabi-v7a)
# Use for ARM7a:
#	LOCAL_LDFLAGS += -L$(LOCAL_PATH)/$(CORE_PATH)/obj/local/armeabi-v7a
else ifeq ($(TARGET_ARCH_ABI), armeabi)
# Use for pre-ARM7a:
#	LOCAL_LDFLAGS += -L$(LOCAL_PATH)/$(CORE_PATH)/obj/local/armeabi
else ifeq ($(TARGET_ARCH_ABI), x86)
	# TODO: set the proper flags here
else
	# Any other architectures that Android could be running on?
endif


LOCAL_CFLAGS += -O3 -ffast-math -frename-registers -fomit-frame-pointer -fsingle-precision-constant -fpredictive-commoning -fno-strict-aliasing -fvisibility=hidden
#LOCAL_LDLIBS += -lcore

LOCAL_LDLIBS := -ldl -llog
LOCAL_SHARED_LIBRARIES := core
LOCAL_STATIC_LIBRARIES := cpufeatures

include $(BUILD_SHARED_LIBRARY)

$(call import-module, android/cpufeatures)
