LOCAL_PATH := $(call my-direction)

include $(CLEAR_VARS)

LOCAL_MODULE    := InputSource
LOCAL_SRC_FILES := ImageSourceEngine.cpp IMUSourceEngine.cpp OpenNI2Engine.cpp FFMPEGReader.cpp FFMPEGWriter.cpp
LOCAL_CFLAGS := -Werror
ifneq ($(FFMPEG_ROOT),)
LOCAL_CFLAGS += -DCOMPILE_WITH_FFMPEG
LOCAL_STATIC_LIBRARIES += $(MY_FFMPEG_MODULE)
endif
ifneq ($(OPENNI2_ROOT),)
LOCAL_SHARED_LIBRARIES += $(MY_OPENNI2_MODULE)
LOCAL_CFLAGS += -DWITH_OPENNI2
endif

LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
#LOCAL_CFLAGS += -DCOMPILE_WITHOUT_CUDA

include $(BUILD_STATIC_LIBRARY)

