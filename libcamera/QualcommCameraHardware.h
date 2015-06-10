/*
** Copyright 2008, Google Inc.
** Copyright (c) 2009-2011, The Linux Foundation. All rights reserved.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef ANDROID_HARDWARE_QUALCOMM_CAMERA_HARDWARE_H
#define ANDROID_HARDWARE_QUALCOMM_CAMERA_HARDWARE_H

#define ICS

#include <utils/threads.h>
#ifdef ICS
#include <hardware/camera.h>
#endif
#include <camera/Camera.h>
#include "QCameraParameters.h"
#include <QComOMXMetadata.h>

extern "C" {
#include <linux/android_pmem.h>
#include <linux/msm_ion.h>
#include <camera.h>
}

struct str_map {
    const char *const desc;
    int val;
};

struct buffer_map {
    msm_frame *frame;
    buffer_handle_t * buffer;
    int size;
    int lockState;
};

typedef enum {
    TARGET_MSM7625,
    TARGET_MSM7625A,
    TARGET_MSM7627,
    TARGET_MSM7627A,
    TARGET_QSD8250,
    TARGET_MSM7630,
    TARGET_MSM8660,
    TARGET_MAX
}targetType;

typedef enum {
    LIVESHOT_DONE,
    LIVESHOT_IN_PROGRESS,
    LIVESHOT_STOPPED
}liveshotState;
#define MIN_UNDEQUEUD_BUFFER_COUNT 3
struct target_map {
    const char *targetStr;
    targetType targetEnum;
};

enum {
    BUFFER_UNLOCKED,
    BUFFER_LOCKED
};

struct board_property{
    targetType target;
    unsigned int previewSizeMask;
    bool hasSceneDetect;
    bool hasSelectableZoneAf;
    bool hasFaceDetect;
};

//EXIF globals
static const char ExifAsciiPrefix[] = { 0x41, 0x53, 0x43, 0x49, 0x49, 0x0, 0x0, 0x0 };          // "ASCII\0\0\0"
static const char ExifUndefinedPrefix[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };   // "\0\0\0\0\0\0\0\0"

//EXIF detfines
#define MAX_EXIF_TABLE_ENTRIES           14
#define GPS_PROCESSING_METHOD_SIZE       101
#define FOCAL_LENGTH_DECIMAL_PRECISION   100
#define EXIF_ASCII_PREFIX_SIZE           8   //(sizeof(ExifAsciiPrefix))

typedef struct{
    //GPS tags
    rat_t       latitude[3];
    rat_t       longitude[3];
    char        lonRef[2];
    char        latRef[2];
    rat_t       altitude;
    rat_t       gpsTimeStamp[3];
    char        gpsDateStamp[20];
    char        gpsProcessingMethod[EXIF_ASCII_PREFIX_SIZE+GPS_PROCESSING_METHOD_SIZE];
    //Other tags
    char        dateTime[20];
    rat_t       focalLength;
    uint16_t    flashMode;
    uint16_t    isoSpeed;
    char        make[17];
    char        model[17];

    bool        mAltitude;
    bool        mLongitude;
    bool        mLatitude;
    bool        mTimeStamp;
    bool        mGpsProcess;

    int         mAltitude_ref;
    long        mGPSTimestamp;
} exif_values_t;

namespace android {

class QualcommCameraHardware : public RefBase{
public:
    void setCallbacks(camera_notify_callback notify_cb,
                            camera_data_callback data_cb,
                            camera_data_timestamp_callback data_cb_timestamp,
                            camera_request_memory get_memory,
                            void *user);

    virtual void enableMsgType(int32_t msgType);
    virtual void disableMsgType(int32_t msgType);
    virtual bool msgTypeEnabled(int32_t msgType);

    virtual status_t startPreview();
    virtual void stopPreview();
    virtual bool previewEnabled();
    virtual status_t startRecording();
    virtual void stopRecording();
    virtual bool recordingEnabled();
    virtual void releaseRecordingFrame(const void *opaque);
    virtual status_t autoFocus();
    virtual status_t cancelAutoFocus();
    virtual status_t takePicture();
    virtual status_t takeLiveSnapshot();
    virtual status_t takeLiveSnapshotInternal();
    virtual status_t cancelPicture();
    virtual status_t setParameters(const QCameraParameters& params);
    virtual QCameraParameters getParameters() const;
    virtual status_t sendCommand(int32_t command, int32_t arg1, int32_t arg2);
    virtual sp<IMemory> getVideoBuffer(int32_t index);
#ifdef ICS
    virtual status_t set_PreviewWindow(void* param);
    virtual status_t setPreviewWindow(preview_stream_ops_t* window);
#endif
    virtual void release();

    static QualcommCameraHardware* createInstance();
    static QualcommCameraHardware* getInstance();

    void receivePreviewFrame(struct msm_frame *frame);
    void receiveLiveSnapshot();
    void receiveCameraStats(camstats_type stype, camera_preview_histogram_info* histinfo);
    void receiveRecordingFrame(struct msm_frame *frame);
    void receiveJpegPicture(status_t status, mm_camera_buffer_t *encoded_buffer);
    void notifyShutter(bool mPlayShutterSoundOnly);
    void receive_camframe_error_timeout();
    static void getCameraInfo();
    void receiveRawPicture(status_t status,struct msm_frame *postviewframe);
    int allocate_ion_memory(int *main_ion_fd, struct ion_allocation_data* alloc,
    struct ion_fd_data* ion_info_fd, int ion_type, int size, int *memfd);
    int deallocate_ion_memory(int *main_ion_fd, struct ion_fd_data* ion_info_fd);
    virtual ~QualcommCameraHardware();
    int storeMetaDataInBuffers(int enable);

private:
    QualcommCameraHardware();
    status_t startPreviewInternal();
    status_t startRecordingInternal();
    status_t setHistogramOn();
    status_t setHistogramOff();
    status_t runFaceDetection();
    status_t setFaceDetection(const char *str);

    void stopPreviewInternal();
    void *runAutoFocus();
    static void *openAutoFocusThread(void *data) { return ((QualcommCameraHardware *)data)->runAutoFocus(); }
    status_t cancelAutoFocusInternal();
    bool updatePictureDimension(const QCameraParameters& params, int& width, int& height);
    bool native_set_parms(camera_parm_type_t type, uint16_t length, void *value);
    bool native_set_parms(camera_parm_type_t type, uint16_t length, void *value, int *result);

    status_t startInitialPreview();
    void stopInitialPreview();
    status_t getBuffersAndStartPreview();
    void relinquishBuffers();

    /* These constants reflect the number of buffers that libmmcamera requires
       for preview and raw, and need to be updated when libmmcamera
       changes.
    */
    static const int kPreviewBufferCount = NUM_PREVIEW_BUFFERS;
    static const int kRawBufferCount = 1;
    static const int kJpegBufferCount = 1;
    static const int kTotalPreviewBufferCount = kPreviewBufferCount + MIN_UNDEQUEUD_BUFFER_COUNT;
    int numCapture;
    int numJpegReceived;

    QCameraParameters mParameters;
    unsigned int frame_size;
    bool mCameraRunning;
    Mutex mCameraRunningLock;
    bool mPreviewInitialized;


    class MMCameraDL : public RefBase{
    private:
        static wp<MMCameraDL> instance;
        MMCameraDL();
        virtual ~MMCameraDL();
        void *libmmcamera;
        static Mutex singletonLock;
    public:
        static sp<MMCameraDL> getInstance();
        void * pointer();
    };

    sp<MMCameraDL> mMMCameraDLRef;

    bool startCamera();
    bool initPreview();
    bool initRecord();
    void deinitPreview();
    bool initRaw(bool initJpegHeap);
    bool initZslBuffers(bool initJpegHeap);
    bool deinitZslBuffers();
    bool initLiveSnapshot(int videowidth, int videoheight);
    bool initRawSnapshot();
    void deinitRaw();
    void deinitRawSnapshot();
    bool mPreviewThreadRunning;
    bool createSnapshotMemory (int numberOfRawBuffers, int numberOfJpegBuffers,
                                   bool initJpegHeap, int snapshotFormat = 1 /*PICTURE_FORMAT_JPEG*/);
    Mutex mPreviewThreadWaitLock;
    Condition mPreviewThreadWait;
    void *openCamera();
    static void *openCameraThread(void *data) { return ((QualcommCameraHardware *)data)->openCamera(); }
    int mHFRCount;
    void *runPreviewThread();
    static void *openPreviewThread(void *data) { return ((QualcommCameraHardware *)data)->runPreviewThread(); }
    void *runHFRThread();
    static void *openHFRThread(void *data) { return ((QualcommCameraHardware *)data)->runHFRThread(); }
    bool mHFRThreadRunning;
	int mapBuffer(msm_frame *frame);
	int mapRawBuffer(msm_frame *frame);
	int mapThumbnailBuffer(msm_frame *frame);
	int mapJpegBuffer(mm_camera_buffer_t* buffer);
        int mapvideoBuffer( msm_frame *frame);
	int mapFrame(buffer_handle_t *buffer);
    Mutex mHFRThreadWaitLock;

    class FrameQueue : public RefBase{
    private:
        Mutex mQueueLock;
        Condition mQueueWait;
        bool mInitialized;

        Vector<struct msm_frame *> mContainer;
    public:
        FrameQueue();
        virtual ~FrameQueue();
        bool add(struct msm_frame *element);
        void flush();
        struct msm_frame* get();
        void init();
        void deinit();
        bool isInitialized();
    };

    FrameQueue mPreviewBusyQueue;

    bool mFrameThreadRunning;
    Mutex mFrameThreadWaitLock;
    Condition mFrameThreadWait;
    void *runFrameThread();
    static void *openFrameThread(void *data) { return ((QualcommCameraHardware *)data)->runFrameThread(); }

    //720p recording video thread
    bool mVideoThreadExit;
    bool mVideoThreadRunning;
    Mutex mVideoThreadWaitLock;
    Condition mVideoThreadWait;
    void *runVideoThread();
    static void *openVideoThread(void *data) { return ((QualcommCameraHardware *)data)->runVideoThread(); }

    // smooth zoom
    int mTargetSmoothZoom;
    bool mSmoothzoomThreadExit;
    bool mSmoothzoomThreadRunning;
    Mutex mSmoothzoomThreadWaitLock;
    Mutex mSmoothzoomThreadLock;
    Condition mSmoothzoomThreadWait;
    void *runSmoothzoomThread();
    static void *openSmoothZoomThread(void *data) { return ((QualcommCameraHardware *)data)->runSmoothzoomThread(); }

    // For Histogram
    int mStatsOn;
    int mCurrent;
    bool mSendData;
    Mutex mStatsWaitLock;
    Condition mStatsWait;

    //For Face Detection
    int mFaceDetectOn;
    bool mSendMetaData;
    Mutex mMetaDataWaitLock;

    bool mShutterPending;
    Mutex mShutterLock;

    bool mSnapshotThreadRunning;
    Mutex mSnapshotThreadWaitLock;
    Condition mSnapshotThreadWait;
    void *runSnapshotThread();
    static void *openSnapshotThread(void *data) { return ((QualcommCameraHardware *)data)->runSnapshotThread(); }
    Mutex mRawPictureHeapLock;
    bool mJpegThreadRunning;
    Mutex mJpegThreadWaitLock;
    Condition mJpegThreadWait;
    bool mInSnapshotMode;
    Mutex mInSnapshotModeWaitLock;
    Condition mInSnapshotModeWait;
    bool mEncodePending;
    Mutex mEncodePendingWaitLock;
    Condition mEncodePendingWait;
	bool mBuffersInitialized;

    void debugShowPreviewFPS() const;

    int mSnapshotFormat;
    void hasAutoFocusSupport();
    void filterPictureSizes();
    void filterPreviewSizes();
    static void storeTargetType();
    bool supportsSceneDetection();
    bool supportsSelectableZoneAf();
    bool supportsFaceDetection();

    void initDefaultParameters();
    bool initImageEncodeParameters(int size);
    bool initZslParameter(void);
    status_t setCameraMode(const QCameraParameters& params);
    status_t setPreviewSize(const QCameraParameters& params);
    status_t setJpegThumbnailSize(const QCameraParameters& params);
    status_t setPreviewFpsRange(const QCameraParameters& params);
    status_t setPreviewFrameRate(const QCameraParameters& params);
    status_t setPreviewFrameRateMode(const QCameraParameters& params);
    status_t setRecordSize(const QCameraParameters& params);
    status_t setPictureSize(const QCameraParameters& params);
    int getISOSpeedValue();
    status_t setJpegQuality(const QCameraParameters& params);
    status_t setAntibanding(const QCameraParameters& params);
    status_t setEffect(const QCameraParameters& params);
    status_t setRecordingHint(const QCameraParameters& params);
    status_t setExposureCompensation(const QCameraParameters &params);
    status_t setAutoExposure(const QCameraParameters& params);
    status_t setWhiteBalance(const QCameraParameters& params);
    status_t setFlash(const QCameraParameters& params);
    status_t setGpsLocation(const QCameraParameters& params);
    status_t setRotation(const QCameraParameters& params);
    status_t setZoom(const QCameraParameters& params);
    status_t setFocusMode(const QCameraParameters& params);
    status_t setFocusAreas(const QCameraParameters& params);
    status_t setMeteringAreas(const QCameraParameters& params);
    status_t setBrightness(const QCameraParameters& params);
    status_t setSkinToneEnhancement(const QCameraParameters& params);
    status_t setOrientation(const QCameraParameters& params);
    status_t setLensshadeValue(const QCameraParameters& params);
    status_t setMCEValue(const QCameraParameters& params);
    status_t setHDRImaging(const QCameraParameters& params);
    status_t setExpBracketing(const QCameraParameters& params);
    status_t setISOValue(const QCameraParameters& params);
    status_t setPictureFormat(const QCameraParameters& params);
    status_t setSharpness(const QCameraParameters& params);
    status_t setContrast(const QCameraParameters& params);
    status_t setSaturation(const QCameraParameters& params);
    status_t setSceneMode(const QCameraParameters& params);
    status_t setContinuousAf(const QCameraParameters& params);
    status_t setTouchAfAec(const QCameraParameters& params);
    status_t setSceneDetect(const QCameraParameters& params);
    status_t setStrTextures(const QCameraParameters& params);
    status_t setPreviewFormat(const QCameraParameters& params);
    status_t setSelectableZoneAf(const QCameraParameters& params);
    status_t setHighFrameRate(const QCameraParameters& params);
    status_t setRedeyeReduction(const QCameraParameters& params);
    status_t setDenoise(const QCameraParameters& params);
    status_t setZslParam(const QCameraParameters& params);
    status_t setSnapshotCount(const QCameraParameters& params);
    void setGpsParameters();
    bool storePreviewFrameForPostview();
    bool isValidDimension(int w, int h);
    status_t updateFocusDistances(const char *focusmode);
    int mStoreMetaDataInFrame;

    Mutex mLock;
	Mutex mDisplayLock;
    Mutex mCamframeTimeoutLock;
    bool camframe_timeout_flag;
    bool mReleasedRecordingFrame;

    Mutex mParametersLock;


    Mutex mCallbackLock;
	Mutex mRecordLock;
	Mutex mRecordFrameLock;
	Condition mRecordWait;
    Condition mStateWait;

    unsigned int        mPreviewFrameSize;
    unsigned int        mRecordFrameSize;
    int                 mRawSize;
    int                 mCbCrOffsetRaw;
    int                 mJpegMaxSize;
    int32_t                 mStatSize;


    cam_ctrl_dimension_t mDimension;
    bool mAutoFocusThreadRunning;
    Mutex mAutoFocusThreadLock;

    Mutex mAfLock;

    pthread_t mFrameThread;
    pthread_t mVideoThread;
    pthread_t mPreviewThread;
    pthread_t mSnapshotThread;
    pthread_t mDeviceOpenThread;
    pthread_t mSmoothzoomThread;
    pthread_t mHFRThread;

    bool mInitialized;

    int mBrightness;
    int mSkinToneEnhancement;
    int mHJR;
    unsigned int mThumbnailMapped[MAX_SNAPSHOT_BUFFERS];
    unsigned int mThumbnailLockState[MAX_SNAPSHOT_BUFFERS];
    int mRawfd[MAX_SNAPSHOT_BUFFERS];
    int mRawSnapshotfd;
    int mJpegfd[MAX_SNAPSHOT_BUFFERS];
    int mRecordfd[9];
    camera_memory_t *mPreviewMapped[kPreviewBufferCount + MIN_UNDEQUEUD_BUFFER_COUNT];
    camera_memory_t *mRawMapped[MAX_SNAPSHOT_BUFFERS];
    camera_memory_t *mJpegMapped[MAX_SNAPSHOT_BUFFERS];
    camera_memory_t *mRawSnapshotMapped;
    camera_memory_t *mStatsMapped[3];
    camera_memory_t *mRecordMapped[9];
    camera_memory_t *mJpegCopyMapped;
    camera_memory_t* metadata_memory[9];
    camera_memory_t *mJpegLiveSnapMapped;
    int raw_main_ion_fd[MAX_SNAPSHOT_BUFFERS];
    int raw_snapshot_main_ion_fd;
    int record_main_ion_fd[9];
    struct ion_allocation_data raw_alloc[MAX_SNAPSHOT_BUFFERS];
    struct ion_allocation_data raw_snapshot_alloc;
    struct ion_allocation_data record_alloc[9];
    struct ion_fd_data raw_ion_info_fd[MAX_SNAPSHOT_BUFFERS];
    struct ion_fd_data raw_snapshot_ion_info_fd;
    struct ion_fd_data record_ion_info_fd[9];

    struct msm_frame frames[kPreviewBufferCount + MIN_UNDEQUEUD_BUFFER_COUNT];
    struct buffer_map frame_buffer[kPreviewBufferCount + MIN_UNDEQUEUD_BUFFER_COUNT];
    struct msm_frame *recordframes;
    bool *record_buffers_tracking_flag;
    preview_stream_ops_t* mPreviewWindow;
    android_native_buffer_t *mPostViewBuffer;
    buffer_handle_t *mThumbnailBuffer[MAX_SNAPSHOT_BUFFERS];
    bool mIs3DModeOn;

    int32_t mMsgEnabled;    // camera msg to be handled
    camera_notify_callback mNotifyCallback;
    camera_data_callback mDataCallback;
    camera_data_timestamp_callback mDataCallbackTimestamp;
    camera_request_memory mGetMemory;
    void *mCallbackCookie;  // same for all callbacks
    int mDebugFps;
    int kPreviewBufferCountActual;
    int previewWidth, previewHeight;
    bool mSnapshotDone;
    int maxSnapshotWidth;
    int maxSnapshotHeight;
    bool mHasAutoFocusSupport;
    int videoWidth, videoHeight;

    bool mDisEnabled;
    int mRotation;
    bool mResetWindowCrop;
    int mThumbnailWidth, mThumbnailHeight;
    status_t setDIS();
    bool strTexturesOn;
    int mPictureWidth;
    int mPictureHeight;
    int mPostviewWidth;
    int mPostviewHeight;
	int mTotalPreviewBufferCount;
    int mDenoiseValue;
    int mZslEnable;
    int mZslPanorama;
    bool mZslFlashEnable;
    cam_3d_frame_format_t mSnapshot3DFormat;
    bool mSnapshotCancel;
    Mutex mSnapshotCancelLock;
    int mActualPictWidth;
    int mActualPictHeight;
    bool mUseJpegDownScaling;
    bool mPreviewStopping;
    bool mHdrMode;

    bool mMultiTouch;

    int mRecordingState;

    //EXIF
    void addExifTag(exif_tag_id_t tagid, exif_tag_type_t type,
        uint32_t count, uint8_t copy, void *data);
    void setExifTags();
    void initExifData();
    void deinitExifData();
    void setExifTagsGPS();
    exif_tags_info_t* getExifData() { return mExifData; }
    int getExifTableNumEntries() { return mExifTableNumEntries; }
    void parseGPSCoordinate(const char *latlonString, rat_t* coord);
    exif_tags_info_t    mExifData[MAX_EXIF_TABLE_ENTRIES];  //Exif tags for JPEG encoder
    exif_values_t       mExifValues;                        //Exif values in usable format
    int                 mExifTableNumEntries;               //Number of entries in mExifData
};

extern "C" int HAL_getNumberOfCameras();
extern "C" void HAL_getCameraInfo(int cameraId, struct CameraInfo* cameraInfo);
extern "C" QualcommCameraHardware* HAL_openCameraHardware(int cameraId);
}; // namespace android

#endif
