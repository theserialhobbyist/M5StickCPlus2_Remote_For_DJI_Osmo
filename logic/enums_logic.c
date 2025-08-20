#include "enums_logic.h"

const char* camera_mode_to_string(camera_mode_t mode) {
    switch (mode) {
        case CAMERA_MODE_SLOW_MOTION:
            return "Slow Motion";
        case CAMERA_MODE_NORMAL:
            return "Video";
        case CAMERA_MODE_TIMELAPSE:
            return "Timelapse";
        case CAMERA_MODE_PHOTO:
            return "Photo";
        case CAMERA_MODE_HYPERLAPSE:
            return "Hyperlapse";
        case CAMERA_MODE_LIVE_STREAMING:
            return "Live Streaming";
        case CAMERA_MODE_UVC_STREAMING:
            return "UVC Live Streaming";
        case CAMERA_MODE_SUPERNIGHT:
            return "SuperNight";
        case CAMERA_MODE_SUBJECT_TRACKING:
            return "Subject Tracking";
        default:
            return "Other Mode, see 1D06 Cmd";
    }
}

const char* camera_status_to_string(camera_status_t status) {
    switch (status) {
        case CAMERA_STATUS_SCREEN_OFF:
            return "Screen off";
        case CAMERA_STATUS_LIVE_STREAMING:
            return "Live streaming (including screen-on without recording)";
        case CAMERA_STATUS_PLAYBACK:
            return "Playback";
        case CAMERA_STATUS_PHOTO_OR_RECORDING:
            return "Photo or recording";
        case CAMERA_STATUS_PRE_RECORDING:
            return "Pre-recording";
        default:
            return "Unknown status";
    }
}

const char* video_resolution_to_string(video_resolution_t res) {
    switch (res) {
        case VIDEO_RESOLUTION_1080P: return "1920x1080P";
        case VIDEO_RESOLUTION_4K_16_9: return "4096x2160P 4K 16:9";
        case VIDEO_RESOLUTION_2K_16_9: return "2720x1530P 2.7K 16:9";
        case VIDEO_RESOLUTION_1080P_9_16: return "1920x1080P 9:16";
        case VIDEO_RESOLUTION_2K_9_16: return "2720x1530P 9:16";
        case VIDEO_RESOLUTION_2K_4_3: return "2720x2040P 2.7K 4:3";
        case VIDEO_RESOLUTION_4K_4_3: return "4096x3072P 4K 4:3";
        case VIDEO_RESOLUTION_4K_9_16: return "4096x2160P 4K 9:16";
        case VIDEO_RESOLUTION_L: return "Ultra Wide 30MP (Osmo360)";
        case VIDEO_RESOLUTION_M: return "Wide 20MP (Osmo360)";
        case VIDEO_RESOLUTION_S: return "Standard 12MP (Osmo360)";
        default: return "Unknown Resolution";
    }
}

const char* fps_idx_to_string(fps_idx_t fps) {
    switch (fps) {
        case FPS_24: return "24fps";
        case FPS_25: return "25fps";
        case FPS_30: return "30fps";
        case FPS_48: return "48fps";
        case FPS_50: return "50fps";
        case FPS_60: return "60fps";
        case FPS_100: return "100fps";
        case FPS_120: return "120fps";
        case FPS_200: return "200fps";
        case FPS_240: return "240fps";
        default: return "Unknown FPS";
    }
}

const char* eis_mode_to_string(eis_mode_t mode) {
    switch (mode) {
        case EIS_MODE_OFF: return "Off";
        case EIS_MODE_RS: return "RS";
        case EIS_MODE_RS_PLUS: return "RS+";
        case EIS_MODE_HB: return "HB";
        case EIS_MODE_HS: return "HS";
        default: return "Unknown EIS mode";
    }
}
