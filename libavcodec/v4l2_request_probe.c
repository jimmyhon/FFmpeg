/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <libudev.h>

#include "libavutil/hwcontext_v4l2request.h"
#include "libavutil/mem.h"
#include "v4l2_request_internal.h"

static const struct {
    uint32_t pixelformat;
    enum AVPixelFormat sw_format;
    uint32_t drm_format;
    uint64_t format_modifier;
} v4l2_request_capture_pixelformats[] = {
    { V4L2_PIX_FMT_NV12, AV_PIX_FMT_NV12, DRM_FORMAT_NV12, DRM_FORMAT_MOD_LINEAR },
#if defined(V4L2_PIX_FMT_NV12_32L32)
    { V4L2_PIX_FMT_NV12_32L32, AV_PIX_FMT_NONE, DRM_FORMAT_NV12, DRM_FORMAT_MOD_ALLWINNER_TILED },
#endif
#if defined(V4L2_PIX_FMT_NV15) && defined(DRM_FORMAT_NV15)
    { V4L2_PIX_FMT_NV15, AV_PIX_FMT_NONE, DRM_FORMAT_NV15, DRM_FORMAT_MOD_LINEAR },
#endif
    { V4L2_PIX_FMT_NV16, AV_PIX_FMT_NV16, DRM_FORMAT_NV16, DRM_FORMAT_MOD_LINEAR },
#if defined(V4L2_PIX_FMT_NV20) && defined(DRM_FORMAT_NV20)
    { V4L2_PIX_FMT_NV20, AV_PIX_FMT_NONE, DRM_FORMAT_NV20, DRM_FORMAT_MOD_LINEAR },
#endif
#if defined(V4L2_PIX_FMT_P010) && defined(DRM_FORMAT_P010)
    { V4L2_PIX_FMT_P010, AV_PIX_FMT_P010, DRM_FORMAT_P010, DRM_FORMAT_MOD_LINEAR },
#endif
#if defined(V4L2_PIX_FMT_NV12_COL128) && defined(V4L2_PIX_FMT_NV12_10_COL128)
    {
        .pixelformat = V4L2_PIX_FMT_NV12_COL128,
        .sw_format = AV_PIX_FMT_NONE,
        .drm_format = DRM_FORMAT_NV12,
        .format_modifier = DRM_FORMAT_MOD_BROADCOM_SAND128,
    },
#if defined(DRM_FORMAT_P030)
    {
        .pixelformat = V4L2_PIX_FMT_NV12_10_COL128,
        .sw_format = AV_PIX_FMT_NONE,
        .drm_format = DRM_FORMAT_P030,
        .format_modifier = DRM_FORMAT_MOD_BROADCOM_SAND128,
    },
#endif
#endif
#if defined(V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT)
    {
        .pixelformat = V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT,
        .sw_format = AV_PIX_FMT_NONE,
        .drm_format = DRM_FORMAT_YUV420_10BIT,
        .format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
                                                   AFBC_FORMAT_MOD_SPARSE |
                                                   AFBC_FORMAT_MOD_SPLIT),
    },
#endif
#if defined(V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT)
    {
        .pixelformat = V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT,
        .sw_format = AV_PIX_FMT_NONE,
        .drm_format = DRM_FORMAT_YUV420_8BIT,
        .format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
                                                   AFBC_FORMAT_MOD_SPARSE |
                                                   AFBC_FORMAT_MOD_SPLIT),
    },
#endif
};

enum AVPixelFormat ff_v4l2_request_get_sw_format(struct v4l2_format *format)
{
    uint32_t pixelformat = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                           format->fmt.pix_mp.pixelformat :
                           format->fmt.pix.pixelformat;

    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        if (pixelformat == v4l2_request_capture_pixelformats[i].pixelformat)
            return v4l2_request_capture_pixelformats[i].sw_format;
    }

    return AV_PIX_FMT_NONE;
}

int ff_v4l2_request_set_drm_descriptor(V4L2RequestFrameDescriptor *framedesc,
                                       struct v4l2_format *format)
{
    AVDRMFrameDescriptor *desc = &framedesc->base;
    AVDRMLayerDescriptor *layer = &desc->layers[0];
    uint32_t pixelformat = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                           format->fmt.pix_mp.pixelformat :
                           format->fmt.pix.pixelformat;

    // Set drm format and format modifier
    layer->format = 0;
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        if (pixelformat == v4l2_request_capture_pixelformats[i].pixelformat) {
            layer->format = v4l2_request_capture_pixelformats[i].drm_format;
            desc->objects[0].format_modifier =
                        v4l2_request_capture_pixelformats[i].format_modifier;
            break;
        }
    }

    if (!layer->format)
        return AVERROR(ENOENT);

    desc->nb_objects = 1;
    desc->objects[0].fd = framedesc->capture.fd;
    desc->objects[0].size = framedesc->capture.size;

    desc->nb_layers = 1;
    layer->nb_planes = 1;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                             format->fmt.pix_mp.plane_fmt[0].bytesperline :
                             format->fmt.pix.bytesperline;

    // AFBC formats only use 1 plane, remaining use 2 planes
    if ((desc->objects[0].format_modifier >> 56) != DRM_FORMAT_MOD_VENDOR_ARM) {
        layer->nb_planes = 2;
        layer->planes[1].object_index = 0;
        layer->planes[1].offset = layer->planes[0].pitch *
                                  (V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                   format->fmt.pix_mp.height :
                                   format->fmt.pix.height);
        layer->planes[1].pitch = layer->planes[0].pitch;
    }

#if defined(V4L2_PIX_FMT_NV12_COL128) && defined(V4L2_PIX_FMT_NV12_10_COL128)
    // Raspberry Pi formats need special handling
    if (pixelformat == V4L2_PIX_FMT_NV12_COL128 ||
        pixelformat == V4L2_PIX_FMT_NV12_10_COL128) {
        desc->objects[0].format_modifier =
            DRM_FORMAT_MOD_BROADCOM_SAND128_COL_HEIGHT(layer->planes[0].pitch);
        layer->planes[1].offset = 128 *
                                  (V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                   format->fmt.pix_mp.height :
                                   format->fmt.pix.height);
        layer->planes[0].pitch = (V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                  format->fmt.pix_mp.width :
                                  format->fmt.pix.width);
        if (pixelformat == V4L2_PIX_FMT_NV12_10_COL128)
            layer->planes[0].pitch *= 2;
        layer->planes[1].pitch = layer->planes[0].pitch;
    }
#endif

    return 0;
}

static int v4l2_request_set_format(AVCodecContext *avctx,
                                   enum v4l2_buf_type type,
                                   uint32_t pixelformat,
                                   uint32_t buffersize)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_format format = {
        .type = type,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        format.fmt.pix_mp.width = avctx->coded_width;
        format.fmt.pix_mp.height = avctx->coded_height;
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = buffersize;
        format.fmt.pix_mp.num_planes = 1;
    } else {
        format.fmt.pix.width = avctx->coded_width;
        format.fmt.pix.height = avctx->coded_height;
        format.fmt.pix.pixelformat = pixelformat;
        format.fmt.pix.sizeimage = buffersize;
    }

    if (ioctl(ctx->video_fd, VIDIOC_S_FMT, &format) < 0)
        return AVERROR(errno);

    return 0;
}

static int v4l2_request_select_capture_format(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    enum v4l2_buf_type type = ctx->format.type;
    struct v4l2_format format = {
        .type = type,
    };
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };
    uint32_t pixelformat;

    // Get the driver preferred (or default) format
    if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &format) < 0)
        return AVERROR(errno);

    pixelformat = V4L2_TYPE_IS_MULTIPLANAR(type) ?
                  format.fmt.pix_mp.pixelformat :
                  format.fmt.pix.pixelformat;

    // Use the driver preferred format when it is supported
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        if (pixelformat == v4l2_request_capture_pixelformats[i].pixelformat)
            return v4l2_request_set_format(avctx, type, pixelformat, 0);
    }

    // Otherwise, use first format that is supported
    while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
            if (fmtdesc.pixelformat == v4l2_request_capture_pixelformats[i].pixelformat)
                return v4l2_request_set_format(avctx, type, fmtdesc.pixelformat, 0);
        }

        fmtdesc.index++;
    }

    return AVERROR(errno);
}

static int v4l2_request_try_framesize(AVCodecContext *avctx,
                                      uint32_t pixelformat)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_frmsizeenum frmsize = {
        .index = 0,
        .pixel_format = pixelformat,
    };

    // Enumerate and check if frame size is supported
    while (ioctl(ctx->video_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0) {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
            frmsize.discrete.width == avctx->coded_width &&
            frmsize.discrete.height == avctx->coded_height) {
            return 0;
        } else if ((frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE ||
                    frmsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) &&
                   avctx->coded_width <= frmsize.stepwise.max_width &&
                   avctx->coded_height <= frmsize.stepwise.max_height) {
            return 0;
        }

        frmsize.index++;
    }

    return AVERROR(errno);
}

static int v4l2_request_try_format(AVCodecContext *avctx,
                                   enum v4l2_buf_type type,
                                   uint32_t pixelformat)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    // Enumerate and check if format is supported
    while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        if (fmtdesc.pixelformat == pixelformat)
            return 0;

        fmtdesc.index++;
    }

    return AVERROR(errno);
}

static int v4l2_request_probe_video_device(const char *path,
                                           AVCodecContext *avctx,
                                           uint32_t pixelformat,
                                           uint32_t buffersize,
                                           struct v4l2_ext_control *control,
                                           int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_capability capability;
    struct v4l2_create_buffers buffers = {
        .count = 0,
        .memory = V4L2_MEMORY_MMAP,
    };
    unsigned int capabilities;
    int ret;

    /*
     * Open video device in non-blocking mode to support decoding using
     * multiple queued requests, required for e.g. multi stage decoding.
     */
    ctx->video_fd = open(path, O_RDWR | O_NONBLOCK);
    if (ctx->video_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to open video device %s: %s (%d)\n",
               path, strerror(errno), errno);
        return AVERROR(errno);
    }

    // Query capabilities of the video device
    if (ioctl(ctx->video_fd, VIDIOC_QUERYCAP, &capability) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to query capabilities of %s: %s (%d)\n",
               path, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    // Use device capabilities when needed
    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
        capabilities = capability.device_caps;
    else
        capabilities = capability.capabilities;

    // Ensure streaming is supported on the video device
    if ((capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING) {
        av_log(ctx, AV_LOG_VERBOSE, "Device %s is missing streaming capability\n", path);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // Ensure multi- or single-planar API can be used
    if ((capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) == V4L2_CAP_VIDEO_M2M_MPLANE) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if ((capabilities & V4L2_CAP_VIDEO_M2M) == V4L2_CAP_VIDEO_M2M) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        av_log(ctx, AV_LOG_VERBOSE, "Device %s is missing mem2mem capability\n", path);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // Query output buffer capabilities
    buffers.format.type = ctx->output_type;
    if (ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to query output buffer capabilities of %s: %s (%d)\n",
               path, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    // Ensure requests can be used
    if ((buffers.capabilities & V4L2_BUF_CAP_SUPPORTS_REQUESTS) !=
        V4L2_BUF_CAP_SUPPORTS_REQUESTS) {
        av_log(ctx, AV_LOG_VERBOSE, "Device %s is missing support for requests\n", path);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // Ensure the codec pixelformat can be used
    ret = v4l2_request_try_format(avctx, ctx->output_type, pixelformat);
    if (ret < 0) {
        av_log(ctx, AV_LOG_VERBOSE, "Device %s is missing support for pixelformat %s\n",
               path, av_fourcc2str(pixelformat));
        goto fail;
    }

    // Ensure frame size is supported, when driver support ENUM_FRAMESIZES
    ret = v4l2_request_try_framesize(avctx, pixelformat);
    if (ret < 0 && ret != AVERROR(ENOTTY)) {
        av_log(ctx, AV_LOG_VERBOSE,
               "Device %s is missing support for frame size %dx%d of pixelformat %s\n",
               path, avctx->coded_width, avctx->coded_height, av_fourcc2str(pixelformat));
        goto fail;
    }

    // Set the codec pixelformat and output buffersize to be used
    ret = v4l2_request_set_format(avctx, ctx->output_type, pixelformat, buffersize);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set output pixelformat %s of %s: %s (%d)\n",
               av_fourcc2str(pixelformat), path, strerror(errno), errno);
        goto fail;
    }

    /*
     * Set any codec specific controls that can help assist the driver
     * make a decision on what capture buffer format can be used.
     */
    ret = ff_v4l2_request_set_controls(avctx, control, count);
    if (ret < 0)
        goto fail;

    // Select a capture buffer format known to the hwaccel
    ret = v4l2_request_select_capture_format(avctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_VERBOSE,
               "Failed to select a capture format for %s of %s: %s (%d)\n",
               av_fourcc2str(pixelformat), path, strerror(errno), errno);
        goto fail;
    }

    // Check codec specific controls, e.g. profile and level
    if (ctx->post_probe) {
        ret = ctx->post_probe(avctx);
        if (ret < 0)
            goto fail;
    }

    // All tests passed, video device should be capable
    return 0;

fail:
    if (ctx->video_fd >= 0) {
        close(ctx->video_fd);
        ctx->video_fd = -1;
    }
    return ret;
}

static int v4l2_request_probe_video_devices(struct udev *udev,
                                            AVCodecContext *avctx,
                                            uint32_t pixelformat,
                                            uint32_t buffersize,
                                            struct v4l2_ext_control *control,
                                            int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct media_device_info device_info;
    struct media_v2_topology topology = {0};
    struct media_v2_interface *interfaces;
    struct udev_device *device;
    const char *path;
    dev_t devnum;
    int ret;

    if (ioctl(ctx->media_fd, MEDIA_IOC_DEVICE_INFO, &device_info) < 0)
        return AVERROR(errno);

    if (ioctl(ctx->media_fd, MEDIA_IOC_G_TOPOLOGY, &topology) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to get media topology: %s (%d)\n",
               strerror(errno), errno);
        return AVERROR(errno);
    }

    if (!topology.num_interfaces)
        return AVERROR(ENOENT);

    interfaces = av_calloc(topology.num_interfaces, sizeof(struct media_v2_interface));
    if (!interfaces)
        return AVERROR(ENOMEM);

    topology.ptr_interfaces = (__u64)(uintptr_t)interfaces;
    if (ioctl(ctx->media_fd, MEDIA_IOC_G_TOPOLOGY, &topology) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to get media topology: %s (%d)\n",
               strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    ret = AVERROR(ENOENT);
    for (int i = 0; i < topology.num_interfaces; i++) {
        if (interfaces[i].intf_type != MEDIA_INTF_T_V4L_VIDEO)
            continue;

        devnum = makedev(interfaces[i].devnode.major, interfaces[i].devnode.minor);
        device = udev_device_new_from_devnum(udev, 'c', devnum);
        if (!device)
            continue;

        path = udev_device_get_devnode(device);
        if (path)
            ret = v4l2_request_probe_video_device(path, avctx, pixelformat,
                                                  buffersize, control, count);
        udev_device_unref(device);

        // Stop when we have found a capable video device
        if (!ret) {
            av_log(avctx, AV_LOG_INFO,
                   "Using V4L2 media driver %s (%u.%u.%u) for %s\n",
                   device_info.driver,
                   device_info.driver_version >> 16,
                   (device_info.driver_version >> 8) & 0xff,
                   device_info.driver_version & 0xff,
                   av_fourcc2str(pixelformat));
            break;
        }
    }

fail:
    av_free(interfaces);
    return ret;
}

static int v4l2_request_probe_media_device(struct udev_device *device,
                                           AVCodecContext *avctx,
                                           uint32_t pixelformat,
                                           uint32_t buffersize,
                                           struct v4l2_ext_control *control,
                                           int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    const char *path;
    int ret;

    path = udev_device_get_devnode(device);
    if (!path)
        return AVERROR(ENODEV);

    // Open enumerated media device
    ctx->media_fd = open(path, O_RDWR);
    if (ctx->media_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to open media device %s: %s (%d)\n",
               path, strerror(errno), errno);
        return AVERROR(errno);
    }

    // Probe video devices of current media device
    ret = v4l2_request_probe_video_devices(udev_device_get_udev(device),
                                           avctx, pixelformat,
                                           buffersize, control, count);

    // Cleanup when no capable video device was found
    if (ret < 0) {
        close(ctx->media_fd);
        ctx->media_fd = -1;
    }

    return ret;
}

static int v4l2_request_probe_media_devices(struct udev *udev,
                                            AVCodecContext *avctx,
                                            uint32_t pixelformat,
                                            uint32_t buffersize,
                                            struct v4l2_ext_control *control,
                                            int count)
{
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *entry;
    struct udev_device *device;
    int ret;

    enumerate = udev_enumerate_new(udev);
    if (!enumerate)
        return AVERROR(ENOMEM);

    udev_enumerate_add_match_subsystem(enumerate, "media");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    ret = AVERROR(ENOENT);
    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        if (!path)
            continue;

        device = udev_device_new_from_syspath(udev, path);
        if (!device)
            continue;

        // Probe media device for a capable video device
        ret = v4l2_request_probe_media_device(device, avctx, pixelformat,
                                              buffersize, control, count);
        udev_device_unref(device);

        // Stop when we have found a capable media and video device
        if (!ret)
            break;
    }

    udev_enumerate_unref(enumerate);
    return ret;
}

int ff_v4l2_request_probe(AVCodecContext *avctx,
                          uint32_t pixelformat, uint32_t buffersize,
                          struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct udev *udev;
    int ret;

    udev = udev_new();
    if (!udev)
        return AVERROR(ENOMEM);

    if (ctx->media_fd >= 0) {
        // Probe video devices of current media device
        ret = v4l2_request_probe_video_devices(udev, avctx, pixelformat,
                                               buffersize, control, count);
    } else {
        // Probe all media devices (auto-detect)
        ret = v4l2_request_probe_media_devices(udev, avctx, pixelformat,
                                               buffersize, control, count);
    }

    udev_unref(udev);
    return ret;
}
