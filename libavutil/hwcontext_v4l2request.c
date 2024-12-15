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
#include <linux/dma-buf.h>
#include <linux/media.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "avassert.h"
#include "hwcontext_drm.h"
#include "hwcontext_internal.h"
#include "hwcontext_v4l2request.h"
#include "mem.h"

static void v4l2request_device_free(AVHWDeviceContext *hwdev)
{
    AVV4L2RequestDeviceContext *hwctx = hwdev->hwctx;

    if (hwctx->media_fd >= 0) {
        close(hwctx->media_fd);
        hwctx->media_fd = -1;
    }
}

static int v4l2request_device_create(AVHWDeviceContext *hwdev, const char *device,
                                     AVDictionary *opts, int flags)
{
    AVV4L2RequestDeviceContext *hwctx = hwdev->hwctx;

    hwctx->media_fd = -1;
    hwdev->free = v4l2request_device_free;

    // Use auto-detect
    if (!device || !device[0])
        return 0;

    hwctx->media_fd = open(device, O_RDWR);
    if (hwctx->media_fd < 0)
        return AVERROR(errno);

    return 0;
}

static int v4l2request_device_init(AVHWDeviceContext *hwdev)
{
    AVV4L2RequestDeviceContext *hwctx = hwdev->hwctx;
    struct media_device_info device_info;

    // Use auto-detect
    if (hwctx->media_fd < 0)
        return 0;

    if (ioctl(hwctx->media_fd, MEDIA_IOC_DEVICE_INFO, &device_info) < 0)
        return AVERROR(errno);

    av_log(hwdev, AV_LOG_VERBOSE, "Using V4L2 media driver %s (%u.%u.%u)\n",
           device_info.driver,
           device_info.driver_version >> 16,
           (device_info.driver_version >> 8) & 0xff,
           device_info.driver_version & 0xff);

    return 0;
}

static int v4l2request_get_buffer(AVHWFramesContext *hwfc, AVFrame *frame)
{
    frame->buf[0] = av_buffer_pool_get(hwfc->pool);
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    frame->data[0] = (uint8_t *)frame->buf[0]->data;

    frame->format = AV_PIX_FMT_DRM_PRIME;
    frame->width  = hwfc->width;
    frame->height = hwfc->height;

    return 0;
}

typedef struct DRMMapping {
    // Address and length of each mmap()ed region.
    int nb_regions;
    int object[AV_DRM_MAX_PLANES];
    void *address[AV_DRM_MAX_PLANES];
    size_t length[AV_DRM_MAX_PLANES];
} DRMMapping;

static void v4l2request_unmap_frame(AVHWFramesContext *hwfc,
                                    HWMapDescriptor *hwmap)
{
    DRMMapping *map = hwmap->priv;

    for (int i = 0; i < map->nb_regions; i++) {
        struct dma_buf_sync sync = {
            .flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ,
        };
        ioctl(map->object[i], DMA_BUF_IOCTL_SYNC, &sync);
        munmap(map->address[i], map->length[i]);
    }

    av_free(map);
}

static int v4l2request_map_frame(AVHWFramesContext *hwfc,
                                 AVFrame *dst, const AVFrame *src)
{
    const AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)src->data[0];
    struct dma_buf_sync sync = {
        .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ,
    };
    DRMMapping *map;
    int ret, i, p, plane;
    void *addr;

    map = av_mallocz(sizeof(*map));
    if (!map)
        return AVERROR(ENOMEM);

    av_assert0(desc->nb_objects <= AV_DRM_MAX_PLANES);
    for (i = 0; i < desc->nb_objects; i++) {
        addr = mmap(NULL, desc->objects[i].size, AV_HWFRAME_MAP_READ, MAP_SHARED,
                    desc->objects[i].fd, 0);
        if (addr == MAP_FAILED) {
            av_log(hwfc, AV_LOG_ERROR, "Failed to map DRM object %d to memory: %s (%d)\n",
                   desc->objects[i].fd, strerror(errno), errno);
            ret = AVERROR(errno);
            goto fail;
        }

        map->address[i] = addr;
        map->length[i]  = desc->objects[i].size;
        map->object[i]  = desc->objects[i].fd;

        /*
         * We're not checking for errors here because the kernel may not
         * support the ioctl, in which case its okay to carry on
         */
        ioctl(desc->objects[i].fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    map->nb_regions = i;

    plane = 0;
    for (i = 0; i < desc->nb_layers; i++) {
        const AVDRMLayerDescriptor *layer = &desc->layers[i];
        for (p = 0; p < layer->nb_planes; p++) {
            dst->data[plane] =
                (uint8_t *)map->address[layer->planes[p].object_index] +
                                        layer->planes[p].offset;
            dst->linesize[plane] =      layer->planes[p].pitch;
            ++plane;
        }
    }
    av_assert0(plane <= AV_DRM_MAX_PLANES);

    dst->width  = src->width;
    dst->height = src->height;

    ret = ff_hwframe_map_create(src->hw_frames_ctx, dst, src,
                                v4l2request_unmap_frame, map);
    if (ret < 0)
        goto fail;

    return 0;

fail:
    for (i = 0; i < desc->nb_objects; i++) {
        if (map->address[i])
            munmap(map->address[i], map->length[i]);
    }
    av_free(map);
    return ret;
}

static int v4l2request_transfer_get_formats(AVHWFramesContext *hwfc,
                                            enum AVHWFrameTransferDirection dir,
                                            enum AVPixelFormat **formats)
{
    enum AVPixelFormat *pix_fmts;

    if (dir == AV_HWFRAME_TRANSFER_DIRECTION_TO)
        return AVERROR(ENOSYS);

    pix_fmts = av_malloc_array(2, sizeof(*pix_fmts));
    if (!pix_fmts)
        return AVERROR(ENOMEM);

    pix_fmts[0] = hwfc->sw_format;
    pix_fmts[1] = AV_PIX_FMT_NONE;

    *formats = pix_fmts;
    return 0;
}

static int v4l2request_transfer_data_from(AVHWFramesContext *hwfc,
                                          AVFrame *dst, const AVFrame *src)
{
    AVFrame *map;
    int ret;

    if (dst->width > hwfc->width || dst->height > hwfc->height)
        return AVERROR(EINVAL);

    map = av_frame_alloc();
    if (!map)
        return AVERROR(ENOMEM);
    map->format = dst->format;

    ret = v4l2request_map_frame(hwfc, map, src);
    if (ret)
        goto fail;

    map->width  = dst->width;
    map->height = dst->height;

    ret = av_frame_copy(dst, map);
    if (ret)
        goto fail;

    ret = 0;
fail:
    av_frame_free(&map);
    return ret;
}

const HWContextType ff_hwcontext_type_v4l2request = {
    .type                   = AV_HWDEVICE_TYPE_V4L2REQUEST,
    .name                   = "V4L2 Request API",

    .device_hwctx_size      = sizeof(AVV4L2RequestDeviceContext),
    .device_create          = v4l2request_device_create,
    .device_init            = v4l2request_device_init,

    .frames_get_buffer      = v4l2request_get_buffer,

    .transfer_get_formats   = v4l2request_transfer_get_formats,
    .transfer_data_from     = v4l2request_transfer_data_from,

    .pix_fmts = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_DRM_PRIME,
        AV_PIX_FMT_NONE
    },
};
