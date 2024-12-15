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

#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "v4l2_request.h"
#include "vp8.h"

typedef struct V4L2RequestControlsVP8 {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_vp8_frame frame;
} V4L2RequestControlsVP8;

static int v4l2_request_vp8_start_frame(AVCodecContext *avctx,
                                        const uint8_t *buffer,
                                        av_unused uint32_t size)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP8_FRAME_CURRENT]->hwaccel_picture_private;
    struct v4l2_ctrl_vp8_frame *ctrl = &controls->frame;
    unsigned int header_size = 3 + 7 * s->keyframe;
    const uint8_t *data = buffer + header_size;
    int ret, i, j, k;

    ret = ff_v4l2_request_start_frame(avctx, &controls->pic,
                                      s->framep[VP8_FRAME_CURRENT]->tf.f);
    if (ret)
        return ret;

    *ctrl = (struct v4l2_ctrl_vp8_frame) {
        .lf = {
            .sharpness_level = s->filter.sharpness,
            .level = s->filter.level,
        },

        .quant = {
            .y_ac_qi = s->quant.yac_qi,
            .y_dc_delta = s->quant.ydc_delta,
            .y2_dc_delta = s->quant.y2dc_delta,
            .y2_ac_delta = s->quant.y2ac_delta,
            .uv_dc_delta = s->quant.uvdc_delta,
            .uv_ac_delta = s->quant.uvac_delta,
        },

        .coder_state = {
            .range = s->coder_state_at_header_end.range,
            .value = s->coder_state_at_header_end.value,
            .bit_count = s->coder_state_at_header_end.bit_count,
        },

        .width = avctx->width,
        .height = avctx->height,

        .horizontal_scale = 0, /* scale not supported by FFmpeg */
        .vertical_scale = 0, /* scale not supported by FFmpeg */

        .version = s->profile & 0x3,
        .prob_skip_false = s->prob->mbskip,
        .prob_intra = s->prob->intra,
        .prob_last = s->prob->last,
        .prob_gf = s->prob->golden,
        .num_dct_parts = s->num_coeff_partitions,

        .first_part_size = s->header_partition_size,
        .first_part_header_bits = (8 * (s->coder_state_at_header_end.input - data) -
                                   s->coder_state_at_header_end.bit_count - 8),
    };

    for (i = 0; i < 4; i++) {
        ctrl->segment.quant_update[i] = s->segmentation.base_quant[i];
        ctrl->segment.lf_update[i] = s->segmentation.filter_level[i];
    }

    for (i = 0; i < 3; i++)
        ctrl->segment.segment_probs[i] = s->prob->segmentid[i];

    if (s->segmentation.enabled)
        ctrl->segment.flags |= V4L2_VP8_SEGMENT_FLAG_ENABLED;

    if (s->segmentation.update_map)
        ctrl->segment.flags |= V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP;

    if (s->segmentation.update_feature_data)
        ctrl->segment.flags |= V4L2_VP8_SEGMENT_FLAG_UPDATE_FEATURE_DATA;

    if (!s->segmentation.absolute_vals)
        ctrl->segment.flags |= V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE;

    for (i = 0; i < 4; i++) {
        ctrl->lf.ref_frm_delta[i] = s->lf_delta.ref[i];
        ctrl->lf.mb_mode_delta[i] = s->lf_delta.mode[i + MODE_I4x4];
    }

    if (s->lf_delta.enabled)
        ctrl->lf.flags |= V4L2_VP8_LF_ADJ_ENABLE;

    if (s->lf_delta.update)
        ctrl->lf.flags |= V4L2_VP8_LF_DELTA_UPDATE;

    if (s->filter.simple)
        ctrl->lf.flags |= V4L2_VP8_LF_FILTER_TYPE_SIMPLE;

    if (s->keyframe) {
        static const uint8_t keyframe_y_mode_probs[4] = {
            145, 156, 163, 128
        };
        static const uint8_t keyframe_uv_mode_probs[3] = {
            142, 114, 183
        };

        memcpy(ctrl->entropy.y_mode_probs, keyframe_y_mode_probs, 4);
        memcpy(ctrl->entropy.uv_mode_probs, keyframe_uv_mode_probs, 3);
    } else {
        for (i = 0; i < 4; i++)
            ctrl->entropy.y_mode_probs[i] = s->prob->pred16x16[i];
        for (i = 0; i < 3; i++)
            ctrl->entropy.uv_mode_probs[i] = s->prob->pred8x8c[i];
    }
    for (i = 0; i < 2; i++)
        for (j = 0; j < 19; j++)
            ctrl->entropy.mv_probs[i][j] = s->prob->mvc[i][j];

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 8; j++) {
            static const int coeff_bands_inverse[8] = {
                0, 1, 2, 3, 5, 6, 4, 15
            };
            int coeff_pos = coeff_bands_inverse[j];

            for (k = 0; k < 3; k++) {
                memcpy(ctrl->entropy.coeff_probs[i][j][k],
                       s->prob->token[i][coeff_pos][k], 11);
            }
        }
    }

    for (i = 0; i < 8; i++)
        ctrl->dct_part_sizes[i] = s->coeff_partition_size[i];

    if (s->framep[VP8_FRAME_PREVIOUS])
        ctrl->last_frame_ts =
            ff_v4l2_request_get_capture_timestamp(s->framep[VP8_FRAME_PREVIOUS]->tf.f);
    if (s->framep[VP8_FRAME_GOLDEN])
        ctrl->golden_frame_ts =
            ff_v4l2_request_get_capture_timestamp(s->framep[VP8_FRAME_GOLDEN]->tf.f);
    if (s->framep[VP8_FRAME_ALTREF])
        ctrl->alt_frame_ts =
            ff_v4l2_request_get_capture_timestamp(s->framep[VP8_FRAME_ALTREF]->tf.f);

    if (s->keyframe)
        ctrl->flags |= V4L2_VP8_FRAME_FLAG_KEY_FRAME;

    if (s->profile & 0x4)
        ctrl->flags |= V4L2_VP8_FRAME_FLAG_EXPERIMENTAL;

    if (!s->invisible)
        ctrl->flags |= V4L2_VP8_FRAME_FLAG_SHOW_FRAME;

    if (s->mbskip_enabled)
        ctrl->flags |= V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF;

    if (s->sign_bias[VP8_FRAME_GOLDEN])
        ctrl->flags |= V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN;

    if (s->sign_bias[VP8_FRAME_ALTREF])
        ctrl->flags |= V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT;

    return 0;
}

static int v4l2_request_vp8_decode_slice(AVCodecContext *avctx,
                                         const uint8_t *buffer, uint32_t size)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP8_FRAME_CURRENT]->hwaccel_picture_private;

    return ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
}

static int v4l2_request_vp8_end_frame(AVCodecContext *avctx)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP8_FRAME_CURRENT]->hwaccel_picture_private;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_VP8_FRAME,
            .ptr = &controls->frame,
            .size = sizeof(controls->frame),
        },
    };

    return ff_v4l2_request_decode_frame(avctx, &controls->pic,
                                        control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_vp8_init(AVCodecContext *avctx)
{
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_VP8_FRAME,
                                2 * 1024 * 1024,
                                NULL, 0);
}

const FFHWAccel ff_vp8_v4l2request_hwaccel = {
    .p.name             = "vp8_v4l2request",
    .p.type             = AVMEDIA_TYPE_VIDEO,
    .p.id               = AV_CODEC_ID_VP8,
    .p.pix_fmt          = AV_PIX_FMT_DRM_PRIME,
    .start_frame        = v4l2_request_vp8_start_frame,
    .decode_slice       = v4l2_request_vp8_decode_slice,
    .end_frame          = v4l2_request_vp8_end_frame,
    .flush              = ff_v4l2_request_flush,
    .frame_priv_data_size = sizeof(V4L2RequestControlsVP8),
    .init               = v4l2_request_vp8_init,
    .uninit             = ff_v4l2_request_uninit,
    .priv_data_size     = sizeof(V4L2RequestContext),
    .frame_params       = ff_v4l2_request_frame_params,
};
