/****************************************************************************
 * cnn.h
 *
 *   Copyright (C) 2017 Leye Wang. All rights reserved.
 *   Author: Leye Wang<leye.wang@hobot.cc>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_CNN_H
#define __INCLUDE_NUTTX_CNN_H

#include <mqueue.h>
#include <pthread.h>
#include <stdint.h>

//#define DEBUG_ADAS

/****************************************************************************
 * Pre-Process Definitions
 ****************************************************************************/

/* operations to get information from merge output */
#define GET_RESP_LEFT(r)        ((r).data[0] >> 20)
#define GET_RESP_TOP(r)         ((r).data[1] >> 20)
#define GET_RESP_WIDTH(r)       ((r).data[0] & 0xFFF)
#define GET_RESP_HEIGHT(r)      ((r).data[1] & 0xFFF)
#define GET_RESP_SCORE(r)       (((r).data[0] >> 12) & 0xFF)
#define GET_RESP_MODEL_ID(r)    (((r).data[1] >> 12) & 0xFF)

/* CNN tensor for post verify, get from feature map */
#define SET_TENSOR_LSW(l,s,w)   (((l)<<20) | ((s)<<12) | ((w)&0xFFF))
#define SET_TENSOR_TIH(t,i,h)   (((t)<<20) | ((i)<<12) | ((h)&0xFFF))

/* CNN configurations, score and NMS threshold */
#define CONTAIN_NUMER(n)        (((n)>>16)&0xFF)
#define OVERLAP_NUMER(n)        (((n)>>8)&0xFF)
#define OVERLAP_DENOM(n)        ((n)&0xFF)

/* cnn function call resizer flag */
#define RSZ_FLAG_ISBYPASS       (1<<3) /* bypass resizer */
#define RSZ_FLAG_Y_ENABLE       (1<<2) /* enable Y channel resizing */
#define RSZ_FLAG_UV_ENABLE      (1<<1) /* enable UV channel resizing */
#define RSZ_FLAG_PAD_ZERO       (1<<0) /* pad zero, otherwise nearest pixel */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the detection response header information */
typedef struct det_rsp_head_s
{
  uint32_t frame_id[4];           /* frame id */
  uint16_t resp_num[8];           /* response counts of each model */
}det_rsp_h_t;

/* This structure describes detection response packed information
 * left:   data[0] [31:20] s12
 * top:    data[1] [31:20] s12
 * width:  data[0] [11:0]  u12
 * height: data[1] [11:0]  u12
 * score:  data[0] [19:12] u8
 * model:  data[1] [14:12] u3
 */
typedef struct det_rsp_s
{
  int32_t data[2];
}det_rsp_t;

/* CNN tensor for post verify, get from feature map */
typedef det_rsp_t cnn_tensor_t;

/* This structure describes information to calculate cnn output */
typedef struct cnn_cfg_s
{
  int32_t  cnn_score; /* threshold for select cnn output */
  uint32_t nms_param; /* nms parameters */
} cnn_cfg_t;

/* This structure describes function call information */
typedef struct cnn_fc_s
{
  uint8_t  reserve[13];       /* bit [103:0] */
  uint8_t  rsz_flags;         /* bit [111:104] */
  uint8_t  dst_height;        /* bit [119:112] */
  uint8_t  dst_width;         /* bit [127:120] */
  int16_t  uv_src_bottom;     /* bit [143:128] valid 13[140:128] */
  int16_t  uv_src_right;      /* bit [159:144] valid 13[156:144] */
  int16_t  uv_src_top;        /* bit [175:160] valid 13[172:160] */
  int16_t  uv_src_left;       /* bit [191:176] valid 13[188:176] */
  uint16_t uv_src_stride;     /* bit [207:192] valid 12[203:192] */
  uint16_t uv_src_height;     /* bit [223:208] valid 12[219:208] */
  uint16_t uv_src_width;      /* bit [239:224] valid 12[235:224] */
  int16_t  y_src_bottom;      /* bit [255:240] valid 13[252:240] */
  int16_t  y_src_right;       /* bit [271:256] valid 13[268:256] */
  int16_t  y_src_top;         /* bit [287:272] valid 13[284:272] */
  int16_t  y_src_left;        /* bit [303:288] valid 13[300:288] */
  uint16_t y_src_stride;      /* bit [319:304] valid 12[315:304] */
  uint16_t y_src_height;      /* bit [335:320] valid 12[331:320] */
  uint16_t y_src_width;       /* bit [351:336] valid 12[347:336] */
  uint32_t uv_src_addr;       /* bit [415:384] */
  uint32_t y_src_addr;        /* bit [447:416] */
  uint16_t interrupt_num;     /* bit [367:352] valid  6[421:416] */
  uint16_t model_inst_len;    /* bit [383:368] valid 10[441:432] */
  uint32_t dyn_base_addr;     /* bit [479:448] */
  uint32_t model_inst_addr;   /* bit [511:480] */
} cnn_fc_t;

/* This structure describes batch function call information for each
 * detection model, aligned to 16 Bytes */
typedef struct fc_batch_s
{
#ifdef DEBUG_ADAS
  uint8_t det_id;
    uint8_t cnn_id;
    uint8_t int_nr;
    uint8_t fc_num;
    uint32_t roi_base;
    uint32_t out_base;
    uint32_t rst_base;
#else
  uint8_t  md_id;             /* Model ID */
  uint8_t  int_nr;            /* FC interrupt NO */
  uint8_t  fc_num;            /* FC number in batch */
  uint8_t  tensor_num;        /* output tensor number */
  uintptr_t roi_info_base;     /* fc_roi_t base for all FC */
  uintptr_t output_base;       /* dyn_base_addr base for all FC */
  uintptr_t tensor_base;       /* NMS output tensor base address */
#endif
} fc_batch_t;

/* This structure bounding box (left, top, right, bottom) */
typedef struct bounding_box_s
{
  int16_t l;
  int16_t t;
  int16_t r;
  int16_t b;
} b_box_t;

/* This structure describes CNN ROI info for decode CNN result */
typedef struct roi_info_s
{
  b_box_t roi;
  uint32_t scale_r;     /* obj_h / norm_len */
  uint32_t reserve;
} roi_info_t;

/* This structure describes tensor link for store addr & conf for
 * parsed tensor */
typedef struct tensor_node_s
{
  struct tensor_node_s *next;
  int32_t conf;
  b_box_t bbox;
} tensor_node_t;

/* This structure describes cnn fcq config information */
typedef struct cnn_fcq_info_s
{
  uint32_t fcq_base;    /* function call queue base address */
  uint32_t fcq_len;     /* function call queue length */
  uint32_t tail;        /* tail of function call queue */
  uint32_t left;        /* left space of function call queue */
} cnn_fcq_info_t;

/* This structure describes config information to enqueue function calls from user space */
typedef struct cnn_config_s
{
  uint32_t fcq_base;   /* function call base address in user space */
  uint32_t fc_cnt;     /* function call numbers */
} cnn_config_t;




#endif    /* __INCLUDE_NUTTX_CNN_H */

