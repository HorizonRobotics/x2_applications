/*
 * @Description: tracking algorithm implementation
 * @Author: chang.huang@horizon.ai
 * @Date: 2016-10-8 16:00:20
 * @Author: chao01.yang@horizon.ai
 * @Date: 2019-12-14 16:00:20
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-12-17 17:21:34
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef IPC_TRACKING_TRACKING_DATA_TYPES_H
#define IPC_TRACKING_TRACKING_DATA_TYPES_H

#include <iostream>

#include "nuttx/cnn.h"
#include "nuttx/list.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_MAP_ID 600
#define MAX_OSV_NUM 1
#define MAX_HUNGARIAN_ROW 16 /* align to 4 */

#define IVU_TO_MATCH 51            /* 0.2 * 256 */
#define IVU_TO_MATCH_SAFE_MARGIN 0 /* 0.1 * 256 */

#define HUNGARIAN_NOT_ASSIGNED -1
#define INF 0x7fff
#define TRACK_SCORE_ELIMINATE 10     /* map to 255 */
#define TRACK_SCORE_TO_INITIALIZE 10 /* 126 */

/* hungarian algorithm */
#define HUNGARIAN_MODE_MINIMIZE_COST 0
#define HUNGARIAN_MODE_MAXIMIZE_UTIL 1

/* kalman filtering */
#define KALMAN_Q_VEL 3  /* velocity system noise */
#define KALMAN_Q_POS 10 /* position system noise */
#define KALMAN_R_POS 7  /* observe position noise */

/* tracking information size */
#define TRACK_RESP_SIZE 0x2000

//#define min(a, b) ((a) < (b) ? (a) : (b))
//#define max(a, b) ((a) > (b) ? (a) : (b))
#define set_bit(v, b) ((v) |= 1 << (b))
#define clr_bit(v, b) ((v) &= ~(1 << (b)))

/* set left, score, width, top, model, height */
#define SET_RESP_L(d) ((d) << 20)
#define SET_RESP_S(d) (((d)&0xff) << 12)
#define SET_RESP_W(d) ((d)&0xfff)
#define SET_RESP_T(d) ((d) << 20)
#define SET_RESP_M(d) (((d)&0xff) << 12)
#define SET_RESP_H(d) ((d)&0xfff)

#define SET_DATA_0(l, s, w) (SET_RESP_L(l) | SET_RESP_S(s) | SET_RESP_W(w))
#define SET_DATA_1(t, m, h) (SET_RESP_T(t) | SET_RESP_M(m) | SET_RESP_H(h))

#define FACE_CNN_PV_INDEX (0)
#define HEAD_CNN_PV_INDEX (1)
#define PREDICT_MODEL_NUM 8
#define LMK_POINTS_NUM 10  // number of landmarks

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

//#define TRACK_POST_FUNCTION

typedef enum _frame_status {
  EMPTY_FRAME = 1,
  TARGET_FRAME,
  EXCEPTION_FRAME,
  DROP_FRAME,
} frame_status_t;

/* tracking response head structure */
typedef struct {
  uint8_t time_stamp[16];
  uint8_t track_n[PREDICT_MODEL_NUM];  // count for each model
  uint32_t track_num;                  // total count
} track_rsp_h_t;

/* this structure defines tracking error type */
enum multi_track_error_num {
  TOO_MUCH_BBOX_NUM = 1,
  WITHOUT_MAP_LIST = 2,
  WITHOUT_OSV_LIST = 3,
  TARGET_NUM_FULL = 4,
};

/* this structure describes mapping relations */
typedef struct box_trk_map_s {
  list_h_t list;
  uint32_t id;
  uint16_t ivu;
} box_trk_map_t;

/* this structure describes bouding box information */
typedef struct box_s {
  float left;
  float top;
  float width;
  float height;
  float score;
  int16_t model_id;
  uint32_t index;
} box_t;

/* this structure describes bouding box information */
typedef struct coord_box_s {
  int16_t left;
  int16_t top;
  int16_t right;
  int16_t bottom;
  float score;
  int16_t model_id;
  uint32_t index;
} coord_box_t;

/* this structure describes coordinate point information */
typedef struct coordinate_point_s {
  int16_t left;
  int16_t top;
  int16_t right;
  int16_t bottom;
  uint32_t index;
} coordinate_point_t;

/* this structure describes observed position, predict position,
 * estimated state position, velocity, etc.*/
typedef struct obs_sta_vel_s {
  list_h_t list; /* list of osv_t */
  coordinate_point_t
      observed; /* observed position (left, top, right, bottom) */
  coordinate_point_t
      predict; /* prediction state position (left, top, right, bottom) */
  coordinate_point_t
      estimated; /* estimated state position (left, top, right, bottom) */
  coordinate_point_t velocity; /* velocity (left, top, right, bottom) */
  float weight;                /* weight, detection score */
  uint16_t time_gap_recip;     /* time gap reciprocal to previous one */
  uint16_t kal_p[16];          /* kalman processing */
} osv_t;

/* this structure describes target information */
typedef struct track_target_s {
  list_h_t list;     /* list of track_target_t */
  list_h_t osv_head; /* list head of osv_t */
  uint32_t id;       /* target id */
  uint16_t existing_time_gap;
  float score;
  uint8_t model_num;
  // uint8_t reserve[11];
} track_target_t;

/* this structure used for hungarian algorithm */
typedef struct {
  int16_t num_rows;
  int16_t num_cols;
  int16_t *cost[MAX_HUNGARIAN_ROW];
  int8_t assignment[MAX_HUNGARIAN_ROW];
} hungarian_problem_t;

/* this structure used for kalman filtering processing */
typedef struct {
  uint16_t type;    /* 0 for size unconcerned, 1 for size related */
  int16_t kal_q[2]; /* system noise, 0 for pos, 1 for vel */
  int16_t kal_r;    /* observe pos noise */
} kalman_param_t;

/* tracking response structure */
typedef struct {
  uint32_t trk_id;  // track id
  uint8_t flag;     // whether to do cnn call
  uint8_t md_id;    // det to cnn model id
  uint8_t shift_l;  // shift left length
  coordinate_point_t estimated;
  int32_t fmap[7 + LMK_POINTS_NUM];  // fmap
  int32_t snap_info;
} track_rst_t;

typedef struct det_md_s {
  uint8_t md_mask;    /* describe enabled det model */
  uint8_t mrg_bypass; /* which model should bypass */
  uint16_t reserve;   /* padding */
  uint32_t md_addr;   /* det model addr in DDR */
  int8_t d2c_mask[8]; /* det to cnn mask */
} det_md_t;

#define ALPHACNN_MAX_OUTFMAP_CNT 4
typedef struct cnn_md_s {
  uint32_t input_info; /* model width, height, aspect ratio */
  uint8_t model_type;  /* 0:unknown, 1:post verify, 2:attribute */
  uint8_t split_flag;  /* 0:normal, 1:split */
  uint16_t outstep_KB; /* model output step size align to KB */
  uint8_t norm_method;
  uint8_t norm_length;
  uint8_t nl_recip; /* 1<<nlr_bit / norm_length */
  uint8_t nlr_bit;
  int8_t c2c_mask[4]; /* cnn to cnn mask, -1: no next cnn */
  int32_t cnn_score;  /* threshold for select cnn output */
  uint32_t nms_param; /* nms threshold parameters */
} cnn_md_t;

#define FC_BATCH_CNT 8
#define CNN_POSTV 0

/* CNN tensor for post verify, get from feature map */
typedef det_rsp_t cnn_tensor_t;

typedef void (*track_post_func_t)(struct tracking_module_data *, uint8_t);

typedef struct tracking_module_data {
  /* members below may be allocated in TCM */
  /*****************START**************/
  list_h_t *active_targets; /* allocate PREDICT_MODEL_NUM */
  list_h_t *map_freelist;
  list_h_t *osv_freelist;
  list_h_t *target_freelist;

  int8_t *dst_indices_bk; /* allocate MAX_DET_TARGET_NUM (152) */
  int8_t *dst_indices;    /* allocate MAX_HUNGARIAN_ROW (16) */
  int8_t *src_indices_bk; /* allocate MAX_TRACK_TARGET_NUM (92) */
  int8_t *src_indices;    /* allocate MAX_HUNGARIAN_ROW (16) */

  list_h_t *trk_to_box_aff; /* allocate MAX_TRACK_TARGET_NUM (92) */
  list_h_t *box_to_trk_aff; /* allocate MAX_DET_TARGET_NUM (152) */
  int16_t *trk_to_box;      /* allocate MAX_TRACK_TARGET_NUM (92) */
  int16_t *box_to_trk;      /* allocate MAX_DET_TARGET_NUM (152) */

  hungarian_problem_t *hung_prob;
  list_h_t *clique_box_head;    /* allocate MAX_HUNGARIAN_ROW (16) */
  list_h_t *clique_target_head; /* allocate MAX_HUNGARIAN_ROW (16) */

  int16_t *
      hungarian_cost; /* allocate MAX_HUNGARIAN_ROW * MAX_HUNGARIAN_ROW (256) */
  int16_t *unchosen_row; /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *row_dec;      /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *slack_row;    /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *row_mate;     /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *col_mate;     /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *parent_row;   /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *col_inc;      /* allocate MAX_HUNGARIAN_ROW (16) */
  int16_t *slack;        /* allocate MAX_HUNGARIAN_ROW (16) */

  box_trk_map_t *map_array;     /* allocate MAX_MAP_ID (400) */
  osv_t *osv_array;             /* allocate MAX_OSV_LIST (92) */
  track_target_t *target_array; /* allocate MAX_TRACK_TARGET_NUM (92) */

  box_t *box; /* (BOX_LEN * TRACKING_SLOT_NUM) */
  /*****************END**************/

  uint32_t track_id[PREDICT_MODEL_NUM];

  uint8_t clq_src_num;
  uint8_t clq_dst_num;

  bool update_no_target_predict = false;
  bool support_hungarian = false;
  bool need_check_merge = false;
  bool is_x1 = false;
  uint32_t max_track_target_num = 256; /* align to 4, 52*/
  uint32_t max_det_target_num = 256;   /* align to 4, 100*/
  uint32_t max_osv_list = max_track_target_num * MAX_OSV_NUM;
  uint32_t track_time_gap_eliminate = 1650;

#ifdef TRACK_POST_FUNCTION
  uint8_t *face_id2main_id;
  uint8_t *head_id2main_id;
  main_id_pair_t *main_id2face_head_id;

  /* post fuction when track vanishing, 0:face, 1:head */
  track_post_func_t track_post_func[PREDICT_MODEL_NUM];
  ipc_context_t *ipc_ctx;
#endif
} tracking_module_data_t;

#endif  // IPC_TRACKING_TRACKING_DATA_TYPES_H
