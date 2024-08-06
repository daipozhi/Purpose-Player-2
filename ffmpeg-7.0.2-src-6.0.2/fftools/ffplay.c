/*
 * Copyright (c) 2003 Fabrice Bellard
 *
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

/**
 * @file
 * simple media player based on the FFmpeg libraries
 */




//daipozhi modified
#include <locale.h>
#include <iconv.h>

#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define DPZ_DEBUG1 0
#define DPZ_DEBUG2 0
#define GB18030    0




#include "config.h"
#include "config_components.h"
#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/eval.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/fifo.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/time.h"
#include "libavutil/bprint.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "libswresample/swresample.h"

#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"

#include <SDL.h>
#include <SDL_thread.h>

#include "cmdutils.h"
#include "ffplay_renderer.h"
#include "opt_common.h"

const char program_name[] = "ffplay";
const int program_birth_year = 2003;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 25
#define EXTERNAL_CLOCK_MIN_FRAMES 2
#define EXTERNAL_CLOCK_MAX_FRAMES 10

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512
/* Calculate actual buffer size keeping in mind not cause too frequent audio callbacks */
#define SDL_AUDIO_MAX_CALLBACKS_PER_SEC 30

/* Step size for volume control in dB */
#define SDL_VOLUME_STEP (0.75)

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.04
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* external clock speed adjustment constants for realtime sources based on buffer fullness */
#define EXTERNAL_CLOCK_SPEED_MIN  0.900
#define EXTERNAL_CLOCK_SPEED_MAX  1.010
#define EXTERNAL_CLOCK_SPEED_STEP 0.001

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */




//daipozhi modified
#define SAMPLE_ARRAY_SIZE (/**/ 16 * /**/ 8 * 65536)




#define CURSOR_HIDE_DELAY 1000000

#define USE_ONEPASS_SUBTITLE_RENDER 1

typedef struct MyAVPacketList {
    AVPacket *pkt;
    int serial;
} MyAVPacketList;

typedef struct PacketQueue {
    AVFifo *pkt_list;
    int nb_packets;
    int size;
    int64_t duration;
    int abort_request;
    int serial;
    SDL_mutex *mutex;
    SDL_cond *cond;
} PacketQueue;

#define VIDEO_PICTURE_QUEUE_SIZE 3
#define SUBPICTURE_QUEUE_SIZE 16
#define SAMPLE_QUEUE_SIZE 9
#define FRAME_QUEUE_SIZE FFMAX(SAMPLE_QUEUE_SIZE, FFMAX(VIDEO_PICTURE_QUEUE_SIZE, SUBPICTURE_QUEUE_SIZE))

typedef struct AudioParams {
    int freq;
    AVChannelLayout ch_layout;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
} AudioParams;

typedef struct Clock {
    double pts;           /* clock base */
    double pts_drift;     /* clock base minus time at which we updated the clock */
    double last_updated;
    double speed;
    int serial;           /* clock is based on a packet with this serial */
    int paused;
    int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
} Clock;

typedef struct FrameData {
    int64_t pkt_pos;
} FrameData;

/* Common struct for handling all types of decoded data and allocated render buffers. */
typedef struct Frame {
    AVFrame *frame;
    AVSubtitle sub;
    int serial;
    double pts;           /* presentation timestamp for the frame */
    double duration;      /* estimated duration of the frame */
    int64_t pos;          /* byte position of the frame in the input file */
    int width;
    int height;
    int format;
    AVRational sar;
    int uploaded;
    int flip_v;
} Frame;

typedef struct FrameQueue {
    Frame queue[FRAME_QUEUE_SIZE];
    int rindex;
    int windex;
    int size;
    int max_size;
    int keep_last;
    int rindex_shown;
    SDL_mutex *mutex;
    SDL_cond *cond;
    PacketQueue *pktq;
} FrameQueue;

enum {
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct Decoder {
    AVPacket *pkt;
    PacketQueue *queue;
    AVCodecContext *avctx;
    int pkt_serial;
    int finished;
    int packet_pending;
    SDL_cond *empty_queue_cond;
    int64_t start_pts;
    AVRational start_pts_tb;
    int64_t next_pts;
    AVRational next_pts_tb;
    SDL_Thread *decoder_tid;
} Decoder;

typedef struct VideoState {
    SDL_Thread *read_tid;
    const AVInputFormat *iformat;
    int abort_request;
    int force_refresh;
    int paused;
    int last_paused;
    int queue_attachments_req;
    int seek_req;
    int seek_flags;
    int64_t seek_pos;
    int64_t seek_rel;
    int read_pause_return;
    AVFormatContext *ic;
    int realtime;

    Clock audclk;
    Clock vidclk;
    Clock extclk;

    FrameQueue pictq;
    FrameQueue subpq;
    FrameQueue sampq;

    Decoder auddec;
    Decoder viddec;
    Decoder subdec;

    int audio_stream;

    int av_sync_type;

    double audio_clock;
    int audio_clock_serial;
    double audio_diff_cum; /* used for AV difference average computation */
    double audio_diff_avg_coef;
    double audio_diff_threshold;
    int audio_diff_avg_count;
    AVStream *audio_st;
    PacketQueue audioq;
    int audio_hw_buf_size;
    uint8_t *audio_buf;
    uint8_t *audio_buf1;
    unsigned int audio_buf_size; /* in bytes */
    unsigned int audio_buf1_size;
    int audio_buf_index; /* in bytes */
    int audio_write_buf_size;
    int audio_volume;
    int muted;
    struct AudioParams audio_src;
    struct AudioParams audio_filter_src;
    struct AudioParams audio_tgt;
    struct SwrContext *swr_ctx;
    int frame_drops_early;
    int frame_drops_late;

    enum ShowMode {
        SHOW_MODE_NONE = -1, SHOW_MODE_VIDEO = 0, SHOW_MODE_WAVES, SHOW_MODE_RDFT, SHOW_MODE_NB
    } show_mode;
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    int last_i_start;
    AVTXContext *rdft;
    av_tx_fn rdft_fn;
    int rdft_bits;
    float *real_data;
    AVComplexFloat *rdft_data;
    int xpos;
    double last_vis_time;
    SDL_Texture *vis_texture;
    SDL_Texture *sub_texture;
    SDL_Texture *vid_texture;

    int subtitle_stream;
    AVStream *subtitle_st;
    PacketQueue subtitleq;

    double frame_timer;
    double frame_last_returned_time;
    double frame_last_filter_delay;
    int video_stream;
    AVStream *video_st;
    PacketQueue videoq;
    double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity
    struct SwsContext *sub_convert_ctx;
    int eof;

    char *filename;
    int width, height, xleft, ytop;
    int step;

    int vfilter_idx;
    AVFilterContext *in_video_filter;   // the first filter in the video chain
    AVFilterContext *out_video_filter;  // the last filter in the video chain
    AVFilterContext *in_audio_filter;   // the first filter in the audio chain
    AVFilterContext *out_audio_filter;  // the last filter in the audio chain
    AVFilterGraph *agraph;              // audio filter graph

    int last_video_stream, last_audio_stream, last_subtitle_stream;

    SDL_cond *continue_read_thread;
} VideoState;




// ==== daipozhi modified ======================================================
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>

static unsigned char u_b1p[  128][14][14]; //utf8 char bmp
static unsigned char u_b2p[ 1920][14][14];
static unsigned char u_b3p[ 2048][14][14];
static unsigned char u_b4p[49152][14][14];
static unsigned char u_b5p[ 2048][14][14];
static unsigned char u_b6p[ 8192][14][14];
static unsigned char u_b1s[  128][2]; //utf8 char size
static unsigned char u_b2s[ 1920][2];
static unsigned char u_b3s[ 2048][2];
static unsigned char u_b4s[49152][2];
static unsigned char u_b5s[ 2048][2];
static unsigned char u_b6s[ 8192][2];

//static char u_tc[30][5];

static int    u_err;
static int    u_err2;

static int    u_char_size_x;
static int    u_char_size_y;    

static unsigned char u_char_bmp[14][14];

static unsigned char* u_str;

static int  u_n1, u_n2, u_n3;
static int  u_nb = 1;
static int  u_ptr = 0;
static int  u_cptr = 0;

static int  u_test_ptr=0;
static char u_test_buf[64][300];

static int u_test(void);
static int u_test_disp_win(void);
static int u_test_disp_ter(void);

static int u_get_char_bmp(unsigned char c1,unsigned char c2,unsigned char c3);

static int u_strcut(char *instr,int instr_size,char *outstr,int outstr_size,int fldlen,int st);
static int u_strlen(char *str,int str_size);

static  int       deb_fh;

static 	char      deb_scrn_str[2001];
static 	char      deb_scrn_str2[8001];

// daipozhi modified
static 	int    deb_seek_bar_cntr=0;

static 	int    deb_echo_str4screenstring(int xx,int yy,const char *str,int str_size,int st,int tx);

static 	int    deb_ch_h=14;
static 	int    deb_ch_w=7;
static 	int    deb_ch_m=30;
//static 	int    deb_ch_d=0;

static void init_opts(void);

#define FN_SIZE       300
#define MAX_FILE_NUM  100000

static  char   deb_filenamebuff[MAX_FILE_NUM][FN_SIZE];
static  char   deb_filenamebuff_ext[MAX_FILE_NUM][6];
static  char   deb_filenamebuff_size[MAX_FILE_NUM][7];
static  char   deb_filenamebuff_date[MAX_FILE_NUM][21];
static  int    deb_filenamebuff_len[MAX_FILE_NUM];
static  int    deb_filenamebuff_len2[MAX_FILE_NUM];
static  char   deb_filenamebuff_type[MAX_FILE_NUM];
static  char   deb_filenamebuff_icon[MAX_FILE_NUM];
static  int    deb_filenamebuff_space[MAX_FILE_NUM];
static  char   deb_filenamebuff_subline[MAX_FILE_NUM];
static  int    deb_filenamebuff_ptr;
static  int    deb_filenamebuff_n;
static  int    deb_filenamebuff_n2;
static  int    deb_filenameplay;
static  char   deb_currentpath[FN_SIZE];


static  char   deb_filenamebuff2[MAX_FILE_NUM][FN_SIZE];
static  char   deb_filenamebuff2_ext[MAX_FILE_NUM][6];
static  char   deb_filenamebuff2_size[MAX_FILE_NUM][7];
static  char   deb_filenamebuff2_date[MAX_FILE_NUM][21];
static  int    deb_filenamebuff2_len[MAX_FILE_NUM];
static  int    deb_filenamebuff2_len2[MAX_FILE_NUM];
static  char   deb_filenamebuff2_type[MAX_FILE_NUM];
static  char   deb_filenamebuff2_icon[MAX_FILE_NUM];
static  int    deb_filenamebuff2_space[MAX_FILE_NUM];
static  char   deb_filenamebuff2_subline[MAX_FILE_NUM];
static  int    deb_filenamebuff2_ptr;


// daipozhi modified 
static 	int    deb_get_dir(void);
static 	char   deb_lower(char c1);
static 	int    deb_lower_string(char *p_instr,int p_instr_size);
static 	char   deb_upper(char c1);
static 	int    deb_upper_string(char *p_instr,int p_instr_size);
static 	int    deb_string2int(char *string,int p1,int p2);
static 	int    deb_disp_dir(VideoState *is);
static 	int    deb_conv_dir(void);
static 	int    deb_disp_bar(VideoState *is);
static  int    deb_utf8_to_gb18030(char *inbuffer,char *outbuffer,int outbufferlen);
static  int    deb_gb18030_to_utf8(char *inbuffer,char *outbuffer,int outbufferlen);
static 	int    deb_filename_ext(char *path,int path_size,char *fext,int fext_size);
static 	int    deb_filename_dir(char *path,char *name);

static 	int deb_video_open_again(VideoState *is, int force_set_video_mode);
static 	int deb_ini_is(VideoState *is);

static 	int deb_st_play=0;
static 	int deb_st_video=0;

static 	int deb_eo_stream=1;

static 	int deb_thr_v=0;
static 	int deb_thr_s=0;
static 	int deb_thr_r=0;
static 	int deb_thr_a =0;
static 	int deb_thr_a2=0;

static 	int deb_load_font(void);

static 	int  deb_record_init(void);
static 	int  deb_record_close(void);
static 	FILE *deb_record_fp;
static 	int  deb_record(const char *p_str1);

/*
static 	char deb_tableline[3000];
*/
static          int  screen_w;
static          int  screen_h;
static    SDL_Rect    deb_m_rect;
static    int         deb_m_ref;
static    int         deb_m_ref_v;
static    int         deb_m_vol=0;



static SDL_Rect  deb_tx_rect;
static uint32_t *deb_tx_pixels;
static int       deb_tx_pitch;
static int       deb_tx_locked=0;
static int 	 deb_set_dot(int x1, int y1, unsigned char r, unsigned char g, unsigned char b);
static int       fill_rect_green(int x,int y,int w,int h,unsigned char c);
static int       clr_rect_black(void);
static int       clr_rect_white(void);



static  int  deb_cover=0;
static  int  deb_cover_close=0;
static  int  deb_frame_num=0;
static  int  deb_stream_err=0;
static  int  deb_stream_open=0;
static  int  deb_opts_stt=0;

static  char deb_dir_buffer[FN_SIZE];

// daipozhi modified 
static 	int  deb_get_dir_ini(void);
static 	int  deb_dir_opened(int ptr );
static 	int  deb_get_space(int ptr);
static 	int  deb_get_space2(int ptr);
//static 	char deb_getfirstchar(char *buffer,int buffer_size);
static 	int  deb_dir_add_after(int ptr);
static 	int  deb_dir_remove_after(int ptr);
static 	int  deb_get_path(int ptr);
static 	int  deb_get_path1(char *buffer1,char *buffer2);
static 	int  deb_get_path2(char *buffer1,char *buffer2);
static 	int  deb_cmp_dir(char *buffer1,char *buffer2);

static  int deb_no_support(char *p_str,int p_str_size);

static  int deb_str_has_null(const char *s1,int s1_size);

static  struct stat   deb_m_info;

static  struct tm*    deb_m_info_tm;
static  int           deb_m_info_len;
static  char          deb_m_info_type;

static int deb_size_format(int pn,char *buffer);
//static int deb_get_dir_len(int ptr);
//static int deb_get_dir_len2(int ptr);


// ---- binary tree ----------------------------------------------------------

// ------- file name compare ----------------------------------------------
#define FNC_NUM 150

#define BTREE1_SIZE   50000
#define BTREE1_LSIZE  50000

static    char  bt_node_mark[BTREE1_SIZE];
static    char  bt_node_val[BTREE1_SIZE][FN_SIZE];
static    char  bt_node_val2[BTREE1_SIZE];
static    char  bt_node_val3[BTREE1_SIZE][6];
static    char  bt_node_val4[BTREE1_SIZE][7];
static    char  bt_node_val5[BTREE1_SIZE][21];
static    char  bt_node_val6[BTREE1_SIZE];

//file name compare
static    char  bt_fnc_str[BTREE1_SIZE][FN_SIZE];
static    char  bt_fnc_msk[BTREE1_SIZE][FN_SIZE];
static    int   bt_fnc_istr[BTREE1_SIZE][FNC_NUM];
static    int   bt_fnc_ptr1[BTREE1_SIZE];
static    int   bt_fnc_ptr2[BTREE1_SIZE];

static    int   bt_stack[BTREE1_SIZE];
static    int   bt_stack_ptr;

static    int   bt_parent;
static    int   bt_parent_side;
static    int   bt_current;
static    int   bt_child_left;
static    int   bt_child_right;

static    int   bt_node_ptr[BTREE1_SIZE][3];
static    int   bt_root_ptr;

static    int   bt_find_ptr;
static    int   bt_find_ptr2;
static    int   bt_find_side;
    
static    int   bt_list_stack[BTREE1_LSIZE];
static    char  bt_list_stack_type[BTREE1_LSIZE];
static    int   bt_list_ptr;

static     char  bt_out_buff[BTREE1_SIZE][FN_SIZE];
static     char  bt_out_buff2[BTREE1_SIZE];
static     char  bt_out_buff3[BTREE1_SIZE][6];
static     char  bt_out_buff4[BTREE1_SIZE][7];
static     char  bt_out_buff5[BTREE1_SIZE][21];
static     char  bt_out_buff6[BTREE1_SIZE];
static     int   bt_out_ptr;

static    int   bt_err;

static    int   bt_init_tree(void);
static    int   bt_new_node(void);
static    int   bt_old_node(int ptr);
static    int   bt_clear_node(int ptr);
static    int   bt_search_node(char *pstr,char ptype);
static    int   bt_insert_node(char *pstr,char ptype);
static    int   bt_delete_node(char *pstr,char ptype);
static    int   bt_smallest(void);
static    int   string_comp(char *ps1,char *ps2);
static    int   bt_after_list(void);
static    int   bt_out_list(char *pstr,char ptype,int ptr);
//static    int   bt_dsp_list(void);
//static    int   bt_save_list(char *fn);

// ---- end of binary tree ---------------------------------------------



// daipozhi modified 
static double get_master_clock(VideoState *is);
static     VideoState *stream_open_is;




// ---- daipozhi modified for sound river ---------------------------------------

#if DPZ_DEBUG2
  #define FFT_BUFFER_SIZE  16384
static  short int deb_sr_fft_deb[4][FFT_BUFFER_SIZE*9];
static  int   deb_sr_fft_deb_chn;
static  int   deb_sr_fft_deb_ptr1;
static  int   deb_sr_fft_deb_ptr2;
static  int   deb_sr_fft_deb_ptr3;
#else
  #define FFT_BUFFER_SIZE  256
#endif  
//---------------------------------------------------------------------------

static int deb_sr_show;
static int deb_sr_show_start;
static int deb_sr_show_nodisp;
static int deb_sr_show_init;
static int deb_sr_rate;
static int deb_sr_ch;
static int deb_sr_fft_start;
static int deb_sr_fft_end;
static int deb_sr_river_over;

static int deb_sr_sample_size;

static int  		deb_sr_time_set;
static long long int	deb_sr_time1;
static long long int	deb_sr_time2;
static long long int 	deb_sr_time3;
static long long int 	deb_sr_time5;
static long long int 	deb_sr_total_bytes;

static int deb_sr_river[396][FFT_BUFFER_SIZE/2];
static int deb_sr_river2[150][FFT_BUFFER_SIZE/2];
static int deb_sr_river_ptr;
static int deb_sr_river_mark[396];
static int deb_sr_river_last;
static long long int deb_sr_river_adj;

static int deb_sr_sample_over;
static int deb_sr_sample_over2;

static int deb_sr_sdl_callback_cnt;

// ------- fft --------------------------------------------------------------------

//*============================================================================
//
//       fourier.h  -  Don Cross <dcross@intersrv.com>
//
//       http://www.intersrv.com/~dcross/fft.html
//
//       Contains definitions for doing Fourier transforms
//       and inverse Fourier transforms.
//
//============================================================================*/

// daipozhi modified after Aug 1998

#define TRUE  1
#define FALSE 0

#define BITS_PER_WORD   (sizeof(long)*8)

#define    DDC_PI   3.1415926535897932
//#define  DDC_PI  (3.1415926535897932384626433)
//#define  DDC_PI  (3.14159265358979323846264338327950288419)


  static  int  deb_sr_fft_float
         ( long     NumSamples,           /* must be a power of 2 */
           float  *RealIn,               /* array of input's real samples */
           float  *RealOut,              /* array of output's reals */
           float  *ImaginaryOut );       /* array of output's imaginaries */


  static  int  deb_sr_ifft_float
         ( long     NumSamples,           /* must be a power of 2 */
           float  *RealIn,               /* array of input's real samples */
           float  *RealOut,              /* array of output's reals */
           float  *ImaginaryOut );       /* array of output's imaginaries */



  static long deb_sr_IsPowerOfTwo(long x);
  static long deb_sr_NumberOfBitsNeeded(long PowerOfTwo);
  static long deb_sr_ReverseBits(long index,long NumBits);

//*
//**   The following function returns an "abstract frequency" of a
//**   given index into a buffer with a given number of frequency samples.
//**   Multiply return value by sampling rate to get frequency expressed in Hz.
//

    static float deb_sr_Index_to_frequency (long NumSamples,long Index );

    static int   deb_sr_CheckPointer (float *p);

// --------------------------------------------------------------------------

static    float  dlp_real_in1[FFT_BUFFER_SIZE];
static    float  dlp_real_ou1[FFT_BUFFER_SIZE];
static    float  dlp_imag_ou1[FFT_BUFFER_SIZE];
static    float  dlp_real_ou2[FFT_BUFFER_SIZE];
static    float  dlp_imag_ou2[FFT_BUFFER_SIZE];


static    int     deb_sr_fft_trans_all(VideoState *is,long pcm);
static    int     deb_sr_fft_cx(int chn,int pcm,int mark);
static    int     deb_sr_fft_set_db(long pcm);

//static    int     char2int(char *string,int p1,int p2);
static    int     deb_sr_cc2i(char c1,char c2);
static    void    deb_sr_i2cc(int k,char *cc);

static    float  get_dlp_real_in1(long addr);
static    float  get_dlp_real_ou1(long addr);
static    float  get_dlp_imag_ou1(long addr);
static    float  get_dlp_real_ou2(long addr);
static    float  get_dlp_imag_ou2(long addr);

static    int    put_dlp_real_in1(long addr,float val);
static    int    put_dlp_real_ou1(long addr,float val);
static    int    put_dlp_imag_ou1(long addr,float val);
static    int    put_dlp_real_ou2(long addr,float val);
static    int    put_dlp_imag_ou2(long addr,float val);

//--------------------------------------------------------------------------

// ---- river display -------------------------------------------------------

static float   deb_sr_db[60];

static int  deb_sr_river_f[151][118][60][2];

static int  deb_sr_river_f_init;
static int  deb_sr_river_f_init_fail;
static int  deb_sr_river_f_cons(void);
static int  deb_sr_river_f_cons_test(VideoState *cur_stream,int ptr);
static int  deb_sr_river_show(VideoState *cur_stream);
static int  deb_sr_river_show_pause(VideoState *cur_stream);

static int  deb_sr_river_f_test=0;

static char deb_sr_d_buff[7680][4320];
static int  deb_sr_d_return[2];
static int  deb_sr_d_cache[8000][2];
static int  deb_sr_d_cache_ptr;
static int  deb_sr_d_line[2][8000][2];
static int  deb_sr_d_line_ptr[2];
static int  deb_sr_d_line_ptr2[2];
static int  deb_sr_d_line_dot[2][2];

static int  deb_sr_d_init(void);
static int  deb_sr_draw_line(int x1,int y1,int x2,int y2);
static int  deb_sr_draw_line2(int x1,int y1,int x2,int y2);

static int  deb_sr_d_cache_clr(void);

static int deb_sr_draw_line3(int x1,int y1,int x2,int y2);
static int deb_sr_draw_line4_ini(void);
static int deb_sr_draw_line4(int x1,int y1,int x2,int y2,int ptr);
static int deb_sr_draw_line4_get(int ptr);
static int deb_sr_draw_line4_get2(int ptr);

// ------- end of sound river --------------------------------------------


// ------- audio resample ------------------------------------------------------

static int deb_resam_rate;
static int deb_resam_dir;

static int deb_resam_f[450];

static unsigned char deb_resam_buff1[500000];
static unsigned char deb_resam_buff2[500000];
static unsigned char deb_resam_buff3[100000];

static int deb_resam_buff1_ptr;
static int deb_resam_buff2_ptr;
static int deb_resam_buff3_ptr;
static int deb_resam_buff2_indx;
static int deb_resam_last_ptr;

// ------- end of resample ------------------------------------------------


// ------- file name compare ----------------------------------------------
//#define FNC_NUM 150

static    char  fnc_str[FN_SIZE];
static    char  fnc_msk[FN_SIZE];
static    int   fnc_istr[FNC_NUM];
static    int   fnc_ptr1,fnc_ptr2;

int fnc_conv(char *p_str);
int fnc_comp(int ptr);
int fnc_save(int ptr);
int fnc_str2int(char *p_string,int p_string_size);
// ------- end of file name compare


// ------- a minimal GUI lib in gcc -----------------

#define G_BUTTON_NUM      20
#define G_SCROLLBAR_NUM    5
#define G_DIRVIEW_NUM      5
#define G_LABEL_NUM       10

int  g_button_posi[ G_BUTTON_NUM][4];
int  g_button_color[G_BUTTON_NUM][3];
char g_button_black[G_BUTTON_NUM];
char g_button_text[ G_BUTTON_NUM][FN_SIZE];
char g_button_text_bl[ G_BUTTON_NUM];
int  g_button_text_color[ G_BUTTON_NUM][3];
int  g_button_cutcorner[G_BUTTON_NUM];
char g_button_enable[G_BUTTON_NUM];
char g_button_delete[G_BUTTON_NUM];
int  g_button_ptr;

char g_cut_map[7][10][10];

float g_button_paint_value_max;
float g_button_paint_value[300];
int g_mouse_down;

//int g_draw_circle4button(/*int x,int y,*/int r/*,int vid,int color1,int color2,int color3,int st*/);
int g_get_button_val(int r);
int g_button_dist_to_edge(int j,int k,int l,int m,int cut,int n,int o); // distance to button's edge

int g_cut_init(void);

int g_create_button(int x,int y,int w,int h,int black_button,int color1,int color2,int color3,char *text,int text_size,int cutcorner);
int g_move_button(int,int x,int y,int w,int h);
int g_set_button_text(int btn_ptr,char *str,int str_size);
int g_set_button_text_color(int btn_ptr,int black_letter,int color1,int color2,int color3);
int g_paint_button(int,int);
int g_enable_button(int btn_ptr,int enable);
int g_delete_button(int btn_ptr);

int g_detect_click(int ,int );
int g_detect_motion(int ,int );

int   g_scrollbar_posi[ G_SCROLLBAR_NUM][4];
int   g_scrollbar_color[G_SCROLLBAR_NUM][3];
char  g_scrollbar_black[G_SCROLLBAR_NUM];
char  g_scrollbar_hori[ G_SCROLLBAR_NUM];
char  g_scrollbar_enable[G_SCROLLBAR_NUM];
char  g_scrollbar_delete[G_SCROLLBAR_NUM];

float g_scrollbar_value_max[G_SCROLLBAR_NUM];
float g_scrollbar_value_now[G_SCROLLBAR_NUM];

int   g_scrollbar_cutcorner[G_SCROLLBAR_NUM];
int   g_scrollbar_ptr;

int g_create_scrollbar(int x,int y,int w,int h,int black_bar,int color1,int color2,int color3,int hori);
int g_move_scrollbar(int,int x,int y,int w,int h);
int g_paint_scrollbar(int);
int g_enable_scrollbar(int,int);
int g_delete_scrollbar(int);

int g_set_scrollbar_value_max(int ptr,float v);
float g_get_scrollbar_value_max(int ptr);
int g_set_scrollbar_value_now(int ptr,float v);
float g_get_scrollbar_value_now(int ptr);

int g_dirview_posi[   G_DIRVIEW_NUM][4];
int g_dirview_cutcorner[G_DIRVIEW_NUM];
int g_dirview_ptr;

int g_dirview_mouse_down;
int g_dirview_mouse_down_x;
int g_dirview_mouse_down_y;

int g_create_dirview(int x,int y,int w,int h);
int g_move_dirview(int,int x,int y,int w,int h);
int g_paint_dirview(void);

int g_init(void);

int g_detect_ptr1;
int g_detect_ptr2;
int g_detect_ptr3;
int g_detect_ptr4;

int g_detect_motion_ptr1;
int g_detect_motion_ptr2;
int g_detect_motion_ptr3;
int g_detect_motion_ptr4;

int g_focus_ptr1;
int g_focus_ptr2;
int g_focus_ptr3;
int g_focus_ptr4;

int g_dirview_top;
int g_dirview_bottom;

int  g_label_posi[G_LABEL_NUM][4];
int  g_label_color[G_LABEL_NUM][3];
char g_label_black[G_LABEL_NUM];
char g_label_text[G_LABEL_NUM][FN_SIZE];
char g_label_text_bl[ G_LABEL_NUM];
int  g_label_text_color[ G_LABEL_NUM][3];
char g_label_enable[G_LABEL_NUM];
char g_label_delete[G_LABEL_NUM];
int  g_label_ptr;

int g_create_label(int x,int y,int w,int h,int black_label,int color1,int color2,int color3,char *text,int text_size);
int g_move_label(int,int x,int y,int w,int h);
int g_enable_label(int,int);
int g_delete_label(int);
int g_set_label_text(int lb_ptr,char *str,int str_size);
int g_set_label_text_color(int lb_ptr,int black_letter,int color1,int color2,int color3);
int g_paint_label(int);

unsigned char g_icons[8][14][14][4];
unsigned char g_icons_mask[8][14][14];

int g_load_icon(void);
int g_paint_icon(int xx,int yy,int ic);
int g_icon_id(int dir,char *p_str,int p_str_size);


#define G_LINEEDIT_NUM 20

int  g_lineedit_posi[ G_LINEEDIT_NUM][4];
int  g_lineedit_color[G_LINEEDIT_NUM][3];
char g_lineedit_black[G_LINEEDIT_NUM];
char g_lineedit_text[ G_LINEEDIT_NUM][FN_SIZE];
char g_lineedit_text_bl[ G_LINEEDIT_NUM];
int  g_lineedit_text_color[ G_LINEEDIT_NUM][3];
char g_lineedit_type[ G_LINEEDIT_NUM];
char g_lineedit_enable[G_LINEEDIT_NUM];
char g_lineedit_delete[G_LINEEDIT_NUM];
int  g_lineedit_current_id=(-1);
int  g_lineedit_ptr;

int  g_caret_show;

int g_create_lineedit(const char *,int x,int y,int w,int h,int black_lineedit,int color1,int color2,int color3,char *text,int len_screen,int len_real,int dec);
int g_move_lineedit(int,int x,int y,int w,int h);
int g_set_lineedit_text(int btn_ptr,char *str,int str_size);
int g_set_lineedit_text_color(int btn_ptr,int black_letter,int color1,int color2,int color3);
int g_enable_lineedit(int,int);
int g_delete_lineedit(int);
int g_paint_lineedit(int);
int g_lineedit_set_posi(int le_ptr,int x);		

SDL_Cursor *g_cursor_arrow;
SDL_Cursor *g_cursor_ibeam;

#define  SMG_GET_NUM    200   /*   get_read subroutine var number */
#define  SMG_SIZE       300

#define  SMG_KEY_BACKSP 127
#define  SMG_KEY_RET    10
#define  SMG_KEY_TAB    9
#define  CTRL_B     2
#define  CTRL_K     11
#define  CTRL_I     9
#define  CTRL_L     12
#define  CTRL_Q     17
#define  CTRL_O     15
#define  CTRL_T     20
#define  CTRL_Y     25
#define  CTRL_C     3
#define  CTRL_V     22
#define  CTRL_E     5
#define  CTRL_H     8
#define  CTRL_F     6   /* find condition */
#define  CTRL_N     14  /* no find condition */
#define  CTRL_R     18  /* rollback    */
#define  CTRL_W     23  /* commit      */
#define  CTRL_Y     25  /* delete line in edit window*/
#define  CTRL_D     4   /* delete record */
#define  CTRL_A     1   /* insert record */
#define  CTRL_P     16  /* print data  */
#define  SMG_KEY_F1     321 /* edit memo   */
#define  SMG_KEY_F2     322 /* run ole     */
#define  SMG_KEY_F3     323 /* edit memo   */
#define  SMG_KEY_F4     324 /* run ole     */
#define  SMG_KEY_F5     325 /* edit memo   */
#define  SMG_KEY_F6     326 /* run ole     */
#define  SMG_KEY_F10    310
#define  SMG_KEY_ESC    27  /* close window   */
#define  SMG_KEY_LEFT   276
#define  SMG_KEY_RIGHT  277
#define  SMG_KEY_UP     274
#define  SMG_KEY_DOWN   275
#define  SMG_KEY_INS     312
#define  SMG_KEY_DEL    313
#define  SMG_KEY_PGUP      315 /* page up     */
#define  SMG_KEY_PGDOWN    316 /* page down   */
#define  SMG_KEY_FIND       317 /* start find  */
#define  SMG_KEY_SELECT     318 /* open window */

int   smg_edit_xchar;
int   smg_edit_ychar;

int   smg_chs_string_cut(char *p_str,int pn); // pn :field length

int   smg_key;

int  get_smg_num_conv(int datalen_real,int dec);
int  get_smg_txt_after(char *data,int p1);
int  get_smg_txt_before(char *data,int p1);
int  smg_calc_len(void);
int  smg_erase_last_char(int p1);

int  SetCaretPos(int x,int y);


int   smg_insert;
int   smg_ptr;
int   smg_ptr2;
int   smg_color1,smg_color2;

int   smg_chns_char;

char  smg_chns_str[3];
int   smg_cur1;
int   smg_cur2;

int   smg_p1;

//int   smg_color;
int   smg_confirm;           /* need enter or not need enter */
//int   smg_edit_mode[SMG_GET_NUM];/* 0--box,1--line    */
int   smg_p_y;
int   smg_p_x;


int  tst_smg_get_read(void);

int  smg_get_read_ini(void);
//int  get_read_login(void);
//int  get_read_logout(int ptr);

int  smg_get_read(int p_wptr,int scrn_l,int scrn_c,const char *atten,
	    char *string,int datalen,char datatype,
	    char command,int datalen2,int datadec,
	    char link,int color,int posi,int array_ptr);


int  smg_get_text(int p_wptr,int scrn_l,int scrn_c,char *atten,
           int datalen,int datalen2,char link,
	   int ptr,int color,int posi);
int  smg_get_number(int p_wptr,int scrn_l,int scrn_c,char *atten,
	   int datalen,int datalen2,char link,
	   int ptr,int color,int dec,int posi);
int  smg_get_password(int p_wptr,int scrn_l,int scrn_c,char *atten,
		    int datalen,int datalen_real,char link,
		    int ptr,int color,int posi);
	   
int  smg_get_num_conv(int datalen2,int dec);
int  smg_get_txt_after(char *data,int p1);
int  smg_get_txt_before(char *data,int p1);


int    smg_line[SMG_GET_NUM];
int    smg_colu[SMG_GET_NUM];
char   smg_data[SMG_SIZE];
char   smg_string[SMG_GET_NUM][SMG_SIZE];
char   smg_type[SMG_GET_NUM];
int    smg_dlen[SMG_GET_NUM];
int    smg_dlen_real[SMG_GET_NUM];
int    smg_ddec[SMG_GET_NUM];
char   smg_link[SMG_GET_NUM];
int    smg_color[SMG_GET_NUM];
int    smg_posi[SMG_GET_NUM];
char   smg_atte[SMG_GET_NUM][SMG_SIZE];
int    smg_read_id[SMG_GET_NUM];
char   smg_modi[SMG_GET_NUM];

int   get_smg_line(int gptr);
int   get_smg_colu(int gptr);
int   get_smg_data(int ptr1);
int   get_s_smg_data(char *s1,int s1_size);
int   get_smg_string(int gptr,char *s1,int s1_size);
int   get_c_smg_string(int gptr,int ptr1);
int   get_smg_type(int gptr);
int   get_smg_dlen(int gptr);
int   get_smg_dlen_real(int gptr);
int   get_smg_ddec(int gptr);
int   get_smg_link(int gptr);
int   get_smg_color(int gptr);
int   get_smg_posi(int gptr);
int   get_smg_atte(int gptr,char *s1,int s1_size);
int   get_smg_read_id(int gptr);
int   get_smg_modi(int gptr);

int   set_smg_line(int gptr,int val);
int   set_smg_colu(int gptr,int val);
int   set_smg_data(int ptr1,int val);
int   set_smg_string(int gptr,char *s1,int s1_size);
int   set_c_smg_string(int gptr,int ptr1,int val);
int   set_smg_type(int gptr,int val);
int   set_smg_dlen(int gptr,int val);
int   set_smg_dlen_real(int gptr,int val);
int   set_smg_ddec(int gptr,int val);
int   set_smg_link(int gptr,int val);
int   set_smg_color(int gptr,int val);
int   set_smg_posi(int gptr,int val);
int   set_smg_atte(int gptr,const char *s1,int s1_size);
int   set_smg_read_id(int gptr,int val);
int   set_smg_modi(int gptr,int val);



#define G_CHECKBOX_NUM 20

int   g_checkbox_posi[ G_CHECKBOX_NUM][4];
int   g_checkbox_color[G_CHECKBOX_NUM][3];
char  g_checkbox_black[G_CHECKBOX_NUM];
char  g_checkbox_enable[G_CHECKBOX_NUM];
char  g_checkbox_delete[G_CHECKBOX_NUM];

int   g_checkbox_value[G_CHECKBOX_NUM];

int   g_checkbox_ptr;

int g_create_checkbox(int x,int y,int w,int h,int black_bar,int color1,int color2,int color3);
int g_move_checkbox(int,int x,int y,int w,int h);
int g_enable_checkbox(int ptr,int enable);
int g_delete_checkbox(int ptr);
int g_paint_checkbox(int,int);
int g_draw_line(int x1,int y1,int x2,int y2,int vid,int color1,int color2,int colo3);

int g_set_checkbox_value(int ptr,int v);
int g_get_checkbox_value(int ptr);

#define G_RADIOBUTTON_NUM 20

int   g_radiobutton_posi[ G_RADIOBUTTON_NUM][4];
int   g_radiobutton_color[G_RADIOBUTTON_NUM][3];
char  g_radiobutton_black[G_RADIOBUTTON_NUM];
char  g_radiobutton_enable[G_RADIOBUTTON_NUM];
char  g_radiobutton_delete[G_RADIOBUTTON_NUM];

int   g_radiobutton_value[G_RADIOBUTTON_NUM];
int   g_radiobutton_value_group[G_RADIOBUTTON_NUM];

int   g_radiobutton_ptr;

int g_create_radiobutton(int x,int y,int w,int h,int black_bar,int color1,int color2,int color3,int grp);
int g_move_radiobutton(int,int x,int y,int w,int h);
int g_enable_radiobutton(int ptr,int enable);
int g_delete_radiobutton(int ptr);
int g_paint_radiobutton(int,int);
int g_draw_circle(int x,int y,int r,int vid,int color1,int color2,int colo3,int st);

int g_set_radiobutton_value(int ptr,int v);
int g_get_radiobutton_value(int ptr);

int g_set_radiobutton_value_group(int ptr,int v);
int g_get_radiobutton_value_group(int ptr);



int string_trim_nos(char *pstr);// no space

// ================ end of GUI ===================


// ================= lib automatic return ===============================
//     in today, file name usually very long, and screen is small,
//     lib automatic return separate one line file name to mutiple lines
//     in the best way.
// ======================================================================
// ar : automatic return

int ar_buff1_val[1000];
int ar_buff1_len[1000];
int ar_buff1_ptr;

int ar_buff2_len[1000];
int ar_buff2_ptr;

char ar_buff3[1000];

char ar_buff4[300][FN_SIZE];
int  ar_buff4_ptr;

char ar_char[1000][5];
int  ar_len[1000];
int  ar_len2[1000];
int  ar_size[1000];
int  ar_size2[1000];
int  ar_char_ptr;

int ar_conv(char *p_in_str,int p_in_str_size,int p_len);
int ar_separ2char(char *instr,int instr_size);

// ============== end of lib automatic return ============




/* options specified by the user */
static const AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;




// daipozhi modified
static int default_width  = 1224/*640*/;
static int default_height =  642/*480*/;




static int screen_width  = 0;
static int screen_height = 0;




// daipozhi modified
static int screen_left = 0 /*SDL_WINDOWPOS_CENTERED*/;
static int screen_top = 0 /*SDL_WINDOWPOS_CENTERED*/;




static int audio_disable;
static int video_disable;
static int subtitle_disable;
static const char* wanted_stream_spec[AVMEDIA_TYPE_NB] = {0};
static int seek_by_bytes = -1;
static float seek_interval = 10;
static int display_disable;
static int borderless;
static int alwaysontop;
static int startup_volume = 100;
static int show_status = -1;
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE;
static int64_t duration = AV_NOPTS_VALUE;
static int fast = 0;
static int genpts = 0;
static int lowres = 0;
static int decoder_reorder_pts = -1;




//daipozhi modified
static int autoexit/**/=1/**/;




static int exit_on_keydown;
static int exit_on_mousedown;
static int loop = 1;
static int framedrop = -1;
static int infinite_buffer = -1;
static enum ShowMode show_mode = SHOW_MODE_NONE;
static const char *audio_codec_name;
static const char *subtitle_codec_name;
static const char *video_codec_name;
double rdftspeed = 0.02;
static int64_t cursor_last_shown;
static int cursor_hidden = 0;
static const char **vfilters_list = NULL;
static int nb_vfilters = 0;
static char *afilters = NULL;
static int autorotate = 1;
static int find_stream_info = 1;
static int filter_nbthreads = 0;
static int enable_vulkan = 0;
static char *vulkan_params = NULL;
static const char *hwaccel = NULL;

/* current context */
static int is_full_screen;
static int64_t audio_callback_time;

#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_RendererInfo renderer_info = {0};
static SDL_AudioDeviceID audio_dev;

static VkRenderer *vk_renderer;

static const struct TextureFormatEntry {
    enum AVPixelFormat format;
    int texture_fmt;
} sdl_texture_format_map[] = {
    { AV_PIX_FMT_RGB8,           SDL_PIXELFORMAT_RGB332 },
    { AV_PIX_FMT_RGB444,         SDL_PIXELFORMAT_RGB444 },
    { AV_PIX_FMT_RGB555,         SDL_PIXELFORMAT_RGB555 },
    { AV_PIX_FMT_BGR555,         SDL_PIXELFORMAT_BGR555 },
    { AV_PIX_FMT_RGB565,         SDL_PIXELFORMAT_RGB565 },
    { AV_PIX_FMT_BGR565,         SDL_PIXELFORMAT_BGR565 },
    { AV_PIX_FMT_RGB24,          SDL_PIXELFORMAT_RGB24 },
    { AV_PIX_FMT_BGR24,          SDL_PIXELFORMAT_BGR24 },
    { AV_PIX_FMT_0RGB32,         SDL_PIXELFORMAT_RGB888 },
    { AV_PIX_FMT_0BGR32,         SDL_PIXELFORMAT_BGR888 },
    { AV_PIX_FMT_NE(RGB0, 0BGR), SDL_PIXELFORMAT_RGBX8888 },
    { AV_PIX_FMT_NE(BGR0, 0RGB), SDL_PIXELFORMAT_BGRX8888 },
    { AV_PIX_FMT_RGB32,          SDL_PIXELFORMAT_ARGB8888 },
    { AV_PIX_FMT_RGB32_1,        SDL_PIXELFORMAT_RGBA8888 },
    { AV_PIX_FMT_BGR32,          SDL_PIXELFORMAT_ABGR8888 },
    { AV_PIX_FMT_BGR32_1,        SDL_PIXELFORMAT_BGRA8888 },
    { AV_PIX_FMT_YUV420P,        SDL_PIXELFORMAT_IYUV },
    { AV_PIX_FMT_YUYV422,        SDL_PIXELFORMAT_YUY2 },
    { AV_PIX_FMT_UYVY422,        SDL_PIXELFORMAT_UYVY },
    { AV_PIX_FMT_NONE,           SDL_PIXELFORMAT_UNKNOWN },
};

static int opt_add_vfilter(void *optctx, const char *opt, const char *arg)
{
    int ret = GROW_ARRAY(vfilters_list, nb_vfilters);
    if (ret < 0)
        return ret;

    vfilters_list[nb_vfilters - 1] = av_strdup(arg);
    if (!vfilters_list[nb_vfilters - 1])
        return AVERROR(ENOMEM);

    return 0;
}

static inline
int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1,
                   enum AVSampleFormat fmt2, int64_t channel_count2)
{
    /* If channel count == 1, planar and non-planar formats are the same */
    if (channel_count1 == 1 && channel_count2 == 1)
        return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
    else
        return channel_count1 != channel_count2 || fmt1 != fmt2;
}

static int packet_queue_put_private(PacketQueue *q, AVPacket *pkt)
{
    MyAVPacketList pkt1;
    int ret;

    if (q->abort_request)
       return -1;


    pkt1.pkt = pkt;
    pkt1.serial = q->serial;

    ret = av_fifo_write(q->pkt_list, &pkt1, 1);
    if (ret < 0)
        return ret;
    q->nb_packets++;
    q->size += pkt1.pkt->size + sizeof(pkt1);
    q->duration += pkt1.pkt->duration;
    /* XXX: should duplicate packet data in DV case */
    SDL_CondSignal(q->cond);
    return 0;
}

static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
    AVPacket *pkt1;
    int ret;

    pkt1 = av_packet_alloc();
    if (!pkt1) {
        av_packet_unref(pkt);
        return -1;
    }
    av_packet_move_ref(pkt1, pkt);

    SDL_LockMutex(q->mutex);
    ret = packet_queue_put_private(q, pkt1);
    SDL_UnlockMutex(q->mutex);

    if (ret < 0)
        av_packet_free(&pkt1);

    return ret;
}

static int packet_queue_put_nullpacket(PacketQueue *q, AVPacket *pkt, int stream_index)
{
    pkt->stream_index = stream_index;
    return packet_queue_put(q, pkt);
}

/* packet queue handling */
static int packet_queue_init(PacketQueue *q)
{
    memset(q, 0, sizeof(PacketQueue));
    q->pkt_list = av_fifo_alloc2(1, sizeof(MyAVPacketList), AV_FIFO_FLAG_AUTO_GROW);
    if (!q->pkt_list)
        return AVERROR(ENOMEM);
    q->mutex = SDL_CreateMutex();
    if (!q->mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->cond = SDL_CreateCond();
    if (!q->cond) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->abort_request = 1;
    return 0;
}

static void packet_queue_flush(PacketQueue *q)
{
    MyAVPacketList pkt1;

    SDL_LockMutex(q->mutex);
    while (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0)
        av_packet_free(&pkt1.pkt);
    q->nb_packets = 0;
    q->size = 0;
    q->duration = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_destroy(PacketQueue *q)
{
    packet_queue_flush(q);
    av_fifo_freep2(&q->pkt_list);
    SDL_DestroyMutex(q->mutex);
    SDL_DestroyCond(q->cond);
}

static void packet_queue_abort(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);

    q->abort_request = 1;

    SDL_CondSignal(q->cond);

    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_start(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);
    q->abort_request = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block, int *serial)
{
    MyAVPacketList pkt1;
    int ret;

    SDL_LockMutex(q->mutex);

    for (;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }

        if (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0) {
            q->nb_packets--;
            q->size -= pkt1.pkt->size + sizeof(pkt1);
            q->duration -= pkt1.pkt->duration;
            av_packet_move_ref(pkt, pkt1.pkt);
            if (serial)
                *serial = pkt1.serial;
            av_packet_free(&pkt1.pkt);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            SDL_CondWait(q->cond, q->mutex);
        }
    }
    SDL_UnlockMutex(q->mutex);
    return ret;
}

static int decoder_init(Decoder *d, AVCodecContext *avctx, PacketQueue *queue, SDL_cond *empty_queue_cond) {
    memset(d, 0, sizeof(Decoder));
    d->pkt = av_packet_alloc();
    if (!d->pkt)
        return AVERROR(ENOMEM);
    d->avctx = avctx;
    d->queue = queue;
    d->empty_queue_cond = empty_queue_cond;
    d->start_pts = AV_NOPTS_VALUE;
    d->pkt_serial = -1;
    return 0;
}

static int decoder_decode_frame(Decoder *d, AVFrame *frame, AVSubtitle *sub) {
    int ret = AVERROR(EAGAIN);

    for (;;) {
        if (d->queue->serial == d->pkt_serial) {
            do {
                if (d->queue->abort_request)
                    return -1;

                switch (d->avctx->codec_type) {
                    case AVMEDIA_TYPE_VIDEO:
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) {
                            if (decoder_reorder_pts == -1) {
                                frame->pts = frame->best_effort_timestamp;
                            } else if (!decoder_reorder_pts) {
                                frame->pts = frame->pkt_dts;
                            }
                        }
                        break;
                    case AVMEDIA_TYPE_AUDIO:
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) {
                            AVRational tb = (AVRational){1, frame->sample_rate};
                            if (frame->pts != AV_NOPTS_VALUE)
                                frame->pts = av_rescale_q(frame->pts, d->avctx->pkt_timebase, tb);
                            else if (d->next_pts != AV_NOPTS_VALUE)
                                frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
                            if (frame->pts != AV_NOPTS_VALUE) {
                                d->next_pts = frame->pts + frame->nb_samples;
                                d->next_pts_tb = tb;
                            }
                        }
                        break;
                }
                if (ret == AVERROR_EOF) {
                    d->finished = d->pkt_serial;
                    avcodec_flush_buffers(d->avctx);
                    return 0;
                }
                if (ret >= 0)
                    return 1;
            } while (ret != AVERROR(EAGAIN));
        }

        do {
            if (d->queue->nb_packets == 0)
                SDL_CondSignal(d->empty_queue_cond);
            if (d->packet_pending) {
                d->packet_pending = 0;
            } else {
                int old_serial = d->pkt_serial;
                if (packet_queue_get(d->queue, d->pkt, 1, &d->pkt_serial) < 0)
                    return -1;
                if (old_serial != d->pkt_serial) {
                    avcodec_flush_buffers(d->avctx);
                    d->finished = 0;
                    d->next_pts = d->start_pts;
                    d->next_pts_tb = d->start_pts_tb;
                }
            }
            if (d->queue->serial == d->pkt_serial)
                break;
            av_packet_unref(d->pkt);
        } while (1);

        if (d->avctx->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            int got_frame = 0;
            ret = avcodec_decode_subtitle2(d->avctx, sub, &got_frame, d->pkt);
            if (ret < 0) {
                ret = AVERROR(EAGAIN);
            } else {
                if (got_frame && !d->pkt->data) {
                    d->packet_pending = 1;
                }
                ret = got_frame ? 0 : (d->pkt->data ? AVERROR(EAGAIN) : AVERROR_EOF);
            }
            av_packet_unref(d->pkt);
        } else {
            if (d->pkt->buf && !d->pkt->opaque_ref) {
                FrameData *fd;

                d->pkt->opaque_ref = av_buffer_allocz(sizeof(*fd));
                if (!d->pkt->opaque_ref)
                    return AVERROR(ENOMEM);
                fd = (FrameData*)d->pkt->opaque_ref->data;
                fd->pkt_pos = d->pkt->pos;
            }

            if (avcodec_send_packet(d->avctx, d->pkt) == AVERROR(EAGAIN)) {
                av_log(d->avctx, AV_LOG_ERROR, "Receive_frame and send_packet both returned EAGAIN, which is an API violation.\n");
                d->packet_pending = 1;
            } else {
                av_packet_unref(d->pkt);
            }
        }
    }
}

static void decoder_destroy(Decoder *d) {
    av_packet_free(&d->pkt);
    avcodec_free_context(&d->avctx);
}

static void frame_queue_unref_item(Frame *vp)
{
    av_frame_unref(vp->frame);
    avsubtitle_free(&vp->sub);
}

static int frame_queue_init(FrameQueue *f, PacketQueue *pktq, int max_size, int keep_last)
{
    int i;
    memset(f, 0, sizeof(FrameQueue));
    if (!(f->mutex = SDL_CreateMutex())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    if (!(f->cond = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    f->pktq = pktq;
    f->max_size = FFMIN(max_size, FRAME_QUEUE_SIZE);
    f->keep_last = !!keep_last;
    for (i = 0; i < f->max_size; i++)
        if (!(f->queue[i].frame = av_frame_alloc()))
            return AVERROR(ENOMEM);
    return 0;
}

static void frame_queue_destroy(FrameQueue *f)
{
    int i;
    for (i = 0; i < f->max_size; i++) {
        Frame *vp = &f->queue[i];
        frame_queue_unref_item(vp);
        av_frame_free(&vp->frame);
    }
    SDL_DestroyMutex(f->mutex);
    SDL_DestroyCond(f->cond);
}

static void frame_queue_signal(FrameQueue *f)
{
    SDL_LockMutex(f->mutex);
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

static Frame *frame_queue_peek(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

static Frame *frame_queue_peek_next(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

static Frame *frame_queue_peek_last(FrameQueue *f)
{
    return &f->queue[f->rindex];
}

static Frame *frame_queue_peek_writable(FrameQueue *f)
{
    /* wait until we have space to put a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size >= f->max_size &&
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request)
        return NULL;

    return &f->queue[f->windex];
}

static Frame *frame_queue_peek_readable(FrameQueue *f)
{
    /* wait until we have a readable a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size - f->rindex_shown <= 0 &&
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request)
        return NULL;

    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

static void frame_queue_push(FrameQueue *f)
{
    if (++f->windex == f->max_size)
        f->windex = 0;
    SDL_LockMutex(f->mutex);
    f->size++;
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

static void frame_queue_next(FrameQueue *f)
{
    if (f->keep_last && !f->rindex_shown) {
        f->rindex_shown = 1;
        return;
    }
    frame_queue_unref_item(&f->queue[f->rindex]);
    if (++f->rindex == f->max_size)
        f->rindex = 0;
    SDL_LockMutex(f->mutex);
    f->size--;
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

/* return the number of undisplayed frames in the queue */
static int frame_queue_nb_remaining(FrameQueue *f)
{
    return f->size - f->rindex_shown;
}

/* return last shown position */
static int64_t frame_queue_last_pos(FrameQueue *f)
{
    Frame *fp = &f->queue[f->rindex];
    if (f->rindex_shown && fp->serial == f->pktq->serial)
        return fp->pos;
    else
        return -1;
}

static void decoder_abort(Decoder *d, FrameQueue *fq)
{
    packet_queue_abort(d->queue);
    frame_queue_signal(fq);
    SDL_WaitThread(d->decoder_tid, NULL);
    d->decoder_tid = NULL;
    packet_queue_flush(d->queue);
}

static inline void fill_rectangle(int x, int y, int w, int h)
{
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    if (w && h)
        SDL_RenderFillRect(renderer, &rect);
}




//daipozhi modified
/* draw only the border of a rectangle */
static void fill_border(int xleft, int ytop, int width, int height, int x, int y, int w, int h)
{
    int w1, w2, h1, h2;

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);

    /* fill the background */
    w1 = x;
    if (w1 < 0)
        w1 = 0;
    w2 = width - (x + w);
    if (w2 < 0)
        w2 = 0;
    h1 = y;
    if (h1 < 0)
        h1 = 0;

    h2 = height - (y + h) -/*deb_ch_h*/18*2 ;

    if (h2 < 0)
        h2 = 0;

    fill_rectangle(
                   xleft, ytop,
                   w1, height -/*deb_ch_h*/18*2 
                   );
    fill_rectangle(
                   xleft + width - w2, ytop,
                   w2, height -/*deb_ch_h*/18*2 
                   );
    fill_rectangle(
                   xleft + w1, ytop,
                   width - w1 - w2, h1
                   );
    fill_rectangle(
                   xleft + w1, ytop + height - h2 -/*deb_ch_h*/18*2 ,
                   width - w1 - w2, h2
                   );
}




//daipozhi modified
static void fill_bottom(int width, int height)
{
    SDL_SetRenderDrawColor(renderer, 240, 240, 240, 240);

    fill_rectangle(
                   0, 
                   height-18*2-9,
                   width, 
                   18*2+9
                   );

    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 128);

    fill_rectangle(
                   0, 
                   height-18*2-9,
                   width, 
                   1
                   );
}




static int realloc_texture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture)
{
    Uint32 format;
    int access, w, h;
    if (!*texture || SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 || new_width != w || new_height != h || new_format != format) {
        void *pixels;
        int pitch;
        if (*texture)
            SDL_DestroyTexture(*texture);
        if (!(*texture = SDL_CreateTexture(renderer, new_format, SDL_TEXTUREACCESS_STREAMING, new_width, new_height)))
            return -1;
        if (SDL_SetTextureBlendMode(*texture, blendmode) < 0)
            return -1;
        if (init_texture) {
            if (SDL_LockTexture(*texture, NULL, &pixels, &pitch) < 0)
                return -1;
                
                
                
                
            //daipozhi modified
            memset(pixels, 255 /*0*/, pitch * new_height);
            
            
            
            
            SDL_UnlockTexture(*texture);
        }
        av_log(NULL, AV_LOG_VERBOSE, "Created %dx%d texture with %s.\n", new_width, new_height, SDL_GetPixelFormatName(new_format));
    }
    return 0;
}

static void calculate_display_rect(SDL_Rect *rect,
                                   int scr_xleft, int scr_ytop, int scr_width, int scr_height,
                                   int pic_width, int pic_height, AVRational pic_sar)
{
    AVRational aspect_ratio = pic_sar;
    int64_t width, height, x, y;

    if (av_cmp_q(aspect_ratio, av_make_q(0, 1)) <= 0)
        aspect_ratio = av_make_q(1, 1);

    aspect_ratio = av_mul_q(aspect_ratio, av_make_q(pic_width, pic_height));

    /* XXX: we suppose the screen has a 1.0 pixel ratio */




    //daipozhi modified
    height = scr_height/**/ -18*2 -9 /**/;




    width = av_rescale(height, aspect_ratio.num, aspect_ratio.den) & ~1;
    if (width > scr_width) {
        width = scr_width;
        height = av_rescale(width, aspect_ratio.den, aspect_ratio.num) & ~1;
    }
    x = (scr_width - width) / 2;




    // daipozhi modified
    y = (scr_height - height/**/ -18*2 -9 /**/) / 2;




    rect->x = scr_xleft + x;
    rect->y = scr_ytop  + y;
    rect->w = FFMAX((int)width,  1);
    rect->h = FFMAX((int)height, 1);
}

static void get_sdl_pix_fmt_and_blendmode(int format, Uint32 *sdl_pix_fmt, SDL_BlendMode *sdl_blendmode)
{
    int i;
    *sdl_blendmode = SDL_BLENDMODE_NONE;
    *sdl_pix_fmt = SDL_PIXELFORMAT_UNKNOWN;
    if (format == AV_PIX_FMT_RGB32   ||
        format == AV_PIX_FMT_RGB32_1 ||
        format == AV_PIX_FMT_BGR32   ||
        format == AV_PIX_FMT_BGR32_1)
        *sdl_blendmode = SDL_BLENDMODE_BLEND;
    for (i = 0; i < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; i++) {
        if (format == sdl_texture_format_map[i].format) {
            *sdl_pix_fmt = sdl_texture_format_map[i].texture_fmt;
            return;
        }
    }
}

static int upload_texture(SDL_Texture **tex, AVFrame *frame)
{
    int ret = 0;
    Uint32 sdl_pix_fmt;
    SDL_BlendMode sdl_blendmode;
    get_sdl_pix_fmt_and_blendmode(frame->format, &sdl_pix_fmt, &sdl_blendmode);
    if (realloc_texture(tex, sdl_pix_fmt == SDL_PIXELFORMAT_UNKNOWN ? SDL_PIXELFORMAT_ARGB8888 : sdl_pix_fmt, frame->width, frame->height, sdl_blendmode, 0) < 0)
        return -1;
    switch (sdl_pix_fmt) {
        case SDL_PIXELFORMAT_IYUV:
            if (frame->linesize[0] > 0 && frame->linesize[1] > 0 && frame->linesize[2] > 0) {
                ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0], frame->linesize[0],
                                                       frame->data[1], frame->linesize[1],
                                                       frame->data[2], frame->linesize[2]);
            } else if (frame->linesize[0] < 0 && frame->linesize[1] < 0 && frame->linesize[2] < 0) {
                ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height                    - 1), -frame->linesize[0],
                                                       frame->data[1] + frame->linesize[1] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[1],
                                                       frame->data[2] + frame->linesize[2] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[2]);
            } else {
                av_log(NULL, AV_LOG_ERROR, "Mixed negative and positive linesizes are not supported.\n");
                return -1;
            }
            break;
        default:
            if (frame->linesize[0] < 0) {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0]);
            } else {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0], frame->linesize[0]);
            }
            break;
    }
    return ret;
}

static enum AVColorSpace sdl_supported_color_spaces[] = {
    AVCOL_SPC_BT709,
    AVCOL_SPC_BT470BG,
    AVCOL_SPC_SMPTE170M,
    AVCOL_SPC_UNSPECIFIED,
};

static void set_sdl_yuv_conversion_mode(AVFrame *frame)
{
#if SDL_VERSION_ATLEAST(2,0,8)
    SDL_YUV_CONVERSION_MODE mode = SDL_YUV_CONVERSION_AUTOMATIC;
    if (frame && (frame->format == AV_PIX_FMT_YUV420P || frame->format == AV_PIX_FMT_YUYV422 || frame->format == AV_PIX_FMT_UYVY422)) {
        if (frame->color_range == AVCOL_RANGE_JPEG)
            mode = SDL_YUV_CONVERSION_JPEG;
        else if (frame->colorspace == AVCOL_SPC_BT709)
            mode = SDL_YUV_CONVERSION_BT709;
        else if (frame->colorspace == AVCOL_SPC_BT470BG || frame->colorspace == AVCOL_SPC_SMPTE170M)
            mode = SDL_YUV_CONVERSION_BT601;
    }
    SDL_SetYUVConversionMode(mode); /* FIXME: no support for linear transfer */
#endif
}

static void video_image_display(VideoState *is)
{
    Frame *vp;
    Frame *sp = NULL;
    SDL_Rect rect;
    
    
    
    
    //daipozhi modified
    if (deb_cover_close==1) return ;

    //daipozhi modified
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);




    vp = frame_queue_peek_last(&is->pictq);
    if (vk_renderer) {
        vk_renderer_display(vk_renderer, vp->frame);
        return;
    }

    if (is->subtitle_st) {
        if (frame_queue_nb_remaining(&is->subpq) > 0) {
            sp = frame_queue_peek(&is->subpq);

            if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
                if (!sp->uploaded) {
                    uint8_t* pixels[4];
                    int pitch[4];
                    int i;
                    if (!sp->width || !sp->height) {
                        sp->width = vp->width;
                        sp->height = vp->height;
                    }
                    if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width, sp->height, SDL_BLENDMODE_BLEND, 1) < 0)
                        return;

                    for (i = 0; i < sp->sub.num_rects; i++) {
                        AVSubtitleRect *sub_rect = sp->sub.rects[i];

                        sub_rect->x = av_clip(sub_rect->x, 0, sp->width );
                        sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
                        sub_rect->w = av_clip(sub_rect->w, 0, sp->width  - sub_rect->x);
                        sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);

                        is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA,
                            0, NULL, NULL, NULL);
                        if (!is->sub_convert_ctx) {
                            av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
                            return;
                        }
                        if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)pixels, pitch)) {
                            sws_scale(is->sub_convert_ctx, (const uint8_t * const *)sub_rect->data, sub_rect->linesize,
                                      0, sub_rect->h, pixels, pitch);
                            SDL_UnlockTexture(is->sub_texture);
                        }
                    }
                    sp->uploaded = 1;
                }
            } else
                sp = NULL;
        }
    }

    calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);




    //daipozhi modified
    deb_m_rect=rect;
    
    
    
    
    set_sdl_yuv_conversion_mode(vp->frame);

    if (!vp->uploaded) {
        if (upload_texture(&is->vid_texture, vp->frame) < 0) {
            set_sdl_yuv_conversion_mode(NULL);
            return;
        }
        vp->uploaded = 1;
        vp->flip_v = vp->frame->linesize[0] < 0;
    }

    SDL_RenderCopyEx(renderer, is->vid_texture, NULL, &rect, 0, NULL, vp->flip_v ? SDL_FLIP_VERTICAL : 0);
    set_sdl_yuv_conversion_mode(NULL);
    if (sp) {
#if USE_ONEPASS_SUBTITLE_RENDER
        SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect);
#else
        int i;
        double xratio = (double)rect.w / (double)sp->width;
        double yratio = (double)rect.h / (double)sp->height;
        for (i = 0; i < sp->sub.num_rects; i++) {
            SDL_Rect *sub_rect = (SDL_Rect*)sp->sub.rects[i];
            SDL_Rect target = {.x = rect.x + sub_rect->x * xratio,
                               .y = rect.y + sub_rect->y * yratio,
                               .w = sub_rect->w * xratio,
                               .h = sub_rect->h * yratio};
            SDL_RenderCopy(renderer, is->sub_texture, sub_rect, &target);
        }
#endif
    }




    //daipozhi modified 
    
    fill_border(is->xleft, is->ytop, is->width, is->height, 
                  deb_m_rect.x, deb_m_rect.y, deb_m_rect.w, deb_m_rect.h );
    
    fill_bottom(is->width, is->height);

    //daipozhi modified    
    deb_cover=1;
    deb_m_ref=1;
    deb_m_ref_v=1;
    deb_st_video=1;




}

static inline int compute_mod(int a, int b)
{
    return a < 0 ? a%b + b : a%b;
}

static void video_audio_display(VideoState *s)
{




//daipozhi modified
/*
    int i, i_start, x, y1, y, ys, delay, n, nb_display_channels;
    int ch, channels, h, h2;
    int64_t time_diff;
    int rdft_bits, nb_freq;

    for (rdft_bits = 1; (1 << rdft_bits) < 2 * s->height; rdft_bits++)
        ;
    nb_freq = 1 << (rdft_bits - 1);

    // * compute display index : center on currently output samples 
    channels = s->audio_tgt.ch_layout.nb_channels;
    nb_display_channels = channels;
    if (!s->paused) {
        int data_used= s->show_mode == SHOW_MODE_WAVES ? s->width : (2*nb_freq);
        n = 2 * channels;
        delay = s->audio_write_buf_size;
        delay /= n;

        // * to be more precise, we take into account the time spent since
        //   the last buffer computation * /
        if (audio_callback_time) {
            time_diff = av_gettime_relative() - audio_callback_time;
            delay -= (time_diff * s->audio_tgt.freq) / 1000000;
        }

        delay += 2 * data_used;
        if (delay < data_used)
            delay = data_used;

        i_start= x = compute_mod(s->sample_array_index - delay * channels, SAMPLE_ARRAY_SIZE);
        if (s->show_mode == SHOW_MODE_WAVES) {
            h = INT_MIN;
            for (i = 0; i < 1000; i += channels) {
                int idx = (SAMPLE_ARRAY_SIZE + x - i) % SAMPLE_ARRAY_SIZE;
                int a = s->sample_array[idx];
                int b = s->sample_array[(idx + 4 * channels) % SAMPLE_ARRAY_SIZE];
                int c = s->sample_array[(idx + 5 * channels) % SAMPLE_ARRAY_SIZE];
                int d = s->sample_array[(idx + 9 * channels) % SAMPLE_ARRAY_SIZE];
                int score = a - d;
                if (h < score && (b ^ c) < 0) {
                    h = score;
                    i_start = idx;
                }
            }
        }

        s->last_i_start = i_start;
    } else {
        i_start = s->last_i_start;
    }

    if (s->show_mode == SHOW_MODE_WAVES) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // * total height for one channel * /
        h = s->height / nb_display_channels;
        // * graph height / 2 * /
        h2 = (h * 9) / 20;
        for (ch = 0; ch < nb_display_channels; ch++) {
            i = i_start + ch;
            y1 = s->ytop + ch * h + (h / 2); // * position of center line * /
            for (x = 0; x < s->width; x++) {
                y = (s->sample_array[i] * h2) >> 15;
                if (y < 0) {
                    y = -y;
                    ys = y1 - y;
                } else {
                    ys = y1;
                }
                fill_rectangle(s->xleft + x, ys, 1, y);
                i += channels;
                if (i >= SAMPLE_ARRAY_SIZE)
                    i -= SAMPLE_ARRAY_SIZE;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

        for (ch = 1; ch < nb_display_channels; ch++) {
            y = s->ytop + ch * h;
            fill_rectangle(s->xleft, y, s->width, 1);
        }
    } else {
        int err = 0;
        if (realloc_texture(&s->vis_texture, SDL_PIXELFORMAT_ARGB8888, s->width, s->height, SDL_BLENDMODE_NONE, 1) < 0)
            return;

        if (s->xpos >= s->width)
            s->xpos = 0;
        nb_display_channels= FFMIN(nb_display_channels, 2);
        if (rdft_bits != s->rdft_bits) {
            const float rdft_scale = 1.0;
            av_tx_uninit(&s->rdft);
            av_freep(&s->real_data);
            av_freep(&s->rdft_data);
            s->rdft_bits = rdft_bits;
            s->real_data = av_malloc_array(nb_freq, 4 *sizeof(*s->real_data));
            s->rdft_data = av_malloc_array(nb_freq + 1, 2 *sizeof(*s->rdft_data));
            err = av_tx_init(&s->rdft, &s->rdft_fn, AV_TX_FLOAT_RDFT,
                             0, 1 << rdft_bits, &rdft_scale, 0);
        }
        if (err < 0 || !s->rdft_data) {
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffers for RDFT, switching to waves display\n");
            s->show_mode = SHOW_MODE_WAVES;
        } else {
            float *data_in[2];
            AVComplexFloat *data[2];
            SDL_Rect rect = {.x = s->xpos, .y = 0, .w = 1, .h = s->height};
            uint32_t *pixels;
            int pitch;
            for (ch = 0; ch < nb_display_channels; ch++) {
                data_in[ch] = s->real_data + 2 * nb_freq * ch;
                data[ch] = s->rdft_data + nb_freq * ch;
                i = i_start + ch;
                for (x = 0; x < 2 * nb_freq; x++) {
                    double w = (x-nb_freq) * (1.0 / nb_freq);
                    data_in[ch][x] = s->sample_array[i] * (1.0 - w * w);
                    i += channels;
                    if (i >= SAMPLE_ARRAY_SIZE)
                        i -= SAMPLE_ARRAY_SIZE;
                }
                s->rdft_fn(s->rdft, data[ch], data_in[ch], sizeof(float));
                data[ch][0].im = data[ch][nb_freq].re;
                data[ch][nb_freq].re = 0;
            }
            // * Least efficient way to do this, we should of course
            // * directly access it but it is more than fast enough. * /
            if (!SDL_LockTexture(s->vis_texture, &rect, (void **)&pixels, &pitch)) {
                pitch >>= 2;
                pixels += pitch * s->height;
                for (y = 0; y < s->height; y++) {
                    double w = 1 / sqrt(nb_freq);
                    int a = sqrt(w * sqrt(data[0][y].re * data[0][y].re + data[0][y].im * data[0][y].im));
                    int b = (nb_display_channels == 2 ) ? sqrt(w * hypot(data[1][y].re, data[1][y].im))
                                                        : a;
                    a = FFMIN(a, 255);
                    b = FFMIN(b, 255);
                    pixels -= pitch;
                    *pixels = (a << 16) + (b << 8) + ((a+b) >> 1);
                }
                SDL_UnlockTexture(s->vis_texture);
            }
            SDL_RenderCopy(renderer, s->vis_texture, NULL, NULL);
        }
        if (!s->paused)
            s->xpos++;
    }
*/




}

static void stream_component_close(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecParameters *codecpar;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;
    codecpar = ic->streams[stream_index]->codecpar;

    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        decoder_abort(&is->auddec, &is->sampq);
        SDL_CloseAudioDevice(audio_dev);
        decoder_destroy(&is->auddec);
        swr_free(&is->swr_ctx);
        av_freep(&is->audio_buf1);
        is->audio_buf1_size = 0;
        is->audio_buf = NULL;

        if (is->rdft) {
            av_tx_uninit(&is->rdft);
            av_freep(&is->real_data);
            av_freep(&is->rdft_data);
            is->rdft = NULL;
            is->rdft_bits = 0;
        }
        break;
    case AVMEDIA_TYPE_VIDEO:
        decoder_abort(&is->viddec, &is->pictq);
        decoder_destroy(&is->viddec);
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        decoder_abort(&is->subdec, &is->subpq);
        decoder_destroy(&is->subdec);
        break;
    default:
        break;
    }

    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        is->audio_st = NULL;
        is->audio_stream = -1;
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_st = NULL;
        is->video_stream = -1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_st = NULL;
        is->subtitle_stream = -1;
        break;
    default:
        break;
    }
}

static void stream_close(VideoState *is)
{
    /* XXX: use a special url_shutdown call to abort parse cleanly */
    is->abort_request = 1;
    SDL_WaitThread(is->read_tid, NULL);

    /* close each stream */
    if (is->audio_stream >= 0)
        stream_component_close(is, is->audio_stream);
    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);
    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);

    avformat_close_input(&is->ic);

    packet_queue_destroy(&is->videoq);
    packet_queue_destroy(&is->audioq);
    packet_queue_destroy(&is->subtitleq);

    /* free all pictures */
    frame_queue_destroy(&is->pictq);
    frame_queue_destroy(&is->sampq);
    frame_queue_destroy(&is->subpq);
    SDL_DestroyCond(is->continue_read_thread);
    sws_freeContext(is->sub_convert_ctx);
    av_free(is->filename);
    if (is->vis_texture)
        SDL_DestroyTexture(is->vis_texture);
    if (is->vid_texture)
        SDL_DestroyTexture(is->vid_texture);
    if (is->sub_texture)
        SDL_DestroyTexture(is->sub_texture);
        
        
        
      
    //daipozhi modified  
    //av_free(is);
    
    
    
    
    //daipozhi modified
    deb_st_play=0;




}

static void do_exit(VideoState *is)
{
    if (is) {
        stream_close(is);
    }
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (vk_renderer)
        vk_renderer_destroy(vk_renderer);
    if (window)
        SDL_DestroyWindow(window);
    uninit_opts();




    //daipozhi modified
    deb_opts_stt=0;




    for (int i = 0; i < nb_vfilters; i++)
        av_freep(&vfilters_list[i]);
    av_freep(&vfilters_list);
    av_freep(&video_codec_name);
    av_freep(&audio_codec_name);
    av_freep(&subtitle_codec_name);
    av_freep(&input_filename);
    avformat_network_deinit();
    if (show_status)
        printf("\n");
    SDL_Quit();
    av_log(NULL, AV_LOG_QUIET, "%s", "");
    exit(0);
}

static void sigterm_handler(int sig)
{
    exit(123);
}

static void set_default_window_size(int width, int height, AVRational sar)
{
    SDL_Rect rect;
    int max_width  = screen_width  ? screen_width  : INT_MAX;
    int max_height = screen_height ? screen_height : INT_MAX;
    if (max_width == INT_MAX && max_height == INT_MAX)
        max_height = height;
    calculate_display_rect(&rect, 0, 0, max_width, max_height, width, height, sar);
    default_width  = rect.w;
    default_height = rect.h;
}

static int video_open(VideoState *is)
{
    int w,h;





    //daipozhi modified 
    w = 1224 /*screen_width ? screen_width : default_width*/ ;
    h =  642 /*screen_height ? screen_height : default_height*/ ;

    screen_w=w;
    screen_h=h;




    //daipozhi modified 
    //if (!window_title)
    //    window_title = input_filename;
    SDL_SetWindowTitle(window, "Purpose Player 2");




    SDL_SetWindowSize(window, w, h);
    SDL_SetWindowPosition(window, screen_left, screen_top);
    if (is_full_screen)
        SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    SDL_ShowWindow(window);

    is->width  = w;
    is->height = h;

    return 0;
}




//daipozhi modified
static int deb_video_open_again(VideoState *is, int force_set_video_mode)//daipozhi modified
{
    is->width  = screen_w;
    is->height = screen_h;

    SDL_SetWindowTitle(window, deb_dir_buffer);

    return 0;
}




/* display the current picture, if any */
static void video_display(VideoState *is)
{
    if (!is->width)
        video_open(is);




    //daipozhi modified
    //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    //SDL_RenderClear(renderer);
    
    
    
    
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
        video_audio_display(is);
    else if (is->video_st)
        video_image_display(is);
        
        
        
        
    //daipozhi modified
    //SDL_RenderPresent(renderer);
    
    
    
    
}

static double get_clock(Clock *c)
{
    if (*c->queue_serial != c->serial)
        return NAN;
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime_relative() / 1000000.0;
        return c->pts_drift + time - (time - c->last_updated) * (1.0 - c->speed);
    }
}

static void set_clock_at(Clock *c, double pts, int serial, double time)
{
    c->pts = pts;
    c->last_updated = time;
    c->pts_drift = c->pts - time;
    c->serial = serial;
}

static void set_clock(Clock *c, double pts, int serial)
{
    double time = av_gettime_relative() / 1000000.0;
    set_clock_at(c, pts, serial, time);
}

static void set_clock_speed(Clock *c, double speed)
{
    set_clock(c, get_clock(c), c->serial);
    c->speed = speed;
}

static void init_clock(Clock *c, int *queue_serial)
{
    c->speed = 1.0;
    c->paused = 0;
    c->queue_serial = queue_serial;
    set_clock(c, NAN, -1);
}

static void sync_clock_to_slave(Clock *c, Clock *slave)
{
    double clock = get_clock(c);
    double slave_clock = get_clock(slave);
    if (!isnan(slave_clock) && (isnan(clock) || fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD))
        set_clock(c, slave_clock, slave->serial);
}

static int get_master_sync_type(VideoState *is) {
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        if (is->video_st)
            return AV_SYNC_VIDEO_MASTER;
        else
            return AV_SYNC_AUDIO_MASTER;
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        if (is->audio_st)
            return AV_SYNC_AUDIO_MASTER;
        else
            return AV_SYNC_EXTERNAL_CLOCK;
    } else {
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
    double val;

    switch (get_master_sync_type(is)) {
        case AV_SYNC_VIDEO_MASTER:
            val = get_clock(&is->vidclk);
            break;
        case AV_SYNC_AUDIO_MASTER:
            val = get_clock(&is->audclk);
            break;
        default:
            val = get_clock(&is->extclk);
            break;
    }
    return val;
}

static void check_external_clock_speed(VideoState *is) {
   if (is->video_stream >= 0 && is->videoq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES ||
       is->audio_stream >= 0 && is->audioq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES) {
       set_clock_speed(&is->extclk, FFMAX(EXTERNAL_CLOCK_SPEED_MIN, is->extclk.speed - EXTERNAL_CLOCK_SPEED_STEP));
   } else if ((is->video_stream < 0 || is->videoq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES) &&
              (is->audio_stream < 0 || is->audioq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES)) {
       set_clock_speed(&is->extclk, FFMIN(EXTERNAL_CLOCK_SPEED_MAX, is->extclk.speed + EXTERNAL_CLOCK_SPEED_STEP));
   } else {
       double speed = is->extclk.speed;
       if (speed != 1.0)
           set_clock_speed(&is->extclk, speed + EXTERNAL_CLOCK_SPEED_STEP * (1.0 - speed) / fabs(1.0 - speed));
   }
}

/* seek in the stream */
static void stream_seek(VideoState *is, int64_t pos, int64_t rel, int by_bytes)
{
    if (!is->seek_req) {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (by_bytes)
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        is->seek_req = 1;
        SDL_CondSignal(is->continue_read_thread);
    }
}

/* pause or resume the video */
static void stream_toggle_pause(VideoState *is)
{
    if (is->paused) {
        is->frame_timer += av_gettime_relative() / 1000000.0 - is->vidclk.last_updated;
        if (is->read_pause_return != AVERROR(ENOSYS)) {
            is->vidclk.paused = 0;
        }
        set_clock(&is->vidclk, get_clock(&is->vidclk), is->vidclk.serial);
    }
    set_clock(&is->extclk, get_clock(&is->extclk), is->extclk.serial);
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

static void toggle_pause(VideoState *is)
{
    stream_toggle_pause(is);
    is->step = 0;
}

static void toggle_mute(VideoState *is)
{
    is->muted = !is->muted;
}

static void update_volume(VideoState *is, int sign, double step)
{




    //daipozhi modified
    /*
    double volume_level = is->audio_volume ? (20 * log(is->audio_volume / (double)SDL_MIX_MAXVOLUME) / log(10)) : -1000.0;
    int new_volume = lrint(SDL_MIX_MAXVOLUME * pow(10.0, (volume_level + sign * step) / 20.0));
    is->audio_volume = av_clip(is->audio_volume == new_volume ? (is->audio_volume + sign) : new_volume, 0, SDL_MIX_MAXVOLUME);
    */
    
    
    
    
}

static void step_to_next_frame(VideoState *is)
{
    /* if the stream is paused unpause it, then step */
    if (is->paused)
        stream_toggle_pause(is);
    is->step = 1;
}

static double compute_target_delay(double delay, VideoState *is)
{
    double sync_threshold, diff = 0;

    /* update delay to follow master synchronisation source */
    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
        /* if video is slave, we try to correct big delays by
           duplicating or deleting a frame */
        diff = get_clock(&is->vidclk) - get_master_clock(is);

        /* skip or repeat frame. We take into account the
           delay to compute the threshold. I still don't know
           if it is the best guess */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN, FFMIN(AV_SYNC_THRESHOLD_MAX, delay));
        if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
            if (diff <= -sync_threshold)
                delay = FFMAX(0, delay + diff);
            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD)
                delay = delay + diff;
            else if (diff >= sync_threshold)
                delay = 2 * delay;
        }
    }

    av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n",
            delay, -diff);

    return delay;
}

static double vp_duration(VideoState *is, Frame *vp, Frame *nextvp) {
    if (vp->serial == nextvp->serial) {
        double duration = nextvp->pts - vp->pts;
        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration)
            return vp->duration;
        else
            return duration;
    } else {
        return 0.0;
    }
}

static void update_video_pts(VideoState *is, double pts, int serial)
{
    /* update current video pts */
    set_clock(&is->vidclk, pts, serial);
    sync_clock_to_slave(&is->extclk, &is->vidclk);
}

/* called to display each frame */
static void video_refresh(void *opaque, double *remaining_time)
{
    VideoState *is = opaque;
    double time;

    Frame *sp, *sp2;

    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK && is->realtime)
        check_external_clock_speed(is);

    if (!display_disable && is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
        time = av_gettime_relative() / 1000000.0;
        if (is->force_refresh || is->last_vis_time + rdftspeed < time) {
            video_display(is);
            is->last_vis_time = time;
        }
        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
    }

    if (is->video_st) {
retry:
        if (frame_queue_nb_remaining(&is->pictq) == 0) {
            // nothing to do, no picture to display in the queue
        } else {
            double last_duration, duration, delay;
            Frame *vp, *lastvp;

            /* dequeue the picture */
            lastvp = frame_queue_peek_last(&is->pictq);
            vp = frame_queue_peek(&is->pictq);

            if (vp->serial != is->videoq.serial) {
                frame_queue_next(&is->pictq);
                goto retry;
            }

            if (lastvp->serial != vp->serial)
                is->frame_timer = av_gettime_relative() / 1000000.0;

            if (is->paused)
                goto display;

            /* compute nominal last_duration */
            last_duration = vp_duration(is, lastvp, vp);
            delay = compute_target_delay(last_duration, is);

            time= av_gettime_relative()/1000000.0;
            if (time < is->frame_timer + delay) {
                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
                goto display;
            }

            is->frame_timer += delay;
            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
                is->frame_timer = time;

            SDL_LockMutex(is->pictq.mutex);
            if (!isnan(vp->pts))
                update_video_pts(is, vp->pts, vp->serial);
            SDL_UnlockMutex(is->pictq.mutex);

            if (frame_queue_nb_remaining(&is->pictq) > 1) {
                Frame *nextvp = frame_queue_peek_next(&is->pictq);
                duration = vp_duration(is, vp, nextvp);
                if(!is->step && (framedrop>0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) && time > is->frame_timer + duration){
                    is->frame_drops_late++;
                    frame_queue_next(&is->pictq);
                    goto retry;
                }
            }

            if (is->subtitle_st) {
                while (frame_queue_nb_remaining(&is->subpq) > 0) {
                    sp = frame_queue_peek(&is->subpq);

                    if (frame_queue_nb_remaining(&is->subpq) > 1)
                        sp2 = frame_queue_peek_next(&is->subpq);
                    else
                        sp2 = NULL;

                    if (sp->serial != is->subtitleq.serial
                            || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
                            || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))))
                    {
                        if (sp->uploaded) {
                            int i;
                            for (i = 0; i < sp->sub.num_rects; i++) {
                                AVSubtitleRect *sub_rect = sp->sub.rects[i];
                                uint8_t *pixels;
                                int pitch, j;

                                if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
                                    for (j = 0; j < sub_rect->h; j++, pixels += pitch)
                                        memset(pixels, 0, sub_rect->w << 2);
                                    SDL_UnlockTexture(is->sub_texture);
                                }
                            }
                        }
                        frame_queue_next(&is->subpq);
                    } else {
                        break;
                    }
                }
            }

            frame_queue_next(&is->pictq);
            is->force_refresh = 1;

            if (is->step && !is->paused)
                stream_toggle_pause(is);
        }
display:
        /* display picture */
        if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown)
            video_display(is);
    }
    is->force_refresh = 0;
    if (show_status) {
        AVBPrint buf;
        static int64_t last_time;
        int64_t cur_time;
        int aqsize, vqsize, sqsize;
        double av_diff;

        cur_time = av_gettime_relative();
        if (!last_time || (cur_time - last_time) >= 30000) {
            aqsize = 0;
            vqsize = 0;
            sqsize = 0;
            if (is->audio_st)
                aqsize = is->audioq.size;
            if (is->video_st)
                vqsize = is->videoq.size;
            if (is->subtitle_st)
                sqsize = is->subtitleq.size;
            av_diff = 0;
            if (is->audio_st && is->video_st)
                av_diff = get_clock(&is->audclk) - get_clock(&is->vidclk);
            else if (is->video_st)
                av_diff = get_master_clock(is) - get_clock(&is->vidclk);
            else if (is->audio_st)
                av_diff = get_master_clock(is) - get_clock(&is->audclk);

            av_bprint_init(&buf, 0, AV_BPRINT_SIZE_AUTOMATIC);
            av_bprintf(&buf,
                      "%7.2f %s:%7.3f fd=%4d aq=%5dKB vq=%5dKB sq=%5dB \r",
                      get_master_clock(is),
                      (is->audio_st && is->video_st) ? "A-V" : (is->video_st ? "M-V" : (is->audio_st ? "M-A" : "   ")),
                      av_diff,
                      is->frame_drops_early + is->frame_drops_late,
                      aqsize / 1024,
                      vqsize / 1024,
                      sqsize);

            if (show_status == 1 && AV_LOG_INFO > av_log_get_level())
                fprintf(stderr, "%s", buf.str);
            else
                av_log(NULL, AV_LOG_INFO, "%s", buf.str);

            fflush(stderr);
            av_bprint_finalize(&buf, NULL);

            last_time = cur_time;
        }
    }
}

static int queue_picture(VideoState *is, AVFrame *src_frame, double pts, double duration, int64_t pos, int serial)
{
    Frame *vp;

#if defined(DEBUG_SYNC)
    printf("frame_type=%c pts=%0.3f\n",
           av_get_picture_type_char(src_frame->pict_type), pts);
#endif

    if (!(vp = frame_queue_peek_writable(&is->pictq)))
        return -1;

    vp->sar = src_frame->sample_aspect_ratio;
    vp->uploaded = 0;

    vp->width = src_frame->width;
    vp->height = src_frame->height;
    vp->format = src_frame->format;

    vp->pts = pts;
    vp->duration = duration;
    vp->pos = pos;
    vp->serial = serial;

    set_default_window_size(vp->width, vp->height, vp->sar);

    av_frame_move_ref(vp->frame, src_frame);
    frame_queue_push(&is->pictq);
    return 0;
}

static int get_video_frame(VideoState *is, AVFrame *frame)
{
    int got_picture;

    if ((got_picture = decoder_decode_frame(&is->viddec, frame, NULL)) < 0)
        return -1;

    if (got_picture) {
        double dpts = NAN;

        if (frame->pts != AV_NOPTS_VALUE)
            dpts = av_q2d(is->video_st->time_base) * frame->pts;

        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);

        if (framedrop>0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) {
            if (frame->pts != AV_NOPTS_VALUE) {
                double diff = dpts - get_master_clock(is);
                if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD &&
                    diff - is->frame_last_filter_delay < 0 &&
                    is->viddec.pkt_serial == is->vidclk.serial &&
                    is->videoq.nb_packets) {
                    is->frame_drops_early++;
                    av_frame_unref(frame);
                    got_picture = 0;
                }
            }
        }
    }

    return got_picture;
}

static int configure_filtergraph(AVFilterGraph *graph, const char *filtergraph,
                                 AVFilterContext *source_ctx, AVFilterContext *sink_ctx)
{
    int ret, i;
    int nb_filters = graph->nb_filters;
    AVFilterInOut *outputs = NULL, *inputs = NULL;

    if (filtergraph) {
        outputs = avfilter_inout_alloc();
        inputs  = avfilter_inout_alloc();
        if (!outputs || !inputs) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        outputs->name       = av_strdup("in");
        outputs->filter_ctx = source_ctx;
        outputs->pad_idx    = 0;
        outputs->next       = NULL;

        inputs->name        = av_strdup("out");
        inputs->filter_ctx  = sink_ctx;
        inputs->pad_idx     = 0;
        inputs->next        = NULL;

        if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs, NULL)) < 0)
            goto fail;
    } else {
        if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0)
            goto fail;
    }

    /* Reorder the filters to ensure that inputs of the custom filters are merged first */
    for (i = 0; i < graph->nb_filters - nb_filters; i++)
        FFSWAP(AVFilterContext*, graph->filters[i], graph->filters[i + nb_filters]);

    ret = avfilter_graph_config(graph, NULL);
fail:
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    return ret;
}

static int configure_video_filters(AVFilterGraph *graph, VideoState *is, const char *vfilters, AVFrame *frame)
{
    enum AVPixelFormat pix_fmts[FF_ARRAY_ELEMS(sdl_texture_format_map)];
    char sws_flags_str[512] = "";
    char buffersrc_args[256];
    int ret;
    AVFilterContext *filt_src = NULL, *filt_out = NULL, *last_filter = NULL;
    AVCodecParameters *codecpar = is->video_st->codecpar;
    AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL);
    const AVDictionaryEntry *e = NULL;
    int nb_pix_fmts = 0;
    int i, j;
    AVBufferSrcParameters *par = av_buffersrc_parameters_alloc();

    if (!par)
        return AVERROR(ENOMEM);

    for (i = 0; i < renderer_info.num_texture_formats; i++) {
        for (j = 0; j < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; j++) {
            if (renderer_info.texture_formats[i] == sdl_texture_format_map[j].texture_fmt) {
                pix_fmts[nb_pix_fmts++] = sdl_texture_format_map[j].format;
                break;
            }
        }
    }
    pix_fmts[nb_pix_fmts] = AV_PIX_FMT_NONE;

    while ((e = av_dict_iterate(sws_dict, e))) {
        if (!strcmp(e->key, "sws_flags")) {
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", e->value);
        } else
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", e->key, e->value);
    }
    if (strlen(sws_flags_str))
        sws_flags_str[strlen(sws_flags_str)-1] = '\0';

    graph->scale_sws_opts = av_strdup(sws_flags_str);

    snprintf(buffersrc_args, sizeof(buffersrc_args),
             "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d:"
             "colorspace=%d:range=%d",
             frame->width, frame->height, frame->format,
             is->video_st->time_base.num, is->video_st->time_base.den,
             codecpar->sample_aspect_ratio.num, FFMAX(codecpar->sample_aspect_ratio.den, 1),
             frame->colorspace, frame->color_range);
    if (fr.num && fr.den)
        av_strlcatf(buffersrc_args, sizeof(buffersrc_args), ":frame_rate=%d/%d", fr.num, fr.den);

    if ((ret = avfilter_graph_create_filter(&filt_src,
                                            avfilter_get_by_name("buffer"),
                                            "ffplay_buffer", buffersrc_args, NULL,
                                            graph)) < 0)
        goto fail;
    par->hw_frames_ctx = frame->hw_frames_ctx;
    ret = av_buffersrc_parameters_set(filt_src, par);
    if (ret < 0)
        goto fail;

    ret = avfilter_graph_create_filter(&filt_out,
                                       avfilter_get_by_name("buffersink"),
                                       "ffplay_buffersink", NULL, NULL, graph);
    if (ret < 0)
        goto fail;

    if ((ret = av_opt_set_int_list(filt_out, "pix_fmts", pix_fmts,  AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;
    if (!vk_renderer &&
        (ret = av_opt_set_int_list(filt_out, "color_spaces", sdl_supported_color_spaces,  AVCOL_SPC_UNSPECIFIED, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;

    last_filter = filt_out;

/* Note: this macro adds a filter before the lastly added filter, so the
 * processing order of the filters is in reverse */
#define INSERT_FILT(name, arg) do {                                          \
    AVFilterContext *filt_ctx;                                               \
                                                                             \
    ret = avfilter_graph_create_filter(&filt_ctx,                            \
                                       avfilter_get_by_name(name),           \
                                       "ffplay_" name, arg, NULL, graph);    \
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    ret = avfilter_link(filt_ctx, 0, last_filter, 0);                        \
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    last_filter = filt_ctx;                                                  \
} while (0)

    if (autorotate) {
        double theta = 0.0;
        int32_t *displaymatrix = NULL;
        AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_DISPLAYMATRIX);
        if (sd)
            displaymatrix = (int32_t *)sd->data;
        if (!displaymatrix) {
            const AVPacketSideData *psd = av_packet_side_data_get(is->video_st->codecpar->coded_side_data,
                                                                  is->video_st->codecpar->nb_coded_side_data,
                                                                  AV_PKT_DATA_DISPLAYMATRIX);
            if (psd)
                displaymatrix = (int32_t *)psd->data;
        }
        theta = get_rotation(displaymatrix);

        if (fabs(theta - 90) < 1.0) {
            INSERT_FILT("transpose", "clock");
        } else if (fabs(theta - 180) < 1.0) {
            INSERT_FILT("hflip", NULL);
            INSERT_FILT("vflip", NULL);
        } else if (fabs(theta - 270) < 1.0) {
            INSERT_FILT("transpose", "cclock");
        } else if (fabs(theta) > 1.0) {
            char rotate_buf[64];
            snprintf(rotate_buf, sizeof(rotate_buf), "%f*PI/180", theta);
            INSERT_FILT("rotate", rotate_buf);
        }
    }

    if ((ret = configure_filtergraph(graph, vfilters, filt_src, last_filter)) < 0)
        goto fail;

    is->in_video_filter  = filt_src;
    is->out_video_filter = filt_out;

fail:
    av_freep(&par);
    return ret;
}

static int configure_audio_filters(VideoState *is, const char *afilters, int force_output_format)
{
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
    int sample_rates[2] = { 0, -1 };
    AVFilterContext *filt_asrc = NULL, *filt_asink = NULL;
    char aresample_swr_opts[512] = "";
    const AVDictionaryEntry *e = NULL;
    AVBPrint bp;
    char asrc_args[256];
    int ret;

    avfilter_graph_free(&is->agraph);
    if (!(is->agraph = avfilter_graph_alloc()))
        return AVERROR(ENOMEM);
    is->agraph->nb_threads = filter_nbthreads;

    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_AUTOMATIC);

    while ((e = av_dict_iterate(swr_opts, e)))
        av_strlcatf(aresample_swr_opts, sizeof(aresample_swr_opts), "%s=%s:", e->key, e->value);
    if (strlen(aresample_swr_opts))
        aresample_swr_opts[strlen(aresample_swr_opts)-1] = '\0';
    av_opt_set(is->agraph, "aresample_swr_opts", aresample_swr_opts, 0);

    av_channel_layout_describe_bprint(&is->audio_filter_src.ch_layout, &bp);

    ret = snprintf(asrc_args, sizeof(asrc_args),
                   "sample_rate=%d:sample_fmt=%s:time_base=%d/%d:channel_layout=%s",
                   is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
                   1, is->audio_filter_src.freq, bp.str);

    ret = avfilter_graph_create_filter(&filt_asrc,
                                       avfilter_get_by_name("abuffer"), "ffplay_abuffer",
                                       asrc_args, NULL, is->agraph);
    if (ret < 0)
        goto end;


    ret = avfilter_graph_create_filter(&filt_asink,
                                       avfilter_get_by_name("abuffersink"), "ffplay_abuffersink",
                                       NULL, NULL, is->agraph);
    if (ret < 0)
        goto end;

    if ((ret = av_opt_set_int_list(filt_asink, "sample_fmts", sample_fmts,  AV_SAMPLE_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;
    if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 1, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;

    if (force_output_format) {
        av_bprint_clear(&bp);
        av_channel_layout_describe_bprint(&is->audio_tgt.ch_layout, &bp);
        sample_rates   [0] = is->audio_tgt.freq;
        if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 0, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set(filt_asink, "ch_layouts", bp.str, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set_int_list(filt_asink, "sample_rates"   , sample_rates   ,  -1, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
    }


    if ((ret = configure_filtergraph(is->agraph, afilters, filt_asrc, filt_asink)) < 0)
        goto end;

    is->in_audio_filter  = filt_asrc;
    is->out_audio_filter = filt_asink;

end:
    if (ret < 0)
        avfilter_graph_free(&is->agraph);
    av_bprint_finalize(&bp, NULL);

    return ret;
}

static int audio_thread(void *arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    Frame *af;
    int last_serial = -1;
    int reconfigure;
    int got_frame = 0;
    AVRational tb;
    int ret = 0;

    if (!frame)
        return AVERROR(ENOMEM);

    do {
        if ((got_frame = decoder_decode_frame(&is->auddec, frame, NULL)) < 0)
            goto the_end;

        if (got_frame) {
                tb = (AVRational){1, frame->sample_rate};

                reconfigure =
                    cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.ch_layout.nb_channels,
                                   frame->format, frame->ch_layout.nb_channels)    ||
                    av_channel_layout_compare(&is->audio_filter_src.ch_layout, &frame->ch_layout) ||
                    is->audio_filter_src.freq           != frame->sample_rate ||
                    is->auddec.pkt_serial               != last_serial;

                if (reconfigure) {
                    char buf1[1024], buf2[1024];
                    av_channel_layout_describe(&is->audio_filter_src.ch_layout, buf1, sizeof(buf1));
                    av_channel_layout_describe(&frame->ch_layout, buf2, sizeof(buf2));
                    av_log(NULL, AV_LOG_DEBUG,
                           "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
                           is->audio_filter_src.freq, is->audio_filter_src.ch_layout.nb_channels, av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
                           frame->sample_rate, frame->ch_layout.nb_channels, av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);

                    is->audio_filter_src.fmt            = frame->format;
                    ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &frame->ch_layout);
                    if (ret < 0)
                        goto the_end;
                    is->audio_filter_src.freq           = frame->sample_rate;
                    last_serial                         = is->auddec.pkt_serial;

                    if ((ret = configure_audio_filters(is, afilters, 1)) < 0)
                        goto the_end;
                }

            if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0)
                goto the_end;

            while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame, 0)) >= 0) {
                FrameData *fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL;
                tb = av_buffersink_get_time_base(is->out_audio_filter);
                if (!(af = frame_queue_peek_writable(&is->sampq)))
                    goto the_end;

                af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
                af->pos = fd ? fd->pkt_pos : -1;
                af->serial = is->auddec.pkt_serial;
                af->duration = av_q2d((AVRational){frame->nb_samples, frame->sample_rate});

                av_frame_move_ref(af->frame, frame);
                frame_queue_push(&is->sampq);

                if (is->audioq.serial != is->auddec.pkt_serial)
                    break;
            }
            if (ret == AVERROR_EOF)
                is->auddec.finished = is->auddec.pkt_serial;




	    deb_thr_a=1;   //daipozhi modified




        }
    } while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);
 the_end:
    avfilter_graph_free(&is->agraph);
    av_frame_free(&frame);
    return ret;
}

static int decoder_start(Decoder *d, int (*fn)(void *), const char *thread_name, void* arg)
{
    packet_queue_start(d->queue);
    d->decoder_tid = SDL_CreateThread(fn, thread_name, arg);
    if (!d->decoder_tid) {
        av_log(NULL, AV_LOG_ERROR, "SDL_CreateThread(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    return 0;
}

static int video_thread(void *arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    double pts;
    double duration;
    int ret;
    AVRational tb = is->video_st->time_base;
    AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL);

    AVFilterGraph *graph = NULL;
    AVFilterContext *filt_out = NULL, *filt_in = NULL;
    int last_w = 0;
    int last_h = 0;
    enum AVPixelFormat last_format = -2;
    int last_serial = -1;
    int last_vfilter_idx = 0;

    if (!frame)
        return AVERROR(ENOMEM);

    for (;;) {
        ret = get_video_frame(is, frame);
        if (ret < 0)
            goto the_end;
        if (!ret)
            continue;

        if (   last_w != frame->width
            || last_h != frame->height
            || last_format != frame->format
            || last_serial != is->viddec.pkt_serial
            || last_vfilter_idx != is->vfilter_idx) {
            av_log(NULL, AV_LOG_DEBUG,
                   "Video frame changed from size:%dx%d format:%s serial:%d to size:%dx%d format:%s serial:%d\n",
                   last_w, last_h,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(last_format), "none"), last_serial,
                   frame->width, frame->height,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(frame->format), "none"), is->viddec.pkt_serial);
            avfilter_graph_free(&graph);
            graph = avfilter_graph_alloc();
            if (!graph) {
                ret = AVERROR(ENOMEM);
                goto the_end;
            }
            graph->nb_threads = filter_nbthreads;
            if ((ret = configure_video_filters(graph, is, vfilters_list ? vfilters_list[is->vfilter_idx] : NULL, frame)) < 0) {
                SDL_Event event;
                event.type = FF_QUIT_EVENT;
                event.user.data1 = is;
                SDL_PushEvent(&event);
                goto the_end;
            }
            filt_in  = is->in_video_filter;
            filt_out = is->out_video_filter;
            last_w = frame->width;
            last_h = frame->height;
            last_format = frame->format;
            last_serial = is->viddec.pkt_serial;
            last_vfilter_idx = is->vfilter_idx;
            frame_rate = av_buffersink_get_frame_rate(filt_out);
        }

        ret = av_buffersrc_add_frame(filt_in, frame);
        if (ret < 0)
            goto the_end;

        while (ret >= 0) {
            FrameData *fd;

            is->frame_last_returned_time = av_gettime_relative() / 1000000.0;

            ret = av_buffersink_get_frame_flags(filt_out, frame, 0);
            if (ret < 0) {
                if (ret == AVERROR_EOF)
                    is->viddec.finished = is->viddec.pkt_serial;
                ret = 0;
                break;
            }

            fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL;

            is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
            if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0)
                is->frame_last_filter_delay = 0;
            tb = av_buffersink_get_time_base(filt_out);
            duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational){frame_rate.den, frame_rate.num}) : 0);
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
            ret = queue_picture(is, frame, pts, duration, fd ? fd->pkt_pos : -1, is->viddec.pkt_serial);
            av_frame_unref(frame);
            if (is->videoq.serial != is->viddec.pkt_serial)
                break;
        }

        if (ret < 0)
            goto the_end;




	deb_thr_v=1;  //daipozhi modified 




    }
 the_end:
    avfilter_graph_free(&graph);
    av_frame_free(&frame);
    return 0;
}

static int subtitle_thread(void *arg)
{
    VideoState *is = arg;
    Frame *sp;
    int got_subtitle;
    double pts;

    for (;;) {
        if (!(sp = frame_queue_peek_writable(&is->subpq)))
            return 0;

        if ((got_subtitle = decoder_decode_frame(&is->subdec, NULL, &sp->sub)) < 0)
            break;

        pts = 0;

        if (got_subtitle && sp->sub.format == 0) {
            if (sp->sub.pts != AV_NOPTS_VALUE)
                pts = sp->sub.pts / (double)AV_TIME_BASE;
            sp->pts = pts;
            sp->serial = is->subdec.pkt_serial;
            sp->width = is->subdec.avctx->width;
            sp->height = is->subdec.avctx->height;
            sp->uploaded = 0;

            /* now we can update the picture count */
            frame_queue_push(&is->subpq);
        } else if (got_subtitle) {
            avsubtitle_free(&sp->sub);
        }




	deb_thr_s=1;  //daipozhi modified




    }
    return 0;
}




// daipozhi modified
#if DPZ_DEBUG1
static char m711_str1[300];
#endif




/* copy samples for viewing in editor window */
static void update_sample_display(VideoState *is, short *samples, int samples_size)
{
    int size, len;




    // daipozhi modified
    int l,m;
#if DPZ_DEBUG1
    int k;
#endif




    // daipozhi modified for sound river
    if (deb_sr_show_start==1) deb_sr_total_bytes=deb_sr_total_bytes+samples_size;
#if DPZ_DEBUG1
    k=samples_size/(2*deb_sr_ch);
    sprintf(m711_str1,"income samples,%d,",k);
    deb_record(m711_str1);
#endif




    size = samples_size / sizeof(short);
    while (size > 0) {




        //daipozhi modified
        len = /*SAMPLE_ARRAY_SIZE*/ deb_sr_sample_size - is->sample_array_index;
        
        
        
        
        if (len > size)
            len = size;
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        samples += len;
        is->sample_array_index += len;
        
        
        
        
        //daipozhi modified
        if (is->sample_array_index >= /*SAMPLE_ARRAY_SIZE*/ deb_sr_sample_size )
        {
            is->sample_array_index = 0;




	    // daipozhi modified for sound river
	    deb_sr_sample_over=1;
	}    
	    
	    
        	    
        
        size -= len;
    }




    // daipozhi modified for sound river  ---------------------------------------
    if ((deb_sr_show_start==1)&&(deb_sr_show==1))
    {
	if (deb_sr_show_init==0)
	{
	    deb_sr_show_init=1;
#if DPZ_DEBUG2
	    deb_sr_fft_deb_ptr1=0;
	    deb_sr_fft_deb_ptr2=0;
	    deb_sr_fft_deb_ptr3=0;
	    deb_sr_sdl_callback_cnt=0;
	    deb_sr_fft_deb_chn=0;
#endif
	    for (l=0;l<396;l++)
	    {
		for (m=0;m<FFT_BUFFER_SIZE/2;m++) deb_sr_river[l][m]=0;
		deb_sr_river_mark[l]=0;
	    }

	    for (l=0;l<150;l++)
	    {
		for (m=0;m<FFT_BUFFER_SIZE/2;m++) deb_sr_river2[l][m]=0;
	    }


	    l=is->sample_array_index;
	    l=(FFT_BUFFER_SIZE*deb_sr_ch)*(l/(FFT_BUFFER_SIZE*deb_sr_ch));		// fft start and end
	    m=l;
	    deb_sr_fft_start=l;
	    deb_sr_fft_end=m;

	    return;
	}
	else
	{
	    deb_sr_sample_over2=0;

	    if (deb_sr_fft_end<=is->sample_array_index)	// sample position of now
	    {
		l=deb_sr_fft_end;		// fft start and end
		m=(FFT_BUFFER_SIZE*deb_sr_ch)*(is->sample_array_index/(FFT_BUFFER_SIZE*deb_sr_ch));
		deb_sr_fft_start=l;
		deb_sr_fft_end=m;
	    }
	    else
	    {
		if (deb_sr_sample_over==1)
		{
			l=deb_sr_fft_end; // ring buffer
			m=(FFT_BUFFER_SIZE*deb_sr_ch)*(is->sample_array_index/(FFT_BUFFER_SIZE*deb_sr_ch));
			deb_sr_fft_start=l;
			deb_sr_fft_end=m;
			deb_sr_sample_over2=1;
		}
		else
		{
			deb_sr_fft_start=deb_sr_fft_end;
		}
	    }

	    if (deb_sr_fft_start==deb_sr_fft_end) return;
	    if ((deb_sr_fft_start==deb_sr_sample_size)&&(deb_sr_fft_end==0)) return;
#if DPZ_DEBUG1
	    sprintf(m711_str1,"next,fft start=%d,fft end=%d,samp index=%d,",deb_sr_fft_start,deb_sr_fft_end,is->sample_array_index);
            deb_record(m711_str1);
#endif
	}

        // start fft
	deb_sr_fft_trans_all(is,deb_sr_rate);
    }




}

/* return the wanted number of samples to get better sync if sync_type is video
 * or external master clock */
static int synchronize_audio(VideoState *is, int nb_samples)
{
    int wanted_nb_samples = nb_samples;

    /* if not master, then we try to remove or add samples to correct the clock */
    if (get_master_sync_type(is) != AV_SYNC_AUDIO_MASTER) {
        double diff, avg_diff;
        int min_nb_samples, max_nb_samples;

        diff = get_clock(&is->audclk) - get_master_clock(is);

        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD) {
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum;
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
                /* not enough measures to have a correct estimate */
                is->audio_diff_avg_count++;
            } else {
                /* estimate the A-V difference */
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);

                if (fabs(avg_diff) >= is->audio_diff_threshold) {
                    wanted_nb_samples = nb_samples + (int)(diff * is->audio_src.freq);
                    min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    wanted_nb_samples = av_clip(wanted_nb_samples, min_nb_samples, max_nb_samples);
                }
                av_log(NULL, AV_LOG_TRACE, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                        diff, avg_diff, wanted_nb_samples - nb_samples,
                        is->audio_clock, is->audio_diff_threshold);
            }
        } else {
            /* too big difference : may be initial PTS errors, so
               reset A-V filter */
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum       = 0;
        }
    }

    return wanted_nb_samples;
}

/**
 * Decode one audio frame and return its uncompressed size.
 *
 * The processed audio frame is decoded, converted if required, and
 * stored in is->audio_buf, with size in bytes given by the return
 * value.
 */
static int audio_decode_frame(VideoState *is)
{
    int data_size, resampled_data_size;
    av_unused double audio_clock0;
    int wanted_nb_samples;
    Frame *af;

    if (is->paused)
        return -1;

    do {
#if defined(_WIN32)
        while (frame_queue_nb_remaining(&is->sampq) == 0) {
            if ((av_gettime_relative() - audio_callback_time) > 1000000LL * is->audio_hw_buf_size / is->audio_tgt.bytes_per_sec / 2)
                return -1;
            av_usleep (1000);
        }
#endif
        if (!(af = frame_queue_peek_readable(&is->sampq)))
            return -1;
        frame_queue_next(&is->sampq);
    } while (af->serial != is->audioq.serial);

    data_size = av_samples_get_buffer_size(NULL, af->frame->ch_layout.nb_channels,
                                           af->frame->nb_samples,
                                           af->frame->format, 1);

    wanted_nb_samples = synchronize_audio(is, af->frame->nb_samples);

    if (af->frame->format        != is->audio_src.fmt            ||
        av_channel_layout_compare(&af->frame->ch_layout, &is->audio_src.ch_layout) ||
        af->frame->sample_rate   != is->audio_src.freq           ||
        (wanted_nb_samples       != af->frame->nb_samples && !is->swr_ctx)) {
        int ret;
        swr_free(&is->swr_ctx);
        ret = swr_alloc_set_opts2(&is->swr_ctx,
                            &is->audio_tgt.ch_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
                            &af->frame->ch_layout, af->frame->format, af->frame->sample_rate,
                            0, NULL);
        if (ret < 0 || swr_init(is->swr_ctx) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                    af->frame->sample_rate, av_get_sample_fmt_name(af->frame->format), af->frame->ch_layout.nb_channels,
                    is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.ch_layout.nb_channels);
            swr_free(&is->swr_ctx);
            return -1;
        }
        if (av_channel_layout_copy(&is->audio_src.ch_layout, &af->frame->ch_layout) < 0)
            return -1;
        is->audio_src.freq = af->frame->sample_rate;
        is->audio_src.fmt = af->frame->format;
    }

    if (is->swr_ctx) {
        const uint8_t **in = (const uint8_t **)af->frame->extended_data;
        uint8_t **out = &is->audio_buf1;
        int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate + 256;
        int out_size  = av_samples_get_buffer_size(NULL, is->audio_tgt.ch_layout.nb_channels, out_count, is->audio_tgt.fmt, 0);
        int len2;
        if (out_size < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
            return -1;
        }
        if (wanted_nb_samples != af->frame->nb_samples) {
            if (swr_set_compensation(is->swr_ctx, (wanted_nb_samples - af->frame->nb_samples) * is->audio_tgt.freq / af->frame->sample_rate,
                                        wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate) < 0) {
                av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
                return -1;
            }
        }
        av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size, out_size);
        if (!is->audio_buf1)
            return AVERROR(ENOMEM);
        len2 = swr_convert(is->swr_ctx, out, out_count, in, af->frame->nb_samples);
        if (len2 < 0) {
            av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
            return -1;
        }
        if (len2 == out_count) {
            av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too small\n");
            if (swr_init(is->swr_ctx) < 0)
                swr_free(&is->swr_ctx);
        }
        is->audio_buf = is->audio_buf1;
        resampled_data_size = len2 * is->audio_tgt.ch_layout.nb_channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
    } else {
        is->audio_buf = af->frame->data[0];
        resampled_data_size = data_size;
    }

    audio_clock0 = is->audio_clock;
    /* update the audio clock with the pts */
    if (!isnan(af->pts))
        is->audio_clock = af->pts + (double) af->frame->nb_samples / af->frame->sample_rate;
    else
        is->audio_clock = NAN;
    is->audio_clock_serial = af->serial;
#ifdef DEBUG
    {
        static double last_clock;
        printf("audio: delay=%0.3f clock=%0.3f clock0=%0.3f\n",
               is->audio_clock - last_clock,
               is->audio_clock, audio_clock0);
        last_clock = is->audio_clock;
    }
#endif




    deb_thr_a2=1;   //daipozhi modified




    return resampled_data_size;
}

/* prepare a new audio buffer */
static void sdl_audio_callback(void *opaque, Uint8 *stream, int len)
{
    VideoState *is = opaque;
    int audio_size, len1;




    // daipozhi for sound river
#if DPZ_DEBUG2
    int len2;
    int len3;
Uint8 *stream2;
#endif
    int len4;
    int i,j,l,m,n;
    int rept/*,over*/,start;
    int size,size1,indx;
    uint8_t *nstr;

    len4=len;
    size=deb_resam_buff2_ptr;
    indx=deb_resam_buff2_indx;




    audio_callback_time = av_gettime_relative();

    while (len > 0) {




         // daipozhi modified
         //if (is->audio_buf_index >= is->audio_buf_size) {
         if (indx >= size) {




           audio_size = audio_decode_frame(is);
           if (audio_size < 0) {
                /* if error, just output silence */
               is->audio_buf = NULL;
               is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE / is->audio_tgt.frame_size * is->audio_tgt.frame_size;




               //daipozhi for sound river
               size=is->audio_buf_size;
               for (i=0;i<size;i++) deb_resam_buff2[i]=0;




           } else {




               //daipozhi for sound river  
               // need resample,so sound river is fix speed,frequency's position is fixed

               for (i=0;i<deb_resam_buff3_ptr;i++) deb_resam_buff1[i]=deb_resam_buff3[i];  // left data in before

               nstr=is->audio_buf;  

               for (i=0;i<audio_size;i++) deb_resam_buff1[deb_resam_buff3_ptr+i]=nstr[i];  // copy new data

               j=deb_resam_buff3_ptr+audio_size;  // total data

               if (deb_resam_dir==1) // both are 44.1khz , don't need convert
               {
                 for (i=0;i<j;i++) deb_resam_buff2[i]=deb_resam_buff1[i];
                 size=j;
                 deb_resam_buff3_ptr=0;
               }
               else if (deb_sr_rate==48000)
               {
                 size=0;
                 i=0;
                 rept=0;
                 l=deb_resam_last_ptr;
                 m=deb_resam_f[l];
                 start=m;

                 while (1)
                 {
                     if (deb_resam_last_ptr+i-rept*147>=147)
                     {
                       rept++;
                     }
                     
                     l=deb_resam_last_ptr+i-rept*147;
                     m=deb_resam_f[l];

                     if (  160*rept-start+m  <  j/(deb_sr_ch*2)  )
                     {

                       for (n=0;n<deb_sr_ch;n++)
                       {

                         deb_resam_buff2[(i*deb_sr_ch+n)*2+0]=deb_resam_buff1[( (160*rept-start+m)*deb_sr_ch+n )*2+0];
                         deb_resam_buff2[(i*deb_sr_ch+n)*2+1]=deb_resam_buff1[( (160*rept-start+m)*deb_sr_ch+n )*2+1];
                         if (size<i) size=i;
                       }

                     }
                     else
                     {
                         deb_resam_last_ptr=deb_resam_last_ptr+size-((deb_resam_last_ptr+size)/147)*147;
                       
                         break;
                     }

                     i++;
                 }

                 for (i=  ((160*rept-start+m)*deb_sr_ch+2)*2  ;i<j;i++) deb_resam_buff3[i-  ((160*rept-start+m)*deb_sr_ch+2)*2  ]=deb_resam_buff1[i];
                 deb_resam_buff3_ptr=j-  ((160*rept-start+m)*deb_sr_ch+2)*2  ;
                 size=(size+1)*deb_sr_ch*2;
               }
               else if (deb_sr_rate==32000)
               {
                 size=0;
                 i=0;
                 rept=0;
                 l=deb_resam_last_ptr;
                 m=deb_resam_f[l];
                 start=m;

                 while (1)
                 {
                     if (deb_resam_last_ptr+i-rept*441>=441)
                     {
                       rept++;
                     }
                     
                     l=deb_resam_last_ptr+i-rept*441;
                     m=deb_resam_f[l];

                     if (  320*rept-start+m  <  j/(deb_sr_ch*2)  )
                     {

                       for (n=0;n<deb_sr_ch;n++)
                       {

                         deb_resam_buff2[(i*deb_sr_ch+n)*2+0]=deb_resam_buff1[( (320*rept-start+m)*deb_sr_ch+n )*2+0];
                         deb_resam_buff2[(i*deb_sr_ch+n)*2+1]=deb_resam_buff1[( (320*rept-start+m)*deb_sr_ch+n )*2+1];
                         if (size<i) size=i;

                       }

                     }
                     else
                     {
                         deb_resam_last_ptr=deb_resam_last_ptr+size-((deb_resam_last_ptr+size)/441)*441;
                       
                         break;
                     }

                     i++;
                 }

                 for (i=  ((320*rept-start+m)*deb_sr_ch+2)*2  ;i<j;i++) deb_resam_buff3[i-  ((320*rept-start+m)*deb_sr_ch+2)*2  ]=deb_resam_buff1[i];
                 deb_resam_buff3_ptr=j-  ((320*rept-start+m)*deb_sr_ch+2)*2  ;
                 size=(size+1)*deb_sr_ch*2;
               }
               else size=j;




               //if (is->show_mode != SHOW_MODE_VIDEO)
               //    update_sample_display(is, (int16_t *)is->audio_buf, audio_size);
               update_sample_display(is, (int16_t *)deb_resam_buff2, size);




               is->audio_buf_size = audio_size;
           }
           is->audio_buf_index = 0;




           // daipozhi for sound river
           indx=0;




        }
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len)
            len1 = len;




        //daipozhi for sound river
        size1 = size - indx;
        if (size1>len)
            size1=len;




// daipozhi modified for sound river
#if DPZ_DEBUG2
	len2=0;            //to playback high freq,low freq,middle freq audio,to sure fft is ok,
	len3=0;            //set DPZ_DEBUG2 to 1,need click '[river on]',press key 'm' to switch chanel
	stream2=stream;

	while (len2<len1)
	{
	   if (deb_sr_fft_deb_ptr1+len3<FFT_BUFFER_SIZE*2*deb_sr_ch)  
           {
	     if ( deb_sr_sdl_callback_cnt>=FFT_BUFFER_SIZE*2*deb_sr_ch*2 ) 
	     {
        	if (!is->muted && is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME)
		{
            		memcpy(stream2, (uint8_t *)(deb_sr_fft_deb[deb_sr_fft_deb_ptr3]) + deb_sr_fft_deb_ptr1+len3, 1);
		}
        	else 
		{
            		memset(stream2, 0, 1);
            		if (!is->muted && is->audio_buf)
                	SDL_MixAudioFormat(stream2, (uint8_t *)(deb_sr_fft_deb[deb_sr_fft_deb_ptr3]) + deb_sr_fft_deb_ptr1+len3, AUDIO_S16SYS, 1, is->audio_volume);
       		}

		len2=len2+1;
		len3=len3+1;
		stream2=stream2+1;
	     }
	     else
	     {
		len2=len2+1;
		stream2=stream2+1;

	     }
	   }
	   else
	   {
		deb_sr_fft_deb_ptr3++;
		if (deb_sr_fft_deb_ptr3>=4) deb_sr_fft_deb_ptr3=0;

		deb_sr_fft_deb_ptr1=0;
		len3=0;
	   }

	}
	
	deb_sr_fft_deb_ptr1=deb_sr_fft_deb_ptr1+len3;
#else
        if (!is->muted && is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME)
            //memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
            memcpy(stream, (uint8_t *)deb_resam_buff2 + indx, size1);
        else {
            //memset(stream, 0, len1);
            //if (!is->muted && is->audio_buf)
            //    SDL_MixAudioFormat(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, AUDIO_S16SYS, len1, is->audio_volume);
            memset(stream, 0, size1);
            if (!is->muted && is->audio_buf)
                  SDL_MixAudioFormat(stream, (uint8_t *)deb_resam_buff2 + indx, AUDIO_S16SYS, size1, is->audio_volume);
        }
#endif




        // daipozhi for sound river
        //len -= len1;
        //stream += len1;
        is->audio_buf_index += len1;
        
        
        
        
        // daipozhi for sound river
        len=len-size1;
        stream=stream+size1;
        indx=indx+size1;

        deb_resam_buff2_ptr =size;
        deb_resam_buff2_indx=indx;




    }
    is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index;
    /* Let's assume the audio driver that is used by SDL has two periods. */
    if (!isnan(is->audio_clock)) {
        set_clock_at(&is->audclk, is->audio_clock - (double)(2 * is->audio_hw_buf_size + is->audio_write_buf_size) / is->audio_tgt.bytes_per_sec, is->audio_clock_serial, audio_callback_time / 1000000.0);
        sync_clock_to_slave(&is->extclk, &is->audclk);
    }




    // daipozhi for sound river
    if ( deb_sr_sdl_callback_cnt<FFT_BUFFER_SIZE*2*deb_sr_ch*2 ) deb_sr_sdl_callback_cnt=deb_sr_sdl_callback_cnt+len4;




}




// daipozhi for sound river
#if DPZ_DEBUG1
static char m702_str1[300];
#endif




static int audio_open(void *opaque, AVChannelLayout *wanted_channel_layout, int wanted_sample_rate, struct AudioParams *audio_hw_params)
{
    SDL_AudioSpec wanted_spec, spec;
    const char *env;
    static const int next_nb_channels[] = {0, 0, 1, 6, 2, 6, 4, 6};
    static const int next_sample_rates[] = {0, 44100, 48000, 96000, 192000};
    int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1;
    int wanted_nb_channels = wanted_channel_layout->nb_channels;




    //daipozhi modified for sound river
    int i;
    deb_sr_show=0;
    deb_sr_rate=0;
    deb_sr_ch  =0;
    deb_resam_rate=44100;
    deb_resam_dir =0;




    env = SDL_getenv("SDL_AUDIO_CHANNELS");
    if (env) {
        wanted_nb_channels = atoi(env);
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    wanted_nb_channels = wanted_channel_layout->nb_channels;
    wanted_spec.channels = wanted_nb_channels;




    // daipozhi for sound river
    wanted_spec.freq = deb_resam_rate /*wanted_sample_rate*/;
    
    
    
    
    if (wanted_spec.freq <= 0 || wanted_spec.channels <= 0) {
        av_log(NULL, AV_LOG_ERROR, "Invalid sample rate or channel count!\n");
        return -1;
    }
    while (next_sample_rate_idx && next_sample_rates[next_sample_rate_idx] >= wanted_spec.freq)
        next_sample_rate_idx--;
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.silence = 0;
    wanted_spec.samples = FFMAX(SDL_AUDIO_MIN_BUFFER_SIZE, 2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC));
    wanted_spec.callback = sdl_audio_callback;
    wanted_spec.userdata = opaque;
    while (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE | SDL_AUDIO_ALLOW_CHANNELS_CHANGE))) {
        av_log(NULL, AV_LOG_WARNING, "SDL_OpenAudio (%d channels, %d Hz): %s\n",
               wanted_spec.channels, wanted_spec.freq, SDL_GetError());
        wanted_spec.channels = next_nb_channels[FFMIN(7, wanted_spec.channels)];
        if (!wanted_spec.channels) {
            wanted_spec.freq = next_sample_rates[next_sample_rate_idx--];
            wanted_spec.channels = wanted_nb_channels;
            if (!wanted_spec.freq) {
                av_log(NULL, AV_LOG_ERROR,
                       "No more combinations to try, audio open failed\n");
                return -1;
            }
        }
        av_channel_layout_default(wanted_channel_layout, wanted_spec.channels);
    }
    if (spec.format != AUDIO_S16SYS) {
        av_log(NULL, AV_LOG_ERROR,
               "SDL advised audio format %d is not supported!\n", spec.format);
        return -1;
    }
    if (spec.channels != wanted_spec.channels) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, spec.channels);
        if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
            av_log(NULL, AV_LOG_ERROR,
                   "SDL advised channel count %d is not supported!\n", spec.channels);
            return -1;
        }
    }

    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = spec.freq;
    if (av_channel_layout_copy(&audio_hw_params->ch_layout, wanted_channel_layout) < 0)
        return -1;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
    if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0) {
        av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size failed\n");
        return -1;
    }




    //daipozhi modified for sound river
    deb_sr_rate=spec.freq;
    deb_sr_ch  =spec.channels;
    
    if (spec.freq==32000)
    {
	deb_sr_show=1;
    }
    else if (spec.freq==44100)
    {
	deb_sr_show=1;
    }
    else if (spec.freq==48000)
    {
	deb_sr_show=1;
    }

    deb_sr_sample_size=(SAMPLE_ARRAY_SIZE/(spec.channels*FFT_BUFFER_SIZE))*spec.channels*FFT_BUFFER_SIZE;

#if DPZ_DEBUG1
    sprintf(m702_str1,"init show=%d,rate=%d,ch=%d,",deb_sr_show,deb_sr_rate,deb_sr_ch);
    deb_record(m702_str1);
#endif

    if (spec.freq==44100)
    {
	deb_resam_dir=1;
    }
    else if (spec.freq==48000)
    {
	deb_resam_dir=0; // need resample,so sound river is fix speed,frequency's position is fixed

        //44.1khz:48khz == 147:160

        for (i=0;i<147;i++)
        {
          deb_resam_f[i]= i*160/147;
        }

        deb_resam_buff1_ptr=0;
        deb_resam_buff2_ptr=0;
        deb_resam_buff3_ptr=0;
        deb_resam_buff2_indx=0;
        deb_resam_last_ptr=0;
    }
    else if (spec.freq==32000)
    {
	deb_resam_dir=0; // need resample,so sound river is fix speed,frequency's position is fixed

        //44.1khz:32khz ==  441:320

        for (i=0;i<441;i++)
        {
          deb_resam_f[i]= i*320/441;
        }

        deb_resam_buff1_ptr=0;
        deb_resam_buff2_ptr=0;
        deb_resam_buff3_ptr=0;
        deb_resam_buff2_indx=0;
        deb_resam_last_ptr=0;
    }




    return spec.size;
}

static int create_hwaccel(AVBufferRef **device_ctx)
{
    enum AVHWDeviceType type;
    int ret;
    AVBufferRef *vk_dev;

    *device_ctx = NULL;

    if (!hwaccel)
        return 0;

    type = av_hwdevice_find_type_by_name(hwaccel);
    if (type == AV_HWDEVICE_TYPE_NONE)
        return AVERROR(ENOTSUP);

    ret = vk_renderer_get_hw_dev(vk_renderer, &vk_dev);
    if (ret < 0)
        return ret;

    ret = av_hwdevice_ctx_create_derived(device_ctx, type, vk_dev, 0);
    if (!ret)
        return 0;

    if (ret != AVERROR(ENOSYS))
        return ret;

    av_log(NULL, AV_LOG_WARNING, "Derive %s from vulkan not supported.\n", hwaccel);
    ret = av_hwdevice_ctx_create(device_ctx, type, NULL, NULL, 0);
    return ret;
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    const AVCodec *codec;
    const char *forced_codec_name = NULL;
    AVDictionary *opts = NULL;
    const AVDictionaryEntry *t = NULL;
    int sample_rate;
    AVChannelLayout ch_layout = { 0 };
    int ret = 0;
    int stream_lowres = lowres;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return -1;

    avctx = avcodec_alloc_context3(NULL);
    if (!avctx)
        return AVERROR(ENOMEM);

    ret = avcodec_parameters_to_context(avctx, ic->streams[stream_index]->codecpar);
    if (ret < 0)
        goto fail;
    avctx->pkt_timebase = ic->streams[stream_index]->time_base;

    codec = avcodec_find_decoder(avctx->codec_id);

    switch(avctx->codec_type){
        case AVMEDIA_TYPE_AUDIO   : is->last_audio_stream    = stream_index; forced_codec_name =    audio_codec_name; break;
        case AVMEDIA_TYPE_SUBTITLE: is->last_subtitle_stream = stream_index; forced_codec_name = subtitle_codec_name; break;
        case AVMEDIA_TYPE_VIDEO   : is->last_video_stream    = stream_index; forced_codec_name =    video_codec_name; break;
    }
    if (forced_codec_name)
        codec = avcodec_find_decoder_by_name(forced_codec_name);
    if (!codec) {
        if (forced_codec_name) av_log(NULL, AV_LOG_WARNING,
                                      "No codec could be found with name '%s'\n", forced_codec_name);
        else                   av_log(NULL, AV_LOG_WARNING,
                                      "No decoder could be found for codec %s\n", avcodec_get_name(avctx->codec_id));
        ret = AVERROR(EINVAL);
        goto fail;
    }

    avctx->codec_id = codec->id;
    if (stream_lowres > codec->max_lowres) {
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d\n",
                codec->max_lowres);
        stream_lowres = codec->max_lowres;
    }
    avctx->lowres = stream_lowres;

    if (fast)
        avctx->flags2 |= AV_CODEC_FLAG2_FAST;

    ret = filter_codec_opts(codec_opts, avctx->codec_id, ic,
                            ic->streams[stream_index], codec, &opts);
    if (ret < 0)
        goto fail;

    if (!av_dict_get(opts, "threads", NULL, 0))
        av_dict_set(&opts, "threads", "auto", 0);
    if (stream_lowres)
        av_dict_set_int(&opts, "lowres", stream_lowres, 0);

    av_dict_set(&opts, "flags", "+copy_opaque", AV_DICT_MULTIKEY);

    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO) {
        ret = create_hwaccel(&avctx->hw_device_ctx);
        if (ret < 0)
            goto fail;
    }

    if ((ret = avcodec_open2(avctx, codec, &opts)) < 0) {
        goto fail;
    }
    if ((t = av_dict_get(opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        ret =  AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }

    is->eof = 0;
    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;
    switch (avctx->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        {
            AVFilterContext *sink;

            is->audio_filter_src.freq           = avctx->sample_rate;
            ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &avctx->ch_layout);
            if (ret < 0)
                goto fail;
            is->audio_filter_src.fmt            = avctx->sample_fmt;
            if ((ret = configure_audio_filters(is, afilters, 0)) < 0)
                goto fail;
            sink = is->out_audio_filter;
            sample_rate    = av_buffersink_get_sample_rate(sink);
            ret = av_buffersink_get_ch_layout(sink, &ch_layout);
            if (ret < 0)
                goto fail;
        }

        /* prepare audio output */
        if ((ret = audio_open(is, &ch_layout, sample_rate, &is->audio_tgt)) < 0)
            goto fail;
        is->audio_hw_buf_size = ret;
        is->audio_src = is->audio_tgt;
        is->audio_buf_size  = 0;
        is->audio_buf_index = 0;

        /* init averaging filter */
        is->audio_diff_avg_coef  = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
        is->audio_diff_avg_count = 0;
        /* since we do not have a precise anough audio FIFO fullness,
           we correct audio sync only if larger than this threshold */
        is->audio_diff_threshold = (double)(is->audio_hw_buf_size) / is->audio_tgt.bytes_per_sec;

        is->audio_stream = stream_index;
        is->audio_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->auddec, avctx, &is->audioq, is->continue_read_thread)) < 0)
            goto fail;
        if (is->ic->iformat->flags & AVFMT_NOTIMESTAMPS) {
            is->auddec.start_pts = is->audio_st->start_time;
            is->auddec.start_pts_tb = is->audio_st->time_base;
        }
        if ((ret = decoder_start(&is->auddec, audio_thread, "audio_decoder", is)) < 0)
            goto out;
        SDL_PauseAudioDevice(audio_dev, 0);
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_stream = stream_index;
        is->video_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->viddec, avctx, &is->videoq, is->continue_read_thread)) < 0)
            goto fail;
        if ((ret = decoder_start(&is->viddec, video_thread, "video_decoder", is)) < 0)
            goto out;
        is->queue_attachments_req = 1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_stream = stream_index;
        is->subtitle_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->subdec, avctx, &is->subtitleq, is->continue_read_thread)) < 0)
            goto fail;
        if ((ret = decoder_start(&is->subdec, subtitle_thread, "subtitle_decoder", is)) < 0)
            goto out;
        break;
    default:
        break;
    }
    goto out;

fail:
    avcodec_free_context(&avctx);
out:
    av_channel_layout_uninit(&ch_layout);
    av_dict_free(&opts);

    return ret;
}

static int decode_interrupt_cb(void *ctx)
{
    VideoState *is = ctx;
    return is->abort_request;
}

static int stream_has_enough_packets(AVStream *st, int stream_id, PacketQueue *queue) {
    return stream_id < 0 ||
           queue->abort_request ||
           (st->disposition & AV_DISPOSITION_ATTACHED_PIC) ||
           queue->nb_packets > MIN_FRAMES && (!queue->duration || av_q2d(st->time_base) * queue->duration > 1.0);
}

static int is_realtime(AVFormatContext *s)
{
    if(   !strcmp(s->iformat->name, "rtp")
       || !strcmp(s->iformat->name, "rtsp")
       || !strcmp(s->iformat->name, "sdp")
    )
        return 1;

    if(s->pb && (   !strncmp(s->url, "rtp:", 4)
                 || !strncmp(s->url, "udp:", 4)
                )
    )
        return 1;
    return 0;
}

/* this thread gets the stream from the disk or the network */
static int read_thread(void *arg)
{
    VideoState *is = arg;
    AVFormatContext *ic = NULL;
    int err, i, ret;
    int st_index[AVMEDIA_TYPE_NB];
    AVPacket *pkt = NULL;
    int64_t stream_start_time;
    int pkt_in_play_range = 0;
    const AVDictionaryEntry *t;
    SDL_mutex *wait_mutex = SDL_CreateMutex();
    int scan_all_pmts_set = 0;
    int64_t pkt_ts;

    if (!wait_mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    memset(st_index, -1, sizeof(st_index));
    is->eof = 0;

    pkt = av_packet_alloc();
    if (!pkt) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate packet.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ic = avformat_alloc_context();
    if (!ic) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate context.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ic->interrupt_callback.callback = decode_interrupt_cb;
    ic->interrupt_callback.opaque = is;
    if (!av_dict_get(format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE)) {
        av_dict_set(&format_opts, "scan_all_pmts", "1", AV_DICT_DONT_OVERWRITE);
        scan_all_pmts_set = 1;
    }
    err = avformat_open_input(&ic, is->filename, is->iformat, &format_opts);
    if (err < 0) {
        print_error(is->filename, err);
        ret = -1;
        goto fail;
    }
    if (scan_all_pmts_set)
        av_dict_set(&format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE);

    if ((t = av_dict_get(format_opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        ret = AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }
    is->ic = ic;

    if (genpts)
        ic->flags |= AVFMT_FLAG_GENPTS;

    if (find_stream_info) {
        AVDictionary **opts;
        int orig_nb_streams = ic->nb_streams;

        err = setup_find_stream_info_opts(ic, codec_opts, &opts);
        if (err < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Error setting up avformat_find_stream_info() options\n");
            ret = err;
            goto fail;
        }

        err = avformat_find_stream_info(ic, opts);

        for (i = 0; i < orig_nb_streams; i++)
            av_dict_free(&opts[i]);
        av_freep(&opts);

        if (err < 0) {
            av_log(NULL, AV_LOG_WARNING,
                   "%s: could not find codec parameters\n", is->filename);
            ret = -1;
            goto fail;
        }
    }

    if (ic->pb)
        ic->pb->eof_reached = 0; // FIXME hack, ffplay maybe should not use avio_feof() to test for the end

    if (seek_by_bytes < 0)
        seek_by_bytes = !(ic->iformat->flags & AVFMT_NO_BYTE_SEEK) &&
                        !!(ic->iformat->flags & AVFMT_TS_DISCONT) &&
                        strcmp("ogg", ic->iformat->name);

    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0;

    if (!window_title && (t = av_dict_get(ic->metadata, "title", NULL, 0)))
        window_title = av_asprintf("%s - %s", t->value, input_filename);

    /* if seeking requested, we execute it */
    if (start_time != AV_NOPTS_VALUE) {
        int64_t timestamp;

        timestamp = start_time;
        /* add the stream start time */
        if (ic->start_time != AV_NOPTS_VALUE)
            timestamp += ic->start_time;
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
                    is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }

    is->realtime = is_realtime(ic);

    if (show_status)
        av_dump_format(ic, 0, is->filename, 0);

    for (i = 0; i < ic->nb_streams; i++) {
        AVStream *st = ic->streams[i];
        enum AVMediaType type = st->codecpar->codec_type;
        st->discard = AVDISCARD_ALL;
        if (type >= 0 && wanted_stream_spec[type] && st_index[type] == -1)
            if (avformat_match_stream_specifier(ic, st, wanted_stream_spec[type]) > 0)
                st_index[type] = i;
    }
    for (i = 0; i < AVMEDIA_TYPE_NB; i++) {
        if (wanted_stream_spec[i] && st_index[i] == -1) {
            av_log(NULL, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream\n", wanted_stream_spec[i], av_get_media_type_string(i));
            st_index[i] = INT_MAX;
        }
    }

    if (!video_disable)
        st_index[AVMEDIA_TYPE_VIDEO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
                                st_index[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);
    if (!audio_disable)
        st_index[AVMEDIA_TYPE_AUDIO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
                                st_index[AVMEDIA_TYPE_AUDIO],
                                st_index[AVMEDIA_TYPE_VIDEO],
                                NULL, 0);
    if (!video_disable && !subtitle_disable)
        st_index[AVMEDIA_TYPE_SUBTITLE] =
            av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
                                st_index[AVMEDIA_TYPE_SUBTITLE],
                                (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ?
                                 st_index[AVMEDIA_TYPE_AUDIO] :
                                 st_index[AVMEDIA_TYPE_VIDEO]),
                                NULL, 0);

    is->show_mode = show_mode;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]];
        AVCodecParameters *codecpar = st->codecpar;
        AVRational sar = av_guess_sample_aspect_ratio(ic, st, NULL);
        if (codecpar->width)
            set_default_window_size(codecpar->width, codecpar->height, sar);
    }

    /* open the streams */
    if (st_index[AVMEDIA_TYPE_AUDIO] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }

    ret = -1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }
    if (is->show_mode == SHOW_MODE_NONE)
        is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;

    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }

    if (is->video_stream < 0 && is->audio_stream < 0) {
        av_log(NULL, AV_LOG_FATAL, "Failed to open file '%s' or configure filtergraph\n",
               is->filename);
        ret = -1;
        goto fail;
    }

    if (infinite_buffer < 0 && is->realtime)
        infinite_buffer = 1;




    //daipozhi modified
    deb_stream_err=0;
    deb_stream_open=0;
    deb_eo_stream=0;




    for (;;) {
        if (is->abort_request)
            break;
        if (is->paused != is->last_paused) {
            is->last_paused = is->paused;
            if (is->paused)
                is->read_pause_return = av_read_pause(ic);
            else
                av_read_play(ic);
        }
#if CONFIG_RTSP_DEMUXER || CONFIG_MMSH_PROTOCOL
        if (is->paused &&
                (!strcmp(ic->iformat->name, "rtsp") ||
                 (ic->pb && !strncmp(input_filename, "mmsh:", 5)))) {
            /* wait 10 ms to avoid trying to get another packet */
            /* XXX: horrible */
            SDL_Delay(10);
            continue;
        }
#endif
        if (is->seek_req) {
            int64_t seek_target = is->seek_pos;
            int64_t seek_min    = is->seek_rel > 0 ? seek_target - is->seek_rel + 2: INT64_MIN;
            int64_t seek_max    = is->seek_rel < 0 ? seek_target - is->seek_rel - 2: INT64_MAX;
// FIXME the +-2 is due to rounding being not done in the correct direction in generation
//      of the seek_pos/seek_rel variables

            ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max, is->seek_flags);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR,
                       "%s: error while seeking\n", is->ic->url);
            } else {
                if (is->audio_stream >= 0)
                    packet_queue_flush(&is->audioq);
                if (is->subtitle_stream >= 0)
                    packet_queue_flush(&is->subtitleq);
                if (is->video_stream >= 0)
                    packet_queue_flush(&is->videoq);
                if (is->seek_flags & AVSEEK_FLAG_BYTE) {
                   set_clock(&is->extclk, NAN, 0);
                } else {
                   set_clock(&is->extclk, seek_target / (double)AV_TIME_BASE, 0);
                }
            }
            is->seek_req = 0;
            is->queue_attachments_req = 1;
            is->eof = 0;
            if (is->paused)
                step_to_next_frame(is);
        }
        if (is->queue_attachments_req) {
            if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC) {
                if ((ret = av_packet_ref(pkt, &is->video_st->attached_pic)) < 0)
                    goto fail;
                packet_queue_put(&is->videoq, pkt);
                packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
            }
            is->queue_attachments_req = 0;
        }

        /* if the queue are full, no need to read more */
        if (infinite_buffer<1 &&
              (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE
            || (stream_has_enough_packets(is->audio_st, is->audio_stream, &is->audioq) &&
                stream_has_enough_packets(is->video_st, is->video_stream, &is->videoq) &&
                stream_has_enough_packets(is->subtitle_st, is->subtitle_stream, &is->subtitleq)))) {
            /* wait 10 ms */
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        }
        if (!is->paused &&
            (!is->audio_st || (is->auddec.finished == is->audioq.serial && frame_queue_nb_remaining(&is->sampq) == 0)) &&
            (!is->video_st || (is->viddec.finished == is->videoq.serial && frame_queue_nb_remaining(&is->pictq) == 0))) {
            if (loop != 1 && (!loop || --loop)) {
                stream_seek(is, start_time != AV_NOPTS_VALUE ? start_time : 0, 0, 0);
            } else if (autoexit) {
                ret = AVERROR_EOF;
                goto fail;
            }
        }
        ret = av_read_frame(ic, pkt);
        if (ret < 0) {
            if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) {
                if (is->video_stream >= 0)
                    packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
                if (is->audio_stream >= 0)
                    packet_queue_put_nullpacket(&is->audioq, pkt, is->audio_stream);
                if (is->subtitle_stream >= 0)
                    packet_queue_put_nullpacket(&is->subtitleq, pkt, is->subtitle_stream);
                is->eof = 1;
            }
            if (ic->pb && ic->pb->error) {
                if (autoexit)
                    goto fail;
                else
                    break;
            }
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        } else {
            is->eof = 0;
        }
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        stream_start_time = ic->streams[pkt->stream_index]->start_time;
        pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
        pkt_in_play_range = duration == AV_NOPTS_VALUE ||
                (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0)) *
                av_q2d(ic->streams[pkt->stream_index]->time_base) -
                (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000
                <= ((double)duration / 1000000);
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            packet_queue_put(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream && pkt_in_play_range
                   && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) {
            packet_queue_put(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            packet_queue_put(&is->subtitleq, pkt);
        } else {
            av_packet_unref(pkt);
        }




	deb_thr_r=1;  //daipozhi modified 




    }

    ret = 0;
 fail:




    //daipozhi modified
    deb_eo_stream=1;




    if (ic && !is->ic)
        avformat_close_input(&ic);

    av_packet_free(&pkt);
    if (ret != 0) {
        SDL_Event event;

        event.type = FF_QUIT_EVENT;
        event.user.data1 = is;
        SDL_PushEvent(&event);
    }
    SDL_DestroyMutex(wait_mutex);
    return 0;
}




//daipozhi modified
static VideoState *stream_open(const char *filename,
                               const AVInputFormat *iformat,int step)
{




    //VideoState *is;




	if (step==1)//daipozhi modified 
	{  




    stream_open_is = av_mallocz(sizeof(VideoState));
    if (!stream_open_is)
        return NULL;




          deb_m_vol=SDL_MIX_MAXVOLUME;
          g_set_scrollbar_value_now(1,deb_m_vol);
        }
	if (step==2)//daipozhi modified 
	{




    stream_open_is->last_video_stream = stream_open_is->video_stream = -1;
    stream_open_is->last_audio_stream = stream_open_is->audio_stream = -1;
    stream_open_is->last_subtitle_stream = stream_open_is->subtitle_stream = -1;
    stream_open_is->filename = av_strdup(filename);
    if (!stream_open_is->filename)
        goto fail;
    stream_open_is->iformat = iformat;
    stream_open_is->ytop    = 0;
    stream_open_is->xleft   = 0;

    /* start video display */
    if (frame_queue_init(&stream_open_is->pictq, &stream_open_is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0)
        goto fail;
    if (frame_queue_init(&stream_open_is->subpq, &stream_open_is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0)
        goto fail;
    if (frame_queue_init(&stream_open_is->sampq, &stream_open_is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0)
        goto fail;

    if (packet_queue_init(&stream_open_is->videoq) < 0 ||
        packet_queue_init(&stream_open_is->audioq) < 0 ||
        packet_queue_init(&stream_open_is->subtitleq) < 0)
        goto fail;

    if (!(stream_open_is->continue_read_thread = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        goto fail;
    }

    init_clock(&stream_open_is->vidclk, &stream_open_is->videoq.serial);
    init_clock(&stream_open_is->audclk, &stream_open_is->audioq.serial);
    init_clock(&stream_open_is->extclk, &stream_open_is->extclk.serial);
    stream_open_is->audio_clock_serial = -1;
    if (startup_volume < 0)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d < 0, setting to 0\n", startup_volume);
    if (startup_volume > 100)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d > 100, setting to 100\n", startup_volume);
    startup_volume = av_clip(startup_volume, 0, 100);
    startup_volume = av_clip(SDL_MIX_MAXVOLUME * startup_volume / 100, 0, SDL_MIX_MAXVOLUME);
    
    
    
    
    //daipozhi modified
    stream_open_is->audio_volume = deb_m_vol /*startup_volume*/;
    
    
    
    
    stream_open_is->muted = 0;
    stream_open_is->av_sync_type = av_sync_type;
    stream_open_is->read_tid     = SDL_CreateThread(read_thread, "read_thread", stream_open_is);
    if (!stream_open_is->read_tid) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateThread(): %s\n", SDL_GetError());
fail:
        stream_close(stream_open_is);
        return NULL;
    }




            // daipozhi modified
	    deb_st_play=1;
	}




    return stream_open_is;
}

static void stream_cycle_channel(VideoState *is, int codec_type)
{
    AVFormatContext *ic = is->ic;
    int start_index, stream_index;
    int old_index;
    AVStream *st;
    AVProgram *p = NULL;
    int nb_streams = is->ic->nb_streams;

    if (codec_type == AVMEDIA_TYPE_VIDEO) {
        start_index = is->last_video_stream;
        old_index = is->video_stream;
    } else if (codec_type == AVMEDIA_TYPE_AUDIO) {
        start_index = is->last_audio_stream;
        old_index = is->audio_stream;
    } else {
        start_index = is->last_subtitle_stream;
        old_index = is->subtitle_stream;
    }
    stream_index = start_index;

    if (codec_type != AVMEDIA_TYPE_VIDEO && is->video_stream != -1) {
        p = av_find_program_from_stream(ic, NULL, is->video_stream);
        if (p) {
            nb_streams = p->nb_stream_indexes;
            for (start_index = 0; start_index < nb_streams; start_index++)
                if (p->stream_index[start_index] == stream_index)
                    break;
            if (start_index == nb_streams)
                start_index = -1;
            stream_index = start_index;
        }
    }

    for (;;) {
        if (++stream_index >= nb_streams)
        {
            if (codec_type == AVMEDIA_TYPE_SUBTITLE)
            {
                stream_index = -1;
                is->last_subtitle_stream = -1;
                goto the_end;
            }
            if (start_index == -1)
                return;
            stream_index = 0;
        }
        if (stream_index == start_index)
            return;
        st = is->ic->streams[p ? p->stream_index[stream_index] : stream_index];
        if (st->codecpar->codec_type == codec_type) {
            /* check that parameters are OK */
            switch (codec_type) {
            case AVMEDIA_TYPE_AUDIO:
                if (st->codecpar->sample_rate != 0 &&
                    st->codecpar->ch_layout.nb_channels != 0)
                    goto the_end;
                break;
            case AVMEDIA_TYPE_VIDEO:
            case AVMEDIA_TYPE_SUBTITLE:
                goto the_end;
            default:
                break;
            }
        }
    }
 the_end:
    if (p && stream_index != -1)
        stream_index = p->stream_index[stream_index];
    av_log(NULL, AV_LOG_INFO, "Switch %s stream from #%d to #%d\n",
           av_get_media_type_string(codec_type),
           old_index,
           stream_index);

    stream_component_close(is, old_index);
    stream_component_open(is, stream_index);
}


static void toggle_full_screen(VideoState *is)
{
    is_full_screen = !is_full_screen;
    SDL_SetWindowFullscreen(window, is_full_screen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
}

static void toggle_audio_display(VideoState *is)
{
    int next = is->show_mode;
    do {
        next = (next + 1) % SHOW_MODE_NB;
    } while (next != is->show_mode && (next == SHOW_MODE_VIDEO && !is->video_st || next != SHOW_MODE_VIDEO && !is->audio_st));
    if (is->show_mode != next) {
        is->force_refresh = 1;
        is->show_mode = next;
    }
}

static void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {




    // daipozhi for sound river
    struct timeval  tv;
    



    double remaining_time = 0.0;
    SDL_PumpEvents();
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {




        //daipozhi modified
        deb_m_ref=0;
        deb_m_ref_v=0;
        deb_st_video=0;




	//daipozhi modified
        //if (!cursor_hidden && av_gettime_relative() - cursor_last_shown > CURSOR_HIDE_DELAY) {
        //    SDL_ShowCursor(0);
        //    cursor_hidden = 1;
        //}
        
        
        
        
        if (remaining_time > 0.0)
            av_usleep((int64_t)(remaining_time * 1000000.0));
        remaining_time = REFRESH_RATE;
        
        
        
        
                // daipozhi modified for sound river
		if ((deb_sr_show_start==1)&&(deb_sr_time_set==0)&&(deb_thr_a))
		{
			gettimeofday(&tv,NULL/*,&tz*/);
			deb_sr_time1=tv.tv_sec;
			deb_sr_time2=tv.tv_usec;
			deb_sr_time3=deb_sr_time1*1000000+deb_sr_time2;
			deb_sr_time_set=1;
		}

		//   daipozhi modified 
		if (deb_st_play==1)
		{
			if (deb_frame_num<50)  // at opened stream's first 0.5s to do nothing, to avoid error
			{
				deb_frame_num++;
			}
			else
			{
				if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
				{
				  deb_sr_river_show(is);  // daipozhi for sound river
				}

				if ((deb_thr_v)&&(deb_thr_a)&&(deb_thr_a2)&&(deb_thr_r))
				{
					if (is->video_st)
					{




        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh))
            video_refresh(is, &remaining_time);




					}
				}
				
				deb_seek_bar_cntr++;   //daipozhi modified
				
				if (deb_seek_bar_cntr>=50) //update seek bar in every 0.5s
				{
					deb_seek_bar_cntr=0;
					
					if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
					{
						if ((deb_thr_a)&&(deb_thr_a2)&&(deb_thr_r))  // audio without image
						{
							/*if ((deb_st_play==1)&&(is->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
							      video_image_display(is);
							else*/ if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
							  deb_sr_river_show_pause(is);
							else
							{
							  if (g_caret_show==0) g_caret_show=1;
							  else                 g_caret_show=0;

							  g_paint_dirview();//deb_disp_dir(is);
							}
							//its purpose is deb_disp_bar()
						}
					}
					else if (is->video_st && is->show_mode == SHOW_MODE_VIDEO)
					{
						if ((deb_thr_v)&&(deb_thr_a)&&(deb_thr_a2)&&(deb_thr_r))  //video or audio with picture
						{
							if ((deb_st_play==1)&&(is->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
							{
							    video_image_display(is);
							    //vid=1;
							}
							else if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
							    deb_sr_river_show_pause(is);
							else
							{
							    if (g_caret_show==0) g_caret_show=1;
							    else                 g_caret_show=0;

							    g_paint_dirview();//deb_disp_dir(is);
							}
							//its purpose is deb_disp_bar()
						}
					}
				}
			}
		}
		else
		{                          //no playing
		    deb_seek_bar_cntr++;   //daipozhi modified
				
		    if (deb_seek_bar_cntr>=50) //update seek bar in every 0.5s
		    {
			deb_seek_bar_cntr=0;
					
		        if (g_caret_show==0) g_caret_show=1;
		        else                 g_caret_show=0;

			g_paint_dirview();//deb_disp_dir(is);

		        g_paint_scrollbar(0);//deb_disp_bar(is); // in sdl2 ,you need prepare full window data
		        g_paint_button(0,g_mouse_down);
		        g_paint_label(0);
			g_paint_checkbox(0,g_mouse_down);
			g_paint_radiobutton(0,g_mouse_down);
			g_paint_lineedit(0);
		        if (deb_tx_locked==1)
		        {
		          SDL_UnlockTexture(is->vis_texture);
		          SDL_RenderCopy(renderer, is->vis_texture, NULL, NULL);
		          deb_tx_locked=0;
		        }
		  
		        SDL_RenderPresent(renderer);
		        
			deb_m_ref=0;
			deb_m_ref_v=0;
		    }

		    SDL_PumpEvents();

		    continue;
		}					

		if (deb_m_ref==1)
		{
		  g_set_scrollbar_value_max(0,is->ic->duration/1000000);
		  g_set_scrollbar_value_now(0,get_master_clock(is));
		  g_set_scrollbar_value_max(1,SDL_MIX_MAXVOLUME);
		  
		  deb_disp_bar(is);
		  
		  g_paint_scrollbar(deb_st_video);//deb_disp_bar(is); // in sdl2 ,you need prepare full screen data
		  g_paint_button(deb_st_video,g_mouse_down);
		  g_paint_label(deb_st_video);
		  g_paint_checkbox(deb_st_video,g_mouse_down);
		  g_paint_radiobutton(deb_st_video,g_mouse_down);
		  g_paint_lineedit(deb_st_video);
		  if (deb_tx_locked==1)
		  {
		    SDL_UnlockTexture(is->vis_texture);
		    SDL_RenderCopy(renderer, is->vis_texture, NULL, NULL);
		    deb_tx_locked=0;
		  }
		  SDL_RenderPresent(renderer);
		}
		else
		{
		  if (deb_tx_locked==1)
		  {
		    SDL_UnlockTexture(is->vis_texture);
		    deb_tx_locked=0;
		  }
		}

		deb_m_ref=0;
		deb_m_ref_v=0;




        SDL_PumpEvents();
    }
}

static void seek_chapter(VideoState *is, int incr)
{
    int64_t pos = get_master_clock(is) * AV_TIME_BASE;
    int i;

    if (!is->ic->nb_chapters)
        return;

    /* find the current chapter */
    for (i = 0; i < is->ic->nb_chapters; i++) {
        AVChapter *ch = is->ic->chapters[i];
        if (av_compare_ts(pos, AV_TIME_BASE_Q, ch->start, ch->time_base) < 0) {
            i--;
            break;
        }
    }

    i += incr;
    i = FFMAX(i, 0);
    if (i >= is->ic->nb_chapters)
        return;

    av_log(NULL, AV_LOG_VERBOSE, "Seeking to chapter %d.\n", i);
    stream_seek(is, av_rescale_q(is->ic->chapters[i]->start, is->ic->chapters[i]->time_base,
                                 AV_TIME_BASE_Q), 0, 0);
}

/* handle an event sent by the GUI */
static void event_loop(VideoState *cur_stream)
{
    SDL_Event event;
    
    
    
    
    //daipozhi modified
    double /*incr, pos,*/ frac;




    //daipozhi modified
    int  xx,xx2,yy,yy2,yy3,/*yy4,*/n2,n3;  
    char sc1;               
    int  s_dir_opened;
    long long int  i;
    int  vid;
    char str1[FN_SIZE];
    char str2[FN_SIZE];
    char str3[FN_SIZE];
    int  cntr=0;
    int  j,k,l;
    
    int  s_shift;
    int  s_caps;
    SDL_Keymod mod;




    for (;;) {
        double x;
        refresh_loop_wait_event(cur_stream, &event);
        switch (event.type) {
        case SDL_KEYDOWN:
            //if (exit_on_keydown || event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q) {
            //    do_exit(cur_stream);
            //    break;
            //}
            // If we don't yet have a window, skip all key events, because read_thread might still be initializing...
            if (!cur_stream->width)
                continue;




	    //daipozhi modified
            s_shift=0;
            s_caps=0;
            
            mod=event.key.keysym.mod;
            
            if ( mod & KMOD_CAPS )  s_caps=1;
            if ( mod & KMOD_SHIFT ) s_shift=1;




            switch (event.key.keysym.sym) {
            /*
            case SDLK_f:
                //toggle_full_screen(cur_stream);
                //cur_stream->force_refresh = 1;

                u_test();
                u_test_ptr++;

                break;
            case SDLK_p:
            case SDLK_SPACE:
                //toggle_pause(cur_stream);

		deb_sr_river_f_cons_test(cur_stream,deb_sr_river_f_test);
		deb_sr_river_f_test++;

                break;
            case SDLK_m:
                //toggle_mute(cur_stream);
#if DPZ_DEBUG2
		deb_sr_fft_deb_chn++;
		if (deb_sr_fft_deb_chn>=3) deb_sr_fft_deb_chn=0;
#endif
                break;
            case SDLK_KP_MULTIPLY:
            case SDLK_0:
                //update_volume(cur_stream, 1, SDL_VOLUME_STEP);
                break;
            case SDLK_KP_DIVIDE:
            case SDLK_9:
                //update_volume(cur_stream, -1, SDL_VOLUME_STEP);
                break;
            case SDLK_s: // S: Step to next frame
                //step_to_next_frame(cur_stream);
                break;
            case SDLK_a:
                //stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                break;
            case SDLK_v:
                //stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                break;
            case SDLK_c:
                //stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                //stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                //stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_t:
                //stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_w:
                //if (cur_stream->show_mode == SHOW_MODE_VIDEO && cur_stream->vfilter_idx < nb_vfilters - 1) {
                //    if (++cur_stream->vfilter_idx >= nb_vfilters)
                //        cur_stream->vfilter_idx = 0;
                //} else {
                //    cur_stream->vfilter_idx = 0;
                //    toggle_audio_display(cur_stream);
                //}
                break;
            case SDLK_PAGEUP:
                //if (cur_stream->ic->nb_chapters <= 1) {
                //    incr = 600.0;
                //    goto do_seek;
                //}
                //seek_chapter(cur_stream, 1);
                break;
            case SDLK_PAGEDOWN:
                //if (cur_stream->ic->nb_chapters <= 1) {
                //    incr = -600.0;
                //    goto do_seek;
                //}
                //seek_chapter(cur_stream, -1);
                break;
            case SDLK_LEFT:
                incr = seek_interval ? -seek_interval : -10.0;
                goto do_seek;
            case SDLK_RIGHT:
                incr = seek_interval ? seek_interval : 10.0;
                goto do_seek;
            case SDLK_UP:
                incr = 60.0;
                goto do_seek;
            case SDLK_DOWN:
                incr = -60.0;
            do_seek:




		    //daipozhi modified
		    if (deb_eo_stream==1) break;




                    if (seek_by_bytes) {
                        pos = -1;
                        if (pos < 0 && cur_stream->video_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->pictq);
                        if (pos < 0 && cur_stream->audio_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->sampq);
                        if (pos < 0)
                            pos = avio_tell(cur_stream->ic->pb);
                        if (cur_stream->ic->bit_rate)
                            incr *= cur_stream->ic->bit_rate / 8.0;
                        else
                            incr *= 180000.0;
                        pos += incr;
                        stream_seek(cur_stream, pos, incr, 1);
                    } else {
                        pos = get_master_clock(cur_stream);
                        if (isnan(pos))
                            pos = (double)cur_stream->seek_pos / AV_TIME_BASE;
                        pos += incr;
                        if (cur_stream->ic->start_time != AV_NOPTS_VALUE && pos < cur_stream->ic->start_time / (double)AV_TIME_BASE)
                            pos = cur_stream->ic->start_time / (double)AV_TIME_BASE;
                        stream_seek(cur_stream, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
                    }
                break;
            default:
                break;
            */
            
            
                
                              
	    case SDLK_SPACE:
		smg_key=' ';
		break;
	    case SDLK_BACKQUOTE:
                if (s_shift==1) smg_key='~';
		else smg_key='`';
		break;
            case SDLK_0:
                if (s_shift==1) smg_key=')';
                else smg_key='0';
                break;
            case SDLK_1:
                if (s_shift==1) smg_key='!';
                else smg_key='1';
                break;
            case SDLK_2:
                if (s_shift==1) smg_key='@';
                else smg_key='2';
                break;
            case SDLK_3:
                if (s_shift==1) smg_key='#';
                else smg_key='3';
                break;
            case SDLK_4:
                if (s_shift==1) smg_key='$';
                else smg_key='4';
                break;
            case SDLK_5:
                if (s_shift==1) smg_key='%';
                else smg_key='5';
                break;
            case SDLK_6:
                if (s_shift==1) smg_key='^';
                else smg_key='6';
                break;
            case SDLK_7:
                if (s_shift==1) smg_key='&';
                else smg_key='7';
                break;
            case SDLK_8:
                if (s_shift==1) smg_key='*';
                else smg_key='8';
                break;
            case SDLK_9:
                if (s_shift==1) smg_key='(';
                else smg_key='9';
                break;
            case SDLK_MINUS:
                if (s_shift==1) smg_key='_';
                else smg_key='-';
                break;
            case SDLK_EQUALS:
                if (s_shift==1) smg_key='+';
                else smg_key='=';
                break;


            case SDLK_LEFTBRACKET:
                if (s_shift==1) smg_key='{';
                else smg_key='[';
                break;
            case SDLK_RIGHTBRACKET:
                if (s_shift==1) smg_key='}';
                else smg_key=']';
                break;
            case SDLK_BACKSLASH:
                if (s_shift==1) smg_key='|';
                else smg_key='\\';
                break;


	    case SDLK_SEMICOLON:
                if (s_shift==1) smg_key=':';
		else smg_key=';';
		break;
            case SDLK_QUOTE:
                if (s_shift==1) smg_key='"';
                else smg_key=39;
                break;


            case SDLK_COMMA:
                if (s_shift==1) smg_key='<';
                else smg_key=',';
                break;
            case SDLK_PERIOD:
                if (s_shift==1) smg_key='>';
                else smg_key='.';
                break;
            case SDLK_SLASH:
                if (s_shift==1) smg_key='?';
                else smg_key='/';
                break;


            case SDLK_a:
                if ((s_caps==1)||(s_shift==1)) smg_key='A';
                else smg_key='a';
                break;
            case SDLK_b:
                if ((s_caps==1)||(s_shift==1)) smg_key='B';
                else smg_key='b';
                break;
            case SDLK_c:
                if ((s_caps==1)||(s_shift==1)) smg_key='C';
                else smg_key='c';
                break;
            case SDLK_d:
                if ((s_caps==1)||(s_shift==1)) smg_key='D';
                else smg_key='d';
                break;
            case SDLK_e:
                if ((s_caps==1)||(s_shift==1)) smg_key='E';
                else smg_key='e';
                break;
            case SDLK_f:
                if ((s_caps==1)||(s_shift==1)) smg_key='F';
                else smg_key='f';
                break;
            case SDLK_g:
                if ((s_caps==1)||(s_shift==1)) smg_key='G';
                else smg_key='g';
                break;
            case SDLK_h:
                if ((s_caps==1)||(s_shift==1)) smg_key='H';
                else smg_key='h';
                break;
            case SDLK_i:
                if ((s_caps==1)||(s_shift==1)) smg_key='I';
                else smg_key='i';
                break;
            case SDLK_j:
                if ((s_caps==1)||(s_shift==1)) smg_key='J';
                else smg_key='j';
                break;
            case SDLK_k:
                if ((s_caps==1)||(s_shift==1)) smg_key='K';
                else smg_key='k';
                break;
            case SDLK_l:
                if ((s_caps==1)||(s_shift==1)) smg_key='L';
                else smg_key='l';
                break;
            case SDLK_m:
                if ((s_caps==1)||(s_shift==1)) smg_key='M';
                else smg_key='m';
                break;
            case SDLK_n:
                if ((s_caps==1)||(s_shift==1)) smg_key='N';
                else smg_key='n';
                break;
            case SDLK_o:
                if ((s_caps==1)||(s_shift==1)) smg_key='O';
                else smg_key='o';
                break;
            case SDLK_p:
                if ((s_caps==1)||(s_shift==1)) smg_key='P';
                else smg_key='p';
                break;
            case SDLK_q:
                if ((s_caps==1)||(s_shift==1)) smg_key='Q';
                else smg_key='q';
                break;
            case SDLK_r:
                if ((s_caps==1)||(s_shift==1)) smg_key='R';
                else smg_key='r';
                break;
            case SDLK_s:
                if ((s_caps==1)||(s_shift==1)) smg_key='S';
                else smg_key='s';
                break;
            case SDLK_t:
                if ((s_caps==1)||(s_shift==1)) smg_key='T';
                else smg_key='t';
                break;
            case SDLK_u:
                if ((s_caps==1)||(s_shift==1)) smg_key='U';
                else smg_key='u';
                break;
            case SDLK_v:
                if ((s_caps==1)||(s_shift==1)) smg_key='V';
                else smg_key='v';
                break;
            case SDLK_w:
                if ((s_caps==1)||(s_shift==1)) smg_key='W';
                else smg_key='w';
                break;
            case SDLK_x:
                if ((s_caps==1)||(s_shift==1)) smg_key='X';
                else smg_key='x';
                break;
            case SDLK_y:
                if ((s_caps==1)||(s_shift==1)) smg_key='Y';
                else smg_key='y';
                break;
            case SDLK_z:
                if ((s_caps==1)||(s_shift==1)) smg_key='Z';
                else smg_key='z';
                break;


            case SDLK_LEFT:
                smg_key=SMG_KEY_LEFT;
                break;
            case SDLK_RIGHT:
                smg_key=SMG_KEY_RIGHT;
                break;
            case SDLK_UP:
                smg_key=SMG_KEY_UP;
                break;
            case SDLK_DOWN:
                smg_key=SMG_KEY_DOWN;
                break;
            case SDLK_BACKSPACE:
                smg_key=SMG_KEY_BACKSP;
                break;
            case SDLK_DELETE:
                smg_key=SMG_KEY_DEL;
                break;
            case SDLK_RETURN:
            case SDLK_RETURN2:
                smg_key=SMG_KEY_RET;
                break;
            default:
                smg_key=0;
                break;
            } 
            
            if (g_focus_ptr1==4)
            {
               if (g_lineedit_enable[g_focus_ptr2]!=1) break;
               
		smg_get_read(0,0,0,"",
                     str2,200/7,g_lineedit_type[g_focus_ptr2],
		     'r',300-1,0,
		     0,0,0,
		     g_focus_ptr2);
		     
		deb_disp_bar(cur_stream);
		g_paint_dirview();//deb_disp_dir(cur_stream);
		g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		g_paint_button(0,0);
		g_paint_label(0);
	        g_paint_lineedit(0);
		g_paint_checkbox(0,0);
		g_paint_radiobutton(0,0);
		if (deb_tx_locked==1)
		{
		  SDL_UnlockTexture(cur_stream->vis_texture);
		  SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		  deb_tx_locked=0;
		}
		SDL_RenderPresent(renderer);
		deb_m_ref=0;		     
            }
            
            
            
            
            break;
            
        case SDL_MOUSEBUTTONUP:
        
		g_dirview_mouse_down=0;
		g_dirview_mouse_down_x=0;
		g_dirview_mouse_down_y=0;
		
		g_mouse_down=0;
		
		     deb_m_ref=0;
		     deb_m_ref_v=0;

		     if ((deb_st_play==1)&&(cur_stream->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
		     {
		       video_image_display(cur_stream);
		       vid=1;
		     }
		     else if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		       deb_sr_river_show_pause(cur_stream);
		     else
		     {
		       g_paint_dirview();//deb_disp_dir(cur_stream);
		     }

		     if (deb_m_ref==1)
		     {
		       deb_disp_bar(cur_stream);
		       g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
		       g_paint_button(vid,0);
		       g_paint_label(vid);
		       g_paint_checkbox(vid,0);
		       g_paint_radiobutton(vid,0);
		       g_paint_lineedit(vid);
		       if (deb_tx_locked==1)
		       {
		         SDL_UnlockTexture(cur_stream->vis_texture);
		         SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		         deb_tx_locked=0;
		       }
		       SDL_RenderPresent(renderer);
		       deb_m_ref=0;
		       deb_m_ref_v=0;
		     }
		
		break;
		
        case SDL_MOUSEBUTTONDOWN:
        
        
        

	    g_mouse_down=1;
	        
            vid=0;
            
            /*if (exit_on_mousedown) {
                do_exit(cur_stream);
                break;
            }
            if (event.button.button == SDL_BUTTON_LEFT) {
                static int64_t last_mouse_left_click = 0;
                if (av_gettime_relative() - last_mouse_left_click <= 500000) {
                    toggle_full_screen(cur_stream);
                    cur_stream->force_refresh = 1;
                    last_mouse_left_click = 0;
                } else {
                    last_mouse_left_click = av_gettime_relative();
                }
            }*/

            av_log(NULL, AV_LOG_INFO,"mouse button down,x=%d,y=%d,\n",event.button.x,event.button.y);
            
	    xx=event.button.x;
	    yy=event.button.y;

	    g_detect_click(xx,yy);
	    
	    if ((g_detect_ptr1==1)&&(g_detect_ptr2==0))  // in playlist
	    {
	      if ((deb_cover==1)&&(deb_cover_close==0))
	      {
		deb_cover_close=1;
		deb_disp_bar(cur_stream);
		g_paint_dirview();//deb_disp_dir(cur_stream);
		g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		g_paint_button(0,0);
		g_paint_label(0);
		g_paint_checkbox(0,0);
		g_paint_radiobutton(0,0);
	        g_paint_lineedit(0);
		if (deb_tx_locked==1)
		{
		  SDL_UnlockTexture(cur_stream->vis_texture);
		  SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		  deb_tx_locked=0;
		}
		SDL_RenderPresent(renderer);
		deb_m_ref=0;
		break;
	      }
	      else
	      {
		if ((deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		{
		  deb_sr_show_nodisp=1;
		  deb_sr_show_start=0;
		  deb_disp_bar(cur_stream);
		  g_paint_dirview();//deb_disp_dir(cur_stream);
		  g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		  g_paint_button(0,0);
		  g_paint_label(0);
		  g_paint_checkbox(0,0);
		  g_paint_radiobutton(0,0);
		  g_paint_lineedit(0);
		  if (deb_tx_locked==1)
		  {
		    SDL_UnlockTexture(cur_stream->vis_texture);
		    SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		    deb_tx_locked=0;
		  }
		  SDL_RenderPresent(renderer);
		  deb_m_ref=0;
		  break;
		}
	      }
	    }

	    if ((g_detect_ptr1==1)&&(g_detect_ptr2==0)&&(g_detect_ptr3==1))    // in play list
	    {
		  n2=g_detect_ptr4;
		  
		  while ((deb_filenamebuff_subline[deb_filenamebuff_n+n2]==1)&&(deb_filenamebuff_n+n2>0))
		  {
		    n2--;
		  }

	          if (deb_filenamebuff_n+n2>=deb_filenamebuff_ptr) // more than filenamecnt 
		  {
                    break;
		  }

	          if (deb_filenamebuff_n+n2>=MAX_FILE_NUM) // more than filenamecnt 
		  {
                    break;
		  }

	          if (deb_filenamebuff_n+n2<0)
		  {
                    break;
		  }
		  
		  str3[0]=0;
		  strcpy(str3,deb_filenamebuff[deb_filenamebuff_n+n2]);
		  l=deb_filenamebuff_n+n2+1;
		  
		  while ((deb_filenamebuff_subline[l]==1)&&(l<deb_filenamebuff_ptr))
		  {
		    if (strlen(str3)+strlen(deb_filenamebuff[l])>=FN_SIZE) break;
		    
		    strcat(str3,deb_filenamebuff[l]);
		    l++;
		  }
		  
                  sc1=deb_filenamebuff_type[deb_filenamebuff_n+n2];
                  
                  if (sc1==2) break; // comment line
                  if (sc1==3) break; // empty line
		  if (sc1!=0)        // not dir not empty
		  {
		    if (deb_stream_open==1) break;

		    if (deb_no_support(str3,FN_SIZE)==1) break;

		    deb_get_path(deb_filenamebuff_n+n2);

		    if (deb_st_play==1)
		    {
		      //if (cur_stream->paused) toggle_pause(cur_stream);

		      stream_close(cur_stream);
                      //uninit_opts();
                      //deb_opts_stt=0;
		    }

		    //if (cur_stream->paused) toggle_pause(cur_stream);

		    deb_ini_is(cur_stream);
		    deb_video_open_again(cur_stream,0);

                    if (deb_opts_stt==1)
                    {
                      uninit_opts();
                      deb_opts_stt=0;
                    }

                    init_opts();
                    deb_opts_stt=1;
                   
		    deb_filenameplay=deb_filenamebuff_n+n2;

		    deb_cover=0;
		    deb_cover_close=0;
		    deb_frame_num=0;

		    deb_thr_v=0;
		    deb_thr_s=0;
		    deb_thr_a=0;
		    deb_thr_a2=0;
		    deb_thr_r=0;

		    deb_sr_time_set=0;
		    deb_sr_total_bytes=0;
		    deb_sr_show=0;
		    deb_sr_show_start=0;
		    deb_sr_show_nodisp=1;
		    deb_sr_show_init=0;
		    deb_sr_river_over=0;
		    deb_sr_sample_over=0;
		    deb_sr_sample_over2=0;
		    deb_sr_river_ptr=0;
		    deb_sr_river_last=0;
		    deb_sr_river_adj=0;

		    if (deb_str_has_null(deb_dir_buffer,FN_SIZE)!=1) break;
		    if ((strlen(deb_dir_buffer)>=FN_SIZE)||((int)strlen(deb_dir_buffer)<0)) break;
		    strcpy(deb_currentpath,deb_dir_buffer);

                    deb_stream_err=1;
                    deb_stream_open=1;
                    
		    stream_open(deb_dir_buffer, file_iformat,2);

		    //deb_disp_dir(cur_stream);
		    //deb_disp_bar(cur_stream);
		    //SDL_RenderPresent(renderer);
		    //deb_m_ref=0;
		    break;
		  }
		  else //dir
		  {
		    deb_get_path(deb_filenamebuff_n+n2);
		    
                    s_dir_opened=deb_dir_opened(deb_filenamebuff_n+n2);
                    
                    if (s_dir_opened==0)
                    {
		      i=chdir(deb_dir_buffer);
                      if (i!=0) break;

		      i=(long long int)getcwd(deb_currentpath,FN_SIZE);
                      if (i==0) break;

                      if (deb_cmp_dir(deb_dir_buffer,deb_currentpath)==0)
		      {
			deb_get_dir();
        	        deb_dir_add_after(deb_filenamebuff_n+n2);
			deb_disp_bar(cur_stream);
		        g_paint_dirview();//deb_disp_dir(cur_stream);
		        g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		        g_paint_button(0,0);
		        g_paint_label(0);
			g_paint_checkbox(0,0);
			g_paint_radiobutton(0,0);
		        g_paint_lineedit(0);
			if (deb_tx_locked==1)
			{
			  SDL_UnlockTexture(cur_stream->vis_texture);
			  SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		         deb_tx_locked=0;
			}
		        SDL_RenderPresent(renderer);
		        deb_m_ref=0;
		        
		        break;
		      }
                    }
                    else
                    {
                      deb_dir_remove_after(deb_filenamebuff_n+n2);
		      deb_disp_bar(cur_stream);
		      g_paint_dirview();//deb_disp_dir(cur_stream);
		      g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		      g_paint_button(0,0);
		      g_paint_label(0);
		      g_paint_checkbox(0,0);
		      g_paint_radiobutton(0,0);
		      g_paint_lineedit(0);
		      if (deb_tx_locked==1)
		      {
			  SDL_UnlockTexture(cur_stream->vis_texture);
			  SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		         deb_tx_locked=0;
		      }
		      SDL_RenderPresent(renderer);
		      deb_m_ref=0;
		      
		      break;
                    }
                  }
	    }

	    if ((g_detect_ptr1==1)&&(g_detect_ptr2==0))   //in border line
	    {
		    if (g_detect_ptr3==2)
		    {
		    //n4=cur_stream->height-deb_ch_h*2-deb_ch_h*2;
		    //n5=yy-deb_ch_h*1;
		    //if (n5<n4/2) // up page
		    //{
		      //n3=(cur_stream->height)/deb_ch_h-2-2;
		      n3=g_dirview_posi[g_detect_ptr2][3]/14;
		      
		      if (deb_filenamebuff_n>=n3) deb_filenamebuff_n=deb_filenamebuff_n-n3;
		      else deb_filenamebuff_n=0;

		      deb_disp_bar(cur_stream);
		      g_paint_dirview();//deb_disp_dir(cur_stream);
		      g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		      g_paint_button(0,0);
		      g_paint_label(0);
		      g_paint_checkbox(0,0);
		      g_paint_radiobutton(0,0);
		      g_paint_lineedit(0);
		      if (deb_tx_locked==1)
		      {
			SDL_UnlockTexture(cur_stream->vis_texture);
			SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		        deb_tx_locked=0;
		      }
		      SDL_RenderPresent(renderer);
		      deb_m_ref=0;
		      break;
		    }
		    else if (g_detect_ptr3==3)
		    {
		      g_dirview_mouse_down=1;
		      g_dirview_mouse_down_x=xx;
		      g_dirview_mouse_down_y=yy;
		      break;
		    }
		    else if (g_detect_ptr3==4) // down page
		    {
		      //n3=(cur_stream->height)/deb_ch_h-2-2;
		      n3=g_dirview_posi[g_detect_ptr2][3]/14;
		      
		      if (deb_filenamebuff_n+n3+n3<deb_filenamebuff_ptr) deb_filenamebuff_n=deb_filenamebuff_n+n3;
		      else
		      {
			deb_filenamebuff_n=deb_filenamebuff_ptr-n3;
			if (deb_filenamebuff_n<0) deb_filenamebuff_n=0;
		      }

		      deb_disp_bar(cur_stream);
		      g_paint_dirview();//deb_disp_dir(cur_stream);
		      g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		      g_paint_button(0,0);
		      g_paint_label(0);
		      g_paint_checkbox(0,0);
		      g_paint_radiobutton(0,0);
		      g_paint_lineedit(0);
		      if (deb_tx_locked==1)
		      {
			SDL_UnlockTexture(cur_stream->vis_texture);
			SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		        deb_tx_locked=0;
		      }
		      SDL_RenderPresent(renderer);
		      deb_m_ref=0;
		      break;
		    }
	    }

	    if ((g_detect_ptr1==2)&&(g_detect_ptr2==0)) // in scroll bar
	    {
	        if ((screen_w<640)||(screen_w>7680)) break;
	        if ((screen_h<480)||(screen_h>4320)) break;
	      
		if (deb_stream_open==1) break;

	        if (deb_st_play==1)
	        {

		  if (deb_eo_stream==1) break;

                  x = g_detect_ptr3;//event.button.x-5;


                  if (seek_by_bytes || cur_stream->ic->duration <= 0) {
                    uint64_t size =  avio_size(cur_stream->ic->pb);
                    stream_seek(cur_stream, size*x/(g_scrollbar_posi[0][2]), 0, 1);
                  } else {
                    int64_t ts;
                    int ns, hh, mm, ss;
                    int tns, thh, tmm, tss;
                    tns  = cur_stream->ic->duration / 1000000LL;
                    thh  = tns / 3600;
                    tmm  = (tns % 3600) / 60;
                    tss  = (tns % 60);
                    frac = x / (g_scrollbar_posi[0][2]);
                    ns   = frac * tns;
                    hh   = ns / 3600;
                    mm   = (ns % 3600) / 60;
                    ss   = (ns % 60);
                    av_log(NULL, AV_LOG_INFO,
                           "Seek to %2.0f%% (%2d:%02d:%02d) of total duration (%2d:%02d:%02d)       \n", frac*100,
                            hh, mm, ss, thh, tmm, tss);
                    ts = frac * cur_stream->ic->duration;
                    if (cur_stream->ic->start_time != AV_NOPTS_VALUE)
                        ts += cur_stream->ic->start_time;
                    stream_seek(cur_stream, ts, 0, 0);
                  }
    

		  deb_frame_num=0;
		  //deb_disp_bar(cur_stream);
		  break;
		}
	    }  
	      
	    if ((g_detect_ptr1==2)&&(g_detect_ptr2==1)) //volume bar
	    {
	        deb_m_vol=SDL_MIX_MAXVOLUME*(g_detect_ptr3)/180;
                g_set_scrollbar_value_now(1,deb_m_vol);
		if (deb_st_play==1) cur_stream->audio_volume=deb_m_vol;
		
		deb_m_ref=0;
		deb_m_ref_v=0;

		if ((deb_st_play==1)&&(cur_stream->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
		{
		    video_image_display(cur_stream);
		    vid=1;
		}
		else if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		    deb_sr_river_show_pause(cur_stream);
		else
		{
		    g_paint_dirview();//deb_disp_dir(cur_stream);
		}
		
		if (deb_m_ref==1)
		{
		  deb_disp_bar(cur_stream);
		  g_paint_scrollbar(vid);
		  g_paint_button(vid,0);
		  g_paint_label(vid);
		  g_paint_checkbox(vid,0);
		  g_paint_radiobutton(vid,0);
		  g_paint_lineedit(vid);
		  if (deb_tx_locked==1)
		  {
		    SDL_UnlockTexture(cur_stream->vis_texture);
		    SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		    deb_tx_locked=0;
		  }
		  SDL_RenderPresent(renderer);
		  deb_m_ref=0;
		  deb_m_ref_v=0;
		}
		
		break;
	    }

	    // daipozhi modified 
	    if ((g_detect_ptr1==3)&&(g_detect_ptr2==0))    //pause button
	    {
		if (deb_stream_open==1) break;

		if (deb_st_play==1)
		{
		  toggle_pause(cur_stream);
		  
		  strcpy(str1,g_button_text[0]);
		  
		  if (str1[1]=='a')
		  {
		    strcpy(str1,"Play");
		    g_set_button_text(0,str1,10);
		  }
		  else
		  {
		    strcpy(str1,"Pause");
		    g_set_button_text(0,str1,10);
		  }

		  deb_m_ref=0;
		  deb_m_ref_v=0;

		  if ((deb_st_play==1)&&(cur_stream->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
		  {
		    video_image_display(cur_stream);
		    vid=1;
		  }
		  else if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		    deb_sr_river_show_pause(cur_stream);
		  else
		  {
		    g_paint_dirview();//deb_disp_dir(cur_stream);
		  }

		  if (deb_m_ref==1)
		  {
		    deb_disp_bar(cur_stream);
		    g_paint_scrollbar(vid);
		    g_paint_button(vid,1);
		    g_paint_label(vid);
		    g_paint_checkbox(vid,0);
		    g_paint_radiobutton(vid,0);
		    g_paint_lineedit(vid);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;
		    deb_m_ref_v=0;
		  }

		  break;
		}
 	    }  
	      
	    if ((g_detect_ptr1==3)&&(g_detect_ptr2==1))   // dir,video,river switch button
	    {
		if (deb_stream_open==1) break;

		if ((deb_cover==1)&&(deb_cover_close==0)) // status switch, video,river,file name
		{
		  if (deb_sr_show==1)
		  {
		    deb_cover_close=1;

		    deb_sr_show_start=1;
		    deb_sr_show_nodisp=0;

		    deb_sr_time_set=0;
		    deb_sr_total_bytes=0;

		    deb_sr_show_init=0;
		    deb_sr_river_over=0;
		    deb_sr_sample_over=0;
		    deb_sr_sample_over2=0;
		    deb_sr_river_ptr=0;
		    deb_sr_river_last=0;
		    deb_sr_river_adj=0;

		    deb_disp_bar(cur_stream);
		    g_paint_dirview();//deb_disp_dir(cur_stream);
		    g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		    g_paint_button(0,1);
		    g_paint_label(0);
		    g_paint_checkbox(0,0);
		    g_paint_radiobutton(0,0);
		    g_paint_lineedit(0);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;

		    break;
		  }
		  else
		  {
		    deb_cover_close=1;

		    deb_disp_bar(cur_stream);
		    g_paint_dirview();//deb_disp_dir(cur_stream);
		    g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		    g_paint_button(0,1);
		    g_paint_label(0);
		    g_paint_checkbox(0,0);
		    g_paint_radiobutton(0,0);
		    g_paint_lineedit(0);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;

		    break;
		  }
		}
		else
		{
		  if ((deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		  {
		    deb_sr_show_start=0;
		    deb_sr_show_nodisp=1;

		    deb_disp_bar(cur_stream);
		    g_paint_dirview();//deb_disp_dir(cur_stream);
		    g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		    g_paint_button(0,1);
		    g_paint_label(0);
		    g_paint_checkbox(0,0);
		    g_paint_radiobutton(0,0);
		    g_paint_lineedit(0);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;
		    
		    break;
		  }
		  else
		  {
		    if ((deb_cover==1)&&(deb_cover_close==1))
		    {
		      deb_disp_bar(cur_stream);
		      g_paint_dirview();//deb_disp_dir(cur_stream);
		      g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		      g_paint_button(0,1);
		      g_paint_label(0);
		      g_paint_checkbox(0,0);
		      g_paint_radiobutton(0,0);
		      g_paint_lineedit(0);
		      if (deb_tx_locked==1)
		      {
		        SDL_UnlockTexture(cur_stream->vis_texture);
		        SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		        deb_tx_locked=0;
		      }
		      SDL_RenderPresent(renderer);
		      deb_m_ref=0;

		      deb_cover_close=0;
		      cur_stream->force_refresh = 1;

		      break;
		    }
		    else
		    {
		      if (deb_cover==0)
		      {
			if (deb_sr_show==1)
			{
			  deb_sr_show_start=1;
			  deb_sr_show_nodisp=0;

			  deb_sr_time_set=0;
			  deb_sr_total_bytes=0;

			  deb_sr_show_init=0;
			  deb_sr_river_over=0;
			  deb_sr_sample_over=0;
			  deb_sr_sample_over2=0;
			  deb_sr_river_ptr=0;
			  deb_sr_river_last=0;
			  deb_sr_river_adj=0;
			  
		          deb_disp_bar(cur_stream);
		          g_paint_dirview();//deb_disp_dir(cur_stream);
		          g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		          g_paint_button(0,1);
		          g_paint_label(0);
		          g_paint_checkbox(0,0);
		          g_paint_radiobutton(0,0);
		          g_paint_lineedit(0);
		          if (deb_tx_locked==1)
		          {
		            SDL_UnlockTexture(cur_stream->vis_texture);
		            SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		            deb_tx_locked=0;
		          }
		          SDL_RenderPresent(renderer);
		          deb_m_ref=0;
			  
			  break;
			}
		      }
		    }
		    
		    deb_disp_bar(cur_stream);
		    g_paint_dirview();//deb_disp_dir(cur_stream);
		    g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
		    g_paint_button(0,1);
		    g_paint_label(0);
		    g_paint_checkbox(0,0);
		    g_paint_radiobutton(0,0);
		    g_paint_lineedit(0);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;
		    
		    break;
		  }
		}
		
		//break;
            }

	    if ((g_detect_ptr1==3)&&(g_detect_ptr2==4))   // enable button
	    {
	      strcpy(str1,g_button_text[4]);
		  
	      if (str1[0]=='E')
	      {
	        strcpy(str1,"Disable");
	        g_set_button_text(4,str1,10);
	        
	        g_enable_button(2,1);
	        g_enable_button(3,1);

		g_enable_lineedit(0,1);
		g_enable_lineedit(1,1);
		g_enable_lineedit(2,1);
		
		g_enable_checkbox(0,1);
		g_enable_checkbox(1,1);
		
		g_enable_radiobutton(0,1);
		g_enable_radiobutton(1,1);
		g_enable_radiobutton(2,1);
		g_enable_radiobutton(3,1);

	        g_enable_label(1,1);
	        g_enable_scrollbar(2,1);
	      }
	      else
	      {
	        strcpy(str1,"Enable");
	        g_set_button_text(4,str1,10);

	        g_enable_button(2,0);
	        g_enable_button(3,0);

		g_enable_lineedit(0,0);
		g_enable_lineedit(1,0);
		g_enable_lineedit(2,0);

		g_enable_checkbox(0,0);
		g_enable_checkbox(1,0);

		g_enable_radiobutton(0,0);
		g_enable_radiobutton(1,0);
		g_enable_radiobutton(2,0);
		g_enable_radiobutton(3,0);

	        g_enable_label(1,0);
	        g_enable_scrollbar(2,0);
	      }
	    }

	    if ((g_detect_ptr1==3)&&(g_detect_ptr2==5))   // delete button
	    {
	      g_delete_button(2);
	      g_delete_button(3);

	      g_delete_lineedit(0);
	      g_delete_lineedit(1);
	      g_delete_lineedit(2);
		
	      g_delete_checkbox(0);
	      g_delete_checkbox(1);
		
	      g_delete_radiobutton(0);
	      g_delete_radiobutton(1);
	      g_delete_radiobutton(2);
	      g_delete_radiobutton(3);

	      g_delete_label(1);
	      g_delete_scrollbar(2);
	    }

	    if ((g_detect_ptr1==3)&&(g_detect_ptr2==6))   // create button
	    {
    // for testing of mini GUI lib , color item ,color text
    
    strcpy(str1,"Back");
    g_create_button((cur_stream->width-310)/2,1,45,16,0,255,0,0,str1,5,4);
    g_set_button_text_color(2,0,0,255,0);

    strcpy(str1,"***abcd");
    g_create_lineedit("text",(cur_stream->width-310)/2+45+5,1,200,16,0,255,0,0,str1,(200-4)/7,50,0);
    g_set_lineedit_text_color(0,0,0,255,0);
    
    strcpy(str1,"123");
    g_create_lineedit("number",(cur_stream->width-310)/2+45+5,1+18,200,16,0,255,0,0,str1,(200-4/7),10,2);
    g_set_lineedit_text_color(1,0,0,255,0);
    
    strcpy(str1,"");
    g_create_lineedit("password",(cur_stream->width-310)/2+45+5,1+18*2,200,16,0,255,0,0,str1,(200-4)/7,20,0);
    g_set_lineedit_text_color(2,0,0,255,0);
    
    strcpy(str1,"Search");
    g_create_button((cur_stream->width-310)/2+45+5+200+5,1,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(3,0,0,255,0);

    g_create_checkbox((cur_stream->width-310)/2+45+5,1+18*3,16,16,0,255,0,0);
    g_create_checkbox((cur_stream->width-310)/2+45+5,1+18*4,16,16,0,255,0,0);

    g_create_radiobutton((cur_stream->width-310)/2+45+5,1+18*5,16,16,0,255,0,0,0);
    g_create_radiobutton((cur_stream->width-310)/2+45+5+16+5,1+18*5,16,16,0,255,0,0,0);

    g_create_radiobutton((cur_stream->width-310)/2+45+5,1+18*6,16,16,0,255,0,0,1);
    g_create_radiobutton((cur_stream->width-310)/2+45+5+16+5,1+18*6,16,16,0,255,0,0,1);
    
    g_create_scrollbar((cur_stream->width-310)/2+45+5+200+5+150,1+18*3,180,16,0,255,0,0,0);
    g_set_scrollbar_value_max(2,100);
    
    strcpy(str1,"abc");
    g_create_label((cur_stream->width-310)/2+45+5+200+5+150,1+18*4,150,16,0,255,0,0,str1,6);
    g_set_label_text_color(1,0,0,255,0);
    
    // test end 
	    }
	    
	    if (g_detect_ptr1==4)   // lineedit
	    {
		if ((g_detect_ptr2<0)||(g_detect_ptr2>=G_LINEEDIT_NUM)||(g_detect_ptr2>=g_lineedit_ptr)) break;

		if (g_detect_ptr2!=g_lineedit_current_id)
		{
		  smg_p1=0;
		  g_lineedit_current_id=g_detect_ptr2;
		}
		
		g_lineedit_set_posi(g_detect_ptr2,g_detect_ptr3);		
	    }

	    if (g_detect_ptr1==5)   // check box
	    {
		if ((g_detect_ptr2<0)||(g_detect_ptr2>=G_CHECKBOX_NUM)||(g_detect_ptr2>=g_checkbox_ptr)) break;
		
		if (g_checkbox_value[g_detect_ptr2]==0) g_checkbox_value[g_detect_ptr2]=1;
		else                                    g_checkbox_value[g_detect_ptr2]=0;
	    }

	    if (g_detect_ptr1==6)   // radiobutton
	    {
		if ((g_detect_ptr2<0)||(g_detect_ptr2>=G_RADIOBUTTON_NUM)||(g_detect_ptr2>=g_radiobutton_ptr)) break;
		j=g_radiobutton_value_group[g_detect_ptr2];
		for (k=0;k<g_radiobutton_ptr;k++) if (g_radiobutton_value_group[k]==j) g_radiobutton_value[k]=0;
                g_radiobutton_value[g_detect_ptr2]=1;
	    }
	    
		     deb_m_ref=0;
		     deb_m_ref_v=0;

		     if ((deb_st_play==1)&&(cur_stream->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
		     {
		       video_image_display(cur_stream);
		       vid=1;
		     }
		     else if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		       deb_sr_river_show_pause(cur_stream);
		     else
		     {
		       g_paint_dirview();//deb_disp_dir(cur_stream);
		     }

		     if (deb_m_ref==1)
		     {
		       deb_disp_bar(cur_stream);
		       g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
		       g_paint_button(vid,g_mouse_down);
		       g_paint_label(vid);
		       g_paint_checkbox(vid,g_mouse_down);
		       g_paint_radiobutton(vid,g_mouse_down);
		       g_paint_lineedit(vid);
		       if (deb_tx_locked==1)
		       {
		         SDL_UnlockTexture(cur_stream->vis_texture);
		         SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		         deb_tx_locked=0;
		       }
		       SDL_RenderPresent(renderer);
		       deb_m_ref=0;
		       deb_m_ref_v=0;
		     }
	    
            break;
            
        case SDL_MOUSEMOTION:
        
            /*if (cursor_hidden) {
                SDL_ShowCursor(1);
                cursor_hidden = 0;
            }
            cursor_last_shown = av_gettime_relative();
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button != SDL_BUTTON_RIGHT)
                    break;
                x = event.button.x;
            } else {
                if (!(event.motion.state & SDL_BUTTON_RMASK))
                    break;
                x = event.motion.x;
            }
                if (seek_by_bytes || cur_stream->ic->duration <= 0) {
                    uint64_t size =  avio_size(cur_stream->ic->pb);
                    stream_seek(cur_stream, size*x/cur_stream->width, 0, 1);
                } else {
                    int64_t ts;
                    int ns, hh, mm, ss;
                    int tns, thh, tmm, tss;
                    tns  = cur_stream->ic->duration / 1000000LL;
                    thh  = tns / 3600;
                    tmm  = (tns % 3600) / 60;
                    tss  = (tns % 60);
                    frac = x / cur_stream->width;
                    ns   = frac * tns;
                    hh   = ns / 3600;
                    mm   = (ns % 3600) / 60;
                    ss   = (ns % 60);
                    av_log(NULL, AV_LOG_INFO,
                           "Seek to %2.0f%% (%2d:%02d:%02d) of total duration (%2d:%02d:%02d)       \n", frac*100,
                            hh, mm, ss, thh, tmm, tss);
                    ts = frac * cur_stream->ic->duration;
                    if (cur_stream->ic->start_time != AV_NOPTS_VALUE)
                        ts += cur_stream->ic->start_time;
                    stream_seek(cur_stream, ts, 0, 0);
                }*/
                
	  if (g_dirview_mouse_down==1)
	  {
            cntr++;
            cntr=(cntr % 3);
            if (cntr!=0) break;
              
	    xx2=event.motion.x;
	    yy2=event.motion.y;
	    
	    if (yy2>g_dirview_mouse_down_y)
            {
              yy3=yy2-g_dirview_mouse_down_y;
              //yy4=0;
    
              if (yy3+g_dirview_bottom>g_dirview_posi[0][1]+g_dirview_posi[0][3]-1)
	      {
	        yy3=g_dirview_posi[0][1]+g_dirview_posi[0][3]-1-g_dirview_bottom;
	        //yy4=1;
	      }
    
	      g_dirview_bottom=g_dirview_bottom+yy3;
    
	      deb_filenamebuff_n=deb_filenamebuff_ptr*(g_dirview_bottom-g_dirview_posi[0][1]-1)/(g_dirview_posi[0][3]-2);
	      
	      if (deb_filenamebuff_n>=g_dirview_posi[0][3]/deb_ch_h)
	      {
	        deb_filenamebuff_n=deb_filenamebuff_n-g_dirview_posi[0][3]/deb_ch_h+1;
	      }
	      else
	      {
	        deb_filenamebuff_n=0;
	      }
    	    }
	    else if (yy2<g_dirview_mouse_down_y)
	    {
	      yy3=g_dirview_mouse_down_y-yy2;
    
	      if (g_dirview_top-yy3<g_dirview_posi[0][1]+1)
	      {
	        yy3=g_dirview_top-g_dirview_posi[0][1]-1;
	      }
    
	      g_dirview_top=g_dirview_top-yy3;
    
	      deb_filenamebuff_n=deb_filenamebuff_ptr*(g_dirview_top-g_dirview_posi[0][1]-1)/(g_dirview_posi[0][3]-2);
    	    }  	    
	    
	    deb_disp_bar(cur_stream);
	    g_paint_dirview();//deb_disp_dir(cur_stream);
	    g_paint_scrollbar(0);//deb_disp_bar(cur_stream);
	    g_paint_button(0,0);
	    g_paint_label(0);
	    g_paint_checkbox(0,0);
	    g_paint_radiobutton(0,0);
	    g_paint_lineedit(0);
	    if (deb_tx_locked==1)
	    {
	      SDL_UnlockTexture(cur_stream->vis_texture);
	      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
	      deb_tx_locked=0;
	    }
	    SDL_RenderPresent(renderer);
	    deb_m_ref=0;

	    g_dirview_mouse_down_x=xx2;
	    g_dirview_mouse_down_y=yy2;
	  }
	  else
	  {
	    xx2=event.motion.x;
	    yy2=event.motion.y;
	    
	    g_detect_motion(xx2,yy2);
	    
	    if (g_detect_motion_ptr1==4) SDL_SetCursor(g_cursor_ibeam);
	    else                         SDL_SetCursor(g_cursor_arrow);
	  }  
           



            break;
            
        case SDL_WINDOWEVENT:
            switch (event.window.event) {
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                
                
                
                
	            vid=0;
            
                    //daipozhi modified
                    av_log(NULL, AV_LOG_INFO,"window size changed,\n");




		    //daipozhi modified
                    screen_w = screen_width  = cur_stream->width  = event.window.data1;
                    screen_h = screen_height = cur_stream->height = event.window.data2;
                    
                    //if (cur_stream->vis_texture) {
                    //    SDL_DestroyTexture(cur_stream->vis_texture);
                    //    cur_stream->vis_texture = NULL;
                    //}
                    
                    if (vk_renderer)
                        vk_renderer_resize(vk_renderer, screen_width, screen_height);
                        




	            //daipozhi modified
	            deb_sr_river_f_init=0; //daipozhi modified for sound river
                    
                    // for no testing of mini GUI lib
                    
		    g_move_dirview(0,0,0,cur_stream->width,cur_stream->height-18*2-9);
		    

                    // for testing of mini GUI lib
                    /*
       		    g_move_dirview(0,0,18*7+1,cur_stream->width,cur_stream->height-18*9-9);
       		    */
       		    // test end
                    

		    
		    g_move_scrollbar(0,5,cur_stream->height-18*2+1,cur_stream->width-5-180-5-5,16);
		    g_move_scrollbar(1,cur_stream->width-5-180,cur_stream->height-18*2+1,180,16);
		    g_move_button(0,(cur_stream->width-80)/2,cur_stream->height-18+1,80,16);
		    g_move_button(1,cur_stream->width-120-5,cur_stream->height-18+1,120,16);
		    g_move_label(0,5,cur_stream->height-18+1,150,16);
		    

                    // for testing of mini GUI lib
                    /*
		    g_move_button(2,(cur_stream->width-310)/2,1,45,16);
		    g_move_lineedit(0,(cur_stream->width-310)/2+45+5,1,200,16);
		    g_move_lineedit(1,(cur_stream->width-310)/2+45+5,1+18,200,16);
		    g_move_lineedit(2,(cur_stream->width-310)/2+45+5,1+18*2,200,16);
		    g_move_button(3,(cur_stream->width-310)/2+45+5+200+5,1,55,16);
		    g_move_checkbox(0,(cur_stream->width-310)/2+45+5,1+18*3,16,16);
		    g_move_checkbox(1,(cur_stream->width-310)/2+45+5,1+18*4,16,16);
		    g_move_radiobutton(0,(cur_stream->width-310)/2+45+5,1+18*5,16,16);
		    g_move_radiobutton(1,(cur_stream->width-310)/2+45+5+16+5,1+18*5,16,16);
		    g_move_radiobutton(2,(cur_stream->width-310)/2+45+5,1+18*6,16,16);
		    g_move_radiobutton(3,(cur_stream->width-310)/2+45+5+16+5,1+18*6,16,16);
                    */
                    // test end
                    
             
             	     deb_conv_dir();   //resize dir view
             
		     deb_m_ref=0;
		     deb_m_ref_v=0;

		     if ((deb_st_play==1)&&(cur_stream->show_mode == SHOW_MODE_VIDEO)&&(deb_cover_close==0))
		     {
		       video_image_display(cur_stream);
		       vid=1;
		     }
		     else if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		       deb_sr_river_show_pause(cur_stream);
		     else
		     {
		       g_paint_dirview();//deb_disp_dir(cur_stream);
		     }

		     if (deb_m_ref==1)
		     {
		       deb_disp_bar(cur_stream);
		       g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
		       g_paint_button(vid,0);
		       g_paint_label(vid);
		       g_paint_checkbox(vid,0);
		       g_paint_radiobutton(vid,0);
		       g_paint_lineedit(vid);
		       if (deb_tx_locked==1)
		       {
		         SDL_UnlockTexture(cur_stream->vis_texture);
		         SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		         deb_tx_locked=0;
		       }
		       SDL_RenderPresent(renderer);
		       deb_m_ref=0;
		       deb_m_ref_v=0;
		     }




                    //daipozhi modified
                    break;
                    
                    
                    
                    
                case SDL_WINDOWEVENT_EXPOSED:
                
                
                
                
                    //daipozhi modified
                    av_log(NULL, AV_LOG_INFO,"window exposed,\n");




                    cur_stream->force_refresh = 1;
            }
            break;
        case SDL_QUIT:




        //daipozhi modified
        av_log(NULL, AV_LOG_INFO,"sdl quit,\n");
        
#if DPZ_DEBUG1
	    deb_record_close();
#endif
            do_exit(cur_stream);
            break;



            
        case FF_QUIT_EVENT:




            vid=0;

            //daipozhi modified
            av_log(NULL, AV_LOG_INFO,"ffplay quit,\n");
            
            //do_exit(cur_stream);

            if (deb_stream_err==1)
            {
              deb_st_play=0;

	      deb_disp_bar(cur_stream);
	      g_paint_dirview();//deb_disp_dir(cur_stream);
              g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
              g_paint_button(vid,0);
	      g_paint_label(vid);
	      g_paint_checkbox(vid,0);
	      g_paint_radiobutton(vid,0);
	      g_paint_lineedit(vid);
	      if (deb_tx_locked==1)
	      {
	        SDL_UnlockTexture(cur_stream->vis_texture);
	        SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		deb_tx_locked=0;
	      }
	      SDL_RenderPresent(renderer);
	      deb_m_ref=0;

              break;
            }

	    //if (cur_stream->paused) toggle_pause(cur_stream);

	    stream_close(cur_stream);
            //uninit_opts();
            //deb_opts_stt=0;

	    deb_cover=0;
	    deb_cover_close=0;
	    deb_frame_num=0;

	    deb_ini_is(cur_stream);
	    deb_video_open_again(cur_stream,0);

	    deb_filenameplay++;
	    
	    while ((deb_filenamebuff_subline[deb_filenameplay]==1)&&(deb_filenameplay<deb_filenamebuff_ptr))
	    {
	      deb_filenameplay++;
	    }

	    deb_disp_bar(cur_stream);
	    g_paint_dirview();//deb_disp_dir(cur_stream);
            g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
            g_paint_button(vid,0);
	    g_paint_label(vid);
	    g_paint_checkbox(vid,0);
	    g_paint_radiobutton(vid,0);
	    g_paint_lineedit(vid);
	    if (deb_tx_locked==1)
	    {
	      SDL_UnlockTexture(cur_stream->vis_texture);
	      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
	      deb_tx_locked=0;
	    }
	    SDL_RenderPresent(renderer);
	    deb_m_ref=0;

            if (deb_filenameplay>=deb_filenamebuff_ptr) // more than filenamecnt 
	    {
	      deb_filenameplay=deb_filenamebuff_ptr-1;
              break;
	    }
            if (deb_filenameplay>=MAX_FILE_NUM) // more than MAX_FILE_NUM
	    {
	      deb_filenameplay=MAX_FILE_NUM-1;
              break;
	    }
            if (deb_filenameplay<0) 
	    {
	      deb_filenameplay=0;
              break;
	    }

	    str3[0]=0;
	    strcpy(str3,deb_filenamebuff[deb_filenameplay]);
	    l=deb_filenameplay+1;
		  
	    while ((deb_filenamebuff_subline[l]==1)&&(l<deb_filenamebuff_ptr))
	    {
	      if (strlen(str3)+strlen(deb_filenamebuff[l])>=FN_SIZE) break;
		    
	      strcat(str3,deb_filenamebuff[l]);
	      l++;
	    }

            sc1=deb_filenamebuff_type[deb_filenameplay];  //daipozhi modified for audio
            if (sc1==2) break; // comment line
            if (sc1==3) break; // empty line
	    if (sc1!=0)        // not dir not empty
	    {
	      if (deb_no_support(str3,FN_SIZE)==1) break;

	      deb_get_path(deb_filenameplay);

	      //deb_ini_is(cur_stream);
	      deb_video_open_again(cur_stream,0); //show file name in window title

              if (deb_opts_stt==1)
              {
                uninit_opts();
                deb_opts_stt=0;
              }

              init_opts();
              deb_opts_stt=1;

	      deb_thr_v=0;
	      deb_thr_s=0;
	      deb_thr_a=0;
	      deb_thr_a2=0;
	      deb_thr_r=0;

	      deb_sr_time_set=0;
	      deb_sr_total_bytes=0;
	      deb_sr_show=0;
	      deb_sr_show_start=0;
	      deb_sr_show_nodisp=1;
	      deb_sr_show_init=0;
	      deb_sr_river_over=0;
	      deb_sr_sample_over=0;
	      deb_sr_sample_over2=0;
	      deb_sr_river_ptr=0;
	      deb_sr_river_last=0;
	      deb_sr_river_adj=0;

	      if (deb_str_has_null(deb_dir_buffer,FN_SIZE)!=1) break;
	      if ((strlen(deb_dir_buffer)>=FN_SIZE)||((int)strlen(deb_dir_buffer)<0)) break;
	      strcpy(deb_currentpath,deb_dir_buffer);

              deb_stream_err=1;
              deb_stream_open=1;

	      stream_open(deb_dir_buffer, file_iformat,2);

	      //deb_disp_dir(cur_stream);  
	      //deb_disp_bar(cur_stream);
	      //SDL_RenderPresent(renderer);
	      break;
	    }
/*
            if (deb_opts_stt==1)
            {
              uninit_opts();
              deb_opts_stt=0;
            }

            init_opts();
            deb_opts_stt=1;
*/
            break;




        //daipozhi modified
	case SDL_MOUSEWHEEL:
	
	        vid=0;

                av_log(NULL, AV_LOG_INFO,"mouse wheel event,\n");
                  
		if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0)) break;

		if (cur_stream->video_st && cur_stream->show_mode == SHOW_MODE_VIDEO)
			if (deb_cover_close==0) break;

	        if (event.wheel.y > 0) // wheel scroll up
	        {
		    if (deb_filenamebuff_n>=5) deb_filenamebuff_n=deb_filenamebuff_n-5;
		    else deb_filenamebuff_n=0;
		  
		    deb_disp_bar(cur_stream);
	            g_paint_dirview();//deb_disp_dir(cur_stream);
                    g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
                    g_paint_button(vid,0);
		    g_paint_label(vid);
		    g_paint_checkbox(vid,0);
		    g_paint_radiobutton(vid,0);
		    g_paint_lineedit(vid);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;
		  
		    break;
	        }
	        else if (event.wheel.y < 0) // wheel scroll down
	        {
		    n3=(cur_stream->height)/deb_ch_h-2-2;
		    
		    if (deb_filenamebuff_n+n3+5<deb_filenamebuff_ptr) deb_filenamebuff_n=deb_filenamebuff_n+5;
		    else
		    {
		      deb_filenamebuff_n=deb_filenamebuff_ptr-n3;
		      if (deb_filenamebuff_n<0) deb_filenamebuff_n=0;
		    }
		    
		    deb_disp_bar(cur_stream);
	            g_paint_dirview();//deb_disp_dir(cur_stream);
                    g_paint_scrollbar(vid);//deb_disp_bar(cur_stream);
                    g_paint_button(vid,0);
		    g_paint_label(vid);
		    g_paint_checkbox(vid,0);
		    g_paint_radiobutton(vid,0);
		    g_paint_lineedit(vid);
		    if (deb_tx_locked==1)
		    {
		      SDL_UnlockTexture(cur_stream->vis_texture);
		      SDL_RenderCopy(renderer, cur_stream->vis_texture, NULL, NULL);
		      deb_tx_locked=0;
		    }
		    SDL_RenderPresent(renderer);
		    deb_m_ref=0;
		    
		    break;
	        }

		break;




        default:
            break;
        }
    }
}

static int opt_width(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_width = num;
    return 0;
}

static int opt_height(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_height = num;
    return 0;
}

static int opt_format(void *optctx, const char *opt, const char *arg)
{
    file_iformat = av_find_input_format(arg);
    if (!file_iformat) {
        av_log(NULL, AV_LOG_FATAL, "Unknown input format: %s\n", arg);
        return AVERROR(EINVAL);
    }
    return 0;
}

static int opt_sync(void *optctx, const char *opt, const char *arg)
{
    if (!strcmp(arg, "audio"))
        av_sync_type = AV_SYNC_AUDIO_MASTER;
    else if (!strcmp(arg, "video"))
        av_sync_type = AV_SYNC_VIDEO_MASTER;
    else if (!strcmp(arg, "ext"))
        av_sync_type = AV_SYNC_EXTERNAL_CLOCK;
    else {
        av_log(NULL, AV_LOG_ERROR, "Unknown value for %s: %s\n", opt, arg);
        exit(1);
    }
    return 0;
}

static int opt_show_mode(void *optctx, const char *opt, const char *arg)
{
    show_mode = !strcmp(arg, "video") ? SHOW_MODE_VIDEO :
                !strcmp(arg, "waves") ? SHOW_MODE_WAVES :
                !strcmp(arg, "rdft" ) ? SHOW_MODE_RDFT  : SHOW_MODE_NONE;

    if (show_mode == SHOW_MODE_NONE) {
        double num;
        int ret = parse_number(opt, arg, OPT_TYPE_INT, 0, SHOW_MODE_NB-1, &num);
        if (ret < 0)
            return ret;
        show_mode = num;
    }
    return 0;
}

static int opt_input_file(void *optctx, const char *filename)
{
    if (input_filename) {
        av_log(NULL, AV_LOG_FATAL,
               "Argument '%s' provided as input filename, but '%s' was already specified.\n",
                filename, input_filename);
        return AVERROR(EINVAL);
    }
    if (!strcmp(filename, "-"))
        filename = "fd:";
    input_filename = av_strdup(filename);
    if (!input_filename)
        return AVERROR(ENOMEM);

    return 0;
}

static int opt_codec(void *optctx, const char *opt, const char *arg)
{
   const char *spec = strchr(opt, ':');
   const char **name;
   if (!spec) {
       av_log(NULL, AV_LOG_ERROR,
              "No media specifier was specified in '%s' in option '%s'\n",
               arg, opt);
       return AVERROR(EINVAL);
   }
   spec++;

   switch (spec[0]) {
   case 'a' : name = &audio_codec_name;    break;
   case 's' : name = &subtitle_codec_name; break;
   case 'v' : name = &video_codec_name;    break;
   default:
       av_log(NULL, AV_LOG_ERROR,
              "Invalid media specifier '%s' in option '%s'\n", spec, opt);
       return AVERROR(EINVAL);
   }

   av_freep(name);
   *name = av_strdup(arg);
   return *name ? 0 : AVERROR(ENOMEM);
}

static int dummy;

static const OptionDef options[] = {
    CMDUTILS_COMMON_OPTIONS
    { "x",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_width }, "force displayed width", "width" },
    { "y",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_height }, "force displayed height", "height" },
    { "fs",                 OPT_TYPE_BOOL,            0, { &is_full_screen }, "force full screen" },
    { "an",                 OPT_TYPE_BOOL,            0, { &audio_disable }, "disable audio" },
    { "vn",                 OPT_TYPE_BOOL,            0, { &video_disable }, "disable video" },
    { "sn",                 OPT_TYPE_BOOL,            0, { &subtitle_disable }, "disable subtitling" },
    { "ast",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_AUDIO] }, "select desired audio stream", "stream_specifier" },
    { "vst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_VIDEO] }, "select desired video stream", "stream_specifier" },
    { "sst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_SUBTITLE] }, "select desired subtitle stream", "stream_specifier" },
    { "ss",                 OPT_TYPE_TIME,            0, { &start_time }, "seek to a given position in seconds", "pos" },
    { "t",                  OPT_TYPE_TIME,            0, { &duration }, "play  \"duration\" seconds of audio/video", "duration" },
    { "bytes",              OPT_TYPE_INT,             0, { &seek_by_bytes }, "seek by bytes 0=off 1=on -1=auto", "val" },
    { "seek_interval",      OPT_TYPE_FLOAT,           0, { &seek_interval }, "set seek interval for left/right keys, in seconds", "seconds" },
    { "nodisp",             OPT_TYPE_BOOL,            0, { &display_disable }, "disable graphical display" },
    { "noborder",           OPT_TYPE_BOOL,            0, { &borderless }, "borderless window" },
    { "alwaysontop",        OPT_TYPE_BOOL,            0, { &alwaysontop }, "window always on top" },
    { "volume",             OPT_TYPE_INT,             0, { &startup_volume}, "set startup volume 0=min 100=max", "volume" },
    { "f",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_format }, "force format", "fmt" },
    { "stats",              OPT_TYPE_BOOL,   OPT_EXPERT, { &show_status }, "show status", "" },
    { "fast",               OPT_TYPE_BOOL,   OPT_EXPERT, { &fast }, "non spec compliant optimizations", "" },
    { "genpts",             OPT_TYPE_BOOL,   OPT_EXPERT, { &genpts }, "generate pts", "" },
    { "drp",                OPT_TYPE_INT,    OPT_EXPERT, { &decoder_reorder_pts }, "let decoder reorder pts 0=off 1=on -1=auto", ""},
    { "lowres",             OPT_TYPE_INT,    OPT_EXPERT, { &lowres }, "", "" },
    { "sync",               OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_sync }, "set audio-video sync. type (type=audio/video/ext)", "type" },
    { "autoexit",           OPT_TYPE_BOOL,   OPT_EXPERT, { &autoexit }, "exit at the end", "" },
    { "exitonkeydown",      OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_keydown }, "exit on key down", "" },
    { "exitonmousedown",    OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_mousedown }, "exit on mouse down", "" },
    { "loop",               OPT_TYPE_INT,    OPT_EXPERT, { &loop }, "set number of times the playback shall be looped", "loop count" },
    { "framedrop",          OPT_TYPE_BOOL,   OPT_EXPERT, { &framedrop }, "drop frames when cpu is too slow", "" },
    { "infbuf",             OPT_TYPE_BOOL,   OPT_EXPERT, { &infinite_buffer }, "don't limit the input buffer size (useful with realtime streams)", "" },
    { "window_title",       OPT_TYPE_STRING,          0, { &window_title }, "set window title", "window title" },
    { "left",               OPT_TYPE_INT,    OPT_EXPERT, { &screen_left }, "set the x position for the left of the window", "x pos" },
    { "top",                OPT_TYPE_INT,    OPT_EXPERT, { &screen_top }, "set the y position for the top of the window", "y pos" },
    { "vf",                 OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_add_vfilter }, "set video filters", "filter_graph" },
    { "af",                 OPT_TYPE_STRING,          0, { &afilters }, "set audio filters", "filter_graph" },
    { "rdftspeed",          OPT_TYPE_INT, OPT_AUDIO | OPT_EXPERT, { &rdftspeed }, "rdft speed", "msecs" },
    { "showmode",           OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_show_mode}, "select show mode (0 = video, 1 = waves, 2 = RDFT)", "mode" },
    { "i",                  OPT_TYPE_BOOL,            0, { &dummy}, "read specified file", "input_file"},
    { "codec",              OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_codec}, "force decoder", "decoder_name" },
    { "acodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &audio_codec_name }, "force audio decoder",    "decoder_name" },
    { "scodec",             OPT_TYPE_STRING, OPT_EXPERT, { &subtitle_codec_name }, "force subtitle decoder", "decoder_name" },
    { "vcodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &video_codec_name }, "force video decoder",    "decoder_name" },
    { "autorotate",         OPT_TYPE_BOOL,            0, { &autorotate }, "automatically rotate video", "" },
    { "find_stream_info",   OPT_TYPE_BOOL, OPT_INPUT | OPT_EXPERT, { &find_stream_info },
        "read and decode the streams to fill missing information with heuristics" },
    { "filter_threads",     OPT_TYPE_INT,    OPT_EXPERT, { &filter_nbthreads }, "number of filter threads per graph" },
    { "enable_vulkan",      OPT_TYPE_BOOL,            0, { &enable_vulkan }, "enable vulkan renderer" },
    { "vulkan_params",      OPT_TYPE_STRING, OPT_EXPERT, { &vulkan_params }, "vulkan configuration using a list of key=value pairs separated by ':'" },
    { "hwaccel",            OPT_TYPE_STRING, OPT_EXPERT, { &hwaccel }, "use HW accelerated decoding" },
    { NULL, },
};

static void show_usage(void)
{
    av_log(NULL, AV_LOG_INFO, "Simple media player\n");
    av_log(NULL, AV_LOG_INFO, "usage: %s [options] input_file\n", program_name);
    av_log(NULL, AV_LOG_INFO, "\n");
}

void show_help_default(const char *opt, const char *arg)
{
    av_log_set_callback(log_callback_help);
    show_usage();
    show_help_options(options, "Main options:", 0, OPT_EXPERT);
    show_help_options(options, "Advanced options:", OPT_EXPERT, 0);
    printf("\n");
    show_help_children(avcodec_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avformat_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avfilter_get_class(), AV_OPT_FLAG_FILTERING_PARAM);
    printf("\nWhile playing:\n"
           "q, ESC              quit\n"
           "f                   toggle full screen\n"
           "p, SPC              pause\n"
           "m                   toggle mute\n"
           "9, 0                decrease and increase volume respectively\n"
           "/, *                decrease and increase volume respectively\n"
           "a                   cycle audio channel in the current program\n"
           "v                   cycle video channel\n"
           "t                   cycle subtitle channel in the current program\n"
           "c                   cycle program\n"
           "w                   cycle video filters or show modes\n"
           "s                   activate frame-step mode\n"
           "left/right          seek backward/forward 10 seconds or to custom interval if -seek_interval is set\n"
           "down/up             seek backward/forward 1 minute\n"
           "page down/page up   seek backward/forward 10 minutes\n"
           "right mouse click   seek to percentage in file corresponding to fraction of width\n"
           "left double-click   toggle full screen\n"
           );
}




//daipozhi modified
/* Called from the main */
int main(void/*int argc, char **argv*/)
{
    int flags, ret;
    VideoState *is;




    // daipozhi modified
    int    arc;
    char   arv3[30][30];
    char  *arv2[10];
    char **arv;
    
    char   str1[FN_SIZE];

    arc=2;
    strcpy(arv3[0],"ffplay.exe");
    strcpy(arv3[1],"sample.mp3");
    arv2[0]= arv3[0];
    arv2[1]= arv3[1];
    arv = arv2;




    //daipozhi modified
    setlocale(LC_ALL,"chs");




    init_dynload();

    av_log_set_flags(AV_LOG_SKIP_REPEATED);




    //daipozhi modified
    parse_loglevel(arc, arv, options);




    /* register all codecs, demux and protocols */
#if CONFIG_AVDEVICE
    avdevice_register_all();
#endif
    avformat_network_init();




    //daipozhi modified
    deb_opts_stt=1;




    signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).    */
    signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */




    //daipozhi modified
    show_banner(arc, arv, options);




    //daipozhi modified
    parse_options(NULL, arc, arv, options, opt_input_file);




    //daipozhi modified
    //if (ret < 0)
    //    exit(ret == AVERROR_EXIT ? 0 : 1);




    if (!input_filename) {
        show_usage();
        av_log(NULL, AV_LOG_FATAL, "An input file must be specified\n");
        av_log(NULL, AV_LOG_FATAL,
               "Use -h to get full help or, even better, run 'man %s'\n", program_name);
        exit(1);
    }

    if (display_disable) {
        video_disable = 1;
    }
    flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;
    if (audio_disable)
        flags &= ~SDL_INIT_AUDIO;
    else {
        /* Try to work around an occasional ALSA buffer underflow issue when the
         * period size is NPOT due to ALSA resampling by forcing the buffer size. */
        if (!SDL_getenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE"))
            SDL_setenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE","1", 1);
    }
    if (display_disable)
        flags &= ~SDL_INIT_VIDEO;
    if (SDL_Init (flags)) {
        av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n", SDL_GetError());
        av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
        exit(1);
    }

    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

    if (!display_disable) {
        int flags = SDL_WINDOW_HIDDEN;
        if (alwaysontop)
#if SDL_VERSION_ATLEAST(2,0,5)
            flags |= SDL_WINDOW_ALWAYS_ON_TOP;
#else
            av_log(NULL, AV_LOG_WARNING, "Your SDL version doesn't support SDL_WINDOW_ALWAYS_ON_TOP. Feature will be inactive.\n");
#endif
        if (borderless)
            flags |= SDL_WINDOW_BORDERLESS;
        else
            flags |= SDL_WINDOW_RESIZABLE;

#ifdef SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR
        SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
#endif
        if (hwaccel && !enable_vulkan) {
            av_log(NULL, AV_LOG_INFO, "Enable vulkan renderer to support hwaccel %s\n", hwaccel);
            enable_vulkan = 1;
        }
        if (enable_vulkan) {
            vk_renderer = vk_get_renderer();
            if (vk_renderer) {
#if SDL_VERSION_ATLEAST(2, 0, 6)
                flags |= SDL_WINDOW_VULKAN;
#endif
            } else {
                av_log(NULL, AV_LOG_WARNING, "Doesn't support vulkan renderer, fallback to SDL renderer\n");
                enable_vulkan = 0;
            }
        }
        window = SDL_CreateWindow(program_name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (!window) {
            av_log(NULL, AV_LOG_FATAL, "Failed to create window: %s", SDL_GetError());
            do_exit(NULL);
        }

        if (vk_renderer) {
            AVDictionary *dict = NULL;

            if (vulkan_params) {
                int ret = av_dict_parse_string(&dict, vulkan_params, "=", ":", 0);
                if (ret < 0) {
                    av_log(NULL, AV_LOG_FATAL, "Failed to parse, %s\n", vulkan_params);
                    do_exit(NULL);
                }
            }
            ret = vk_renderer_create(vk_renderer, window, dict);
            av_dict_free(&dict);
            if (ret < 0) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create vulkan renderer, %s\n", av_err2str(ret));
                do_exit(NULL);
            }
        } else {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (!renderer) {
                av_log(NULL, AV_LOG_WARNING, "Failed to initialize a hardware accelerated renderer: %s\n", SDL_GetError());
                renderer = SDL_CreateRenderer(window, -1, 0);
            }
            if (renderer) {
                if (!SDL_GetRendererInfo(renderer, &renderer_info))
                    av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n", renderer_info.name);
            }
            if (!renderer || !renderer_info.num_texture_formats) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create window or renderer: %s", SDL_GetError());
                do_exit(NULL);
            }
        }
    }




    //daipozhi modified
    //is = stream_open(input_filename, file_iformat);
    is = stream_open(input_filename, file_iformat,1);




    if (!is) {
        av_log(NULL, AV_LOG_FATAL, "Failed to initialize VideoState!\n");
        do_exit(NULL);
    }




#if DPZ_DEBUG1
    deb_record_init();
#endif
    deb_load_font();   //daipozhi modified
    
    g_load_icon();

    video_open(is);  //daipozhi modified 

    deb_get_dir_ini();  //daipozhi modified  

    //cur_stream->width
    //cur_stream->height

    g_init();

    // for no testing of mini GUI lib
    
    g_create_dirview(0,0,is->width,is->height-18*2-9);
    
    
    // for testing of mini GUI lib
    /*
    g_create_dirview(0,18*7+1,is->width,is->height-18*9-9);
    */
    // test end
    
    g_create_scrollbar(5,is->height-18*2+1,is->width-5-180-5-5,16,0,224,224,224,0);
    g_create_scrollbar(is->width-5-180,is->height-18*2+1,180,16,0,224,224,224,0);
    
    strcpy(str1,"Play");
    g_create_button((is->width-80)/2,is->height-18+1,80,16,0,160,160,160,str1,5,4);
    g_set_button_text_color(0,1,0,0,0);
    
    strcpy(str1,"");
    g_create_button(is->width-120-5,is->height-18+1,120,16,0,160,160,160,str1,5,4);
    g_set_button_text_color(1,1,0,0,0);

    strcpy(str1,"");
    g_create_label(5,is->height-18+1,150,16,0,240,240,240,str1,5);
    g_set_label_text_color(0,1,0,0,0);

    // for testing of mini GUI lib , color item ,color text
    /*
    strcpy(str1,"Back");
    g_create_button((is->width-310)/2,1,45,16,0,255,0,0,str1,5,4);
    g_set_button_text_color(2,0,0,255,0);

    strcpy(str1,"***abcd");
    g_create_lineedit("text",(is->width-310)/2+45+5,1,200,16,0,255,0,0,str1,(200-4)/7,50,0);
    g_set_lineedit_text_color(0,0,0,255,0);
    
    strcpy(str1,"123");
    g_create_lineedit("number",(is->width-310)/2+45+5,1+18,200,16,0,255,0,0,str1,(200-4/7),10,2);
    g_set_lineedit_text_color(1,0,0,255,0);
    
    strcpy(str1,"");
    g_create_lineedit("password",(is->width-310)/2+45+5,1+18*2,200,16,0,255,0,0,str1,(200-4)/7,20,0);
    g_set_lineedit_text_color(2,0,0,255,0);
    
    strcpy(str1,"Search");
    g_create_button((is->width-310)/2+45+5+200+5,1,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(3,0,0,255,0);

    strcpy(str1,"Enable");
    g_create_button((is->width-310)/2+45+5+200+5+150,1,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(4,0,0,255,0);

    strcpy(str1,"Delete");
    g_create_button((is->width-310)/2+45+5+200+5+150,1+18,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(5,0,0,255,0);

    strcpy(str1,"Create");
    g_create_button((is->width-310)/2+45+5+200+5+150,1+18*2,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(6,0,0,255,0);

    g_create_checkbox((is->width-310)/2+45+5,1+18*3,16,16,0,255,0,0);
    g_create_checkbox((is->width-310)/2+45+5,1+18*4,16,16,0,255,0,0);

    g_create_radiobutton((is->width-310)/2+45+5,1+18*5,16,16,0,255,0,0,0);
    g_create_radiobutton((is->width-310)/2+45+5+16+5,1+18*5,16,16,0,255,0,0,0);

    g_create_radiobutton((is->width-310)/2+45+5,1+18*6,16,16,0,255,0,0,1);
    g_create_radiobutton((is->width-310)/2+45+5+16+5,1+18*6,16,16,0,255,0,0,1);
    
    g_create_scrollbar((is->width-310)/2+45+5+200+5+150,1+18*3,180,16,0,255,0,0,0);
    g_set_scrollbar_value_max(2,100);
    
    strcpy(str1,"abc");
    g_create_label((is->width-310)/2+45+5+200+5+150,1+18*4,150,16,0,255,0,0,str1,6);
    g_set_label_text_color(1,0,0,255,0);
    */
    // test end 


    // for testing of mini GUI lib , color item ,black text
    /*
    strcpy(str1,"Back");
    g_create_button((is->width-310)/2,1,45,16,0,255,0,0,str1,5,4);
    g_set_button_text_color(2,1,0,0,0);

    strcpy(str1,"***abcd");
    g_create_lineedit("text",(is->width-310)/2+45+5,1,200,16,0,255,0,0,str1,(200-4)/7,50,0);
    g_set_lineedit_text_color(0,1,0,0,0);
    
    strcpy(str1,"123");
    g_create_lineedit("number",(is->width-310)/2+45+5,1+18,200,16,0,255,0,0,str1,(200-4/7),10,2);
    g_set_lineedit_text_color(1,1,0,0,0);
    
    strcpy(str1,"");
    g_create_lineedit("password",(is->width-310)/2+45+5,1+18*2,200,16,0,255,0,0,str1,(200-4)/7,20,0);
    g_set_lineedit_text_color(2,1,0,0,0);
    
    strcpy(str1,"Search");
    g_create_button((is->width-310)/2+45+5+200+5,1,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(3,1,0,0,0);

    strcpy(str1,"Enable");
    g_create_button((is->width-310)/2+45+5+200+5+150,1,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(4,1,0,0,0);

    strcpy(str1,"Delete");
    g_create_button((is->width-310)/2+45+5+200+5+150,1+18,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(5,1,0,0,0);

    strcpy(str1,"Create");
    g_create_button((is->width-310)/2+45+5+200+5+150,1+18*2,55,16,0,255,0,0,str1,7,4);
    g_set_button_text_color(6,1,0,0,0);

    g_create_checkbox((is->width-310)/2+45+5,1+18*3,16,16,0,255,0,0);
    g_create_checkbox((is->width-310)/2+45+5,1+18*4,16,16,0,255,0,0);

    g_create_radiobutton((is->width-310)/2+45+5,1+18*5,16,16,0,255,0,0,0);
    g_create_radiobutton((is->width-310)/2+45+5+16+5,1+18*5,16,16,0,255,0,0,0);

    g_create_radiobutton((is->width-310)/2+45+5,1+18*6,16,16,0,255,0,0,1);
    g_create_radiobutton((is->width-310)/2+45+5+16+5,1+18*6,16,16,0,255,0,0,1);
    
    g_create_scrollbar((is->width-310)/2+45+5+200+5+150,1+18*3,180,16,0,255,0,0,0);
    g_set_scrollbar_value_max(2,100);
    
    strcpy(str1,"abc");
    g_create_label((is->width-310)/2+45+5+200+5+150,1+18*4,150,16,0,255,0,0,str1,6);
    g_set_label_text_color(1,1,0,0,0);
    */
    // test end 


    // for testing of mini GUI lib , black item color text
    /*
    strcpy(str1,"Back");
    g_create_button((is->width-310)/2,1,45,16,1,0,0,0,str1,5,4);
    g_set_button_text_color(2,0,255,255,255);

    strcpy(str1,"***abcd");
    g_create_lineedit("text",(is->width-310)/2+45+5,1,200,16,1,0,0,0,str1,(200-4)/7,50,0);
    g_set_lineedit_text_color(0,0,255,255,255);
    
    strcpy(str1,"123");
    g_create_lineedit("number",(is->width-310)/2+45+5,1+18,200,16,1,0,0,0,str1,(200-4/7),10,2);
    g_set_lineedit_text_color(1,0,255,255,255);
    
    strcpy(str1,"");
    g_create_lineedit("password",(is->width-310)/2+45+5,1+18*2,200,16,1,0,0,0,str1,(200-4)/7,20,0);
    g_set_lineedit_text_color(2,0,255,255,255);
    
    strcpy(str1,"Search");
    g_create_button((is->width-310)/2+45+5+200+5,1,55,16,1,0,0,0,str1,7,4);
    g_set_button_text_color(3,0,255,255,255);

    strcpy(str1,"Enable");
    g_create_button((is->width-310)/2+45+5+200+5+150,1,55,16,1,0,0,0,str1,7,4);
    g_set_button_text_color(4,0,255,255,255);

    strcpy(str1,"Delete");
    g_create_button((is->width-310)/2+45+5+200+5+150,1+18,55,16,1,0,0,0,str1,7,4);
    g_set_button_text_color(5,0,255,255,255);

    strcpy(str1,"Create");
    g_create_button((is->width-310)/2+45+5+200+5+150,1+18*2,55,16,1,0,0,0,str1,7,4);
    g_set_button_text_color(6,0,255,255,255);

    g_create_checkbox((is->width-310)/2+45+5,1+18*3,16,16,1,0,0,0);
    g_create_checkbox((is->width-310)/2+45+5,1+18*4,16,16,1,0,0,0);

    g_create_radiobutton((is->width-310)/2+45+5,1+18*5,16,16,1,0,0,0,0);
    g_create_radiobutton((is->width-310)/2+45+5+16+5,1+18*5,16,16,1,0,0,0,0);

    g_create_radiobutton((is->width-310)/2+45+5,1+18*6,16,16,1,0,0,0,1);
    g_create_radiobutton((is->width-310)/2+45+5+16+5,1+18*6,16,16,1,0,0,0,1);

    g_create_scrollbar((is->width-310)/2+45+5+200+5+150,1+18*3,180,16,1,0,0,0,0);
    g_set_scrollbar_value_max(2,100);
    
    strcpy(str1,"abc");
    g_create_label((is->width-310)/2+45+5+200+5+150,1+18*4,150,16,1,0,0,0,str1,6);
    g_set_label_text_color(1,0,255,255,255);
    */
    // test end 


    g_set_scrollbar_value_max(1,SDL_MIX_MAXVOLUME);
    g_set_scrollbar_value_now(1,SDL_MIX_MAXVOLUME);

    g_paint_dirview();
    g_paint_scrollbar(0);
    g_paint_button(0,0);
    g_paint_label(0);
    g_paint_checkbox(0,0);
    g_paint_radiobutton(0,0);
    g_paint_lineedit(0);
    
    if (deb_tx_locked==1)
    {
      SDL_UnlockTexture(is->vis_texture);
      SDL_RenderCopy(renderer, is->vis_texture, NULL, NULL);
      deb_tx_locked=0;
    }
    SDL_RenderPresent(renderer);
    deb_m_ref=0;

    // daipozhi for sound river
    deb_sr_fft_set_db(deb_sr_rate);
    deb_sr_river_f_init=0;

    g_cursor_arrow=SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_ARROW);
    g_cursor_ibeam=SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_IBEAM);




    event_loop(is);

    /* never returns */

    return 0;
}






// daipozhi modified
static int deb_load_font(void)
{
  int  i;    

  // daipozhi modified
  deb_fh=open("./utf8_bmp/utf8_bmp.data",O_RDONLY,S_IRUSR);
  if (deb_fh<0)
  {
    printf("Open file './utf8_bmp/utf8_bmp.data' error\n");
    return(1);
  }

  u_str = (unsigned char*)u_b1p;  i=read(deb_fh, u_str,   128 * 14 * 14);  if (i<  128 * 14 * 14) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b2p;  i=read(deb_fh, u_str,  1920 * 14 * 14);  if (i< 1920 * 14 * 14) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b3p;  i=read(deb_fh, u_str,  2048 * 14 * 14);  if (i< 2048 * 14 * 14) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b4p;  i=read(deb_fh, u_str, 49152 * 14 * 14);  if (i<49152 * 14 * 14) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b5p;  i=read(deb_fh, u_str,  2048 * 14 * 14);  if (i< 2048 * 14 * 14) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b6p;  i=read(deb_fh, u_str,  8192 * 14 * 14);  if (i< 8192 * 14 * 14) { close(deb_fh); return(1); }

  u_str = (unsigned char*)u_b1s;  i=read(deb_fh, u_str,   128 * 2);  if (i<  128 * 2) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b2s;  i=read(deb_fh, u_str,  1920 * 2);  if (i< 1920 * 2) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b3s;  i=read(deb_fh, u_str,  2048 * 2);  if (i< 2048 * 2) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b4s;  i=read(deb_fh, u_str, 49152 * 2);  if (i<49152 * 2) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b5s;  i=read(deb_fh, u_str,  2048 * 2);  if (i< 2048 * 2) { close(deb_fh); return(1); }
  u_str = (unsigned char*)u_b6s;  i=read(deb_fh, u_str,  8192 * 2);  if (i< 8192 * 2) { close(deb_fh); return(1); }

  close(deb_fh);

  return(0);
}

static int deb_set_dot(int x1, int y1, unsigned char r, unsigned char g, unsigned char b)
{
	int       k;
        uint32_t *pixels;

        if ((x1<0)||(x1>=stream_open_is->width)) return(0);
        if ((y1<0)||(y1>=stream_open_is->height)) return(0);

	k = y1*stream_open_is->width + x1;
        pixels=deb_tx_pixels+k;
        *pixels = (r << 16) + (g << 8) + b;
        
	return(0);
}

static int u_get_char_bmp(unsigned char c1,unsigned char c2,unsigned char c3)
{
    int k,l,m,n;
    int i1;
    
    u_err=0;
    u_err2=0;
    
    u_ptr=0;
    u_cptr=0;
    u_nb=1;

    u_char_size_x=7;
    u_char_size_y=14;
    
    if (c1<128)
    {
      u_ptr = 0; u_nb = 1; u_n1 = 0; u_cptr= c1;
      if ((u_cptr<0)||(u_cptr>=128)) u_err=1;
    }
    else if ((c1>=194)&&(c1<=223))
    {
      u_ptr = 1; u_nb = 2; u_n1 = 194; u_n2 = 128; u_cptr= (c1-u_n1)*64+c2-u_n2;
      if ((u_cptr<0)||(u_cptr>=1920)) u_err=1;
    }
    else if (c1==224)
    {
      u_ptr = 2; u_nb = 3; u_n1 = 224; u_n2 = 160; u_n3 = 128; u_cptr=(c1-u_n1)*32*64+(c2-u_n2)*64+c3-u_n3;
      if ((u_cptr<0)||(u_cptr>=2048)) u_err=1;
    }
    else if ((c1>=225)&&(c1<=236))
    {
      u_ptr = 3; u_nb = 3; u_n1 = 225; u_n2 = 128; u_n3 = 128; u_cptr=(c1-u_n1)*64*64+(c2-u_n2)*64+c3-u_n3;
      if ((u_cptr<0)||(u_cptr>=49152)) u_err=1;
    }
    else if (c1==237)
    {
      u_ptr = 4; u_nb = 3; u_n1 = 237; u_n2 = 128; u_n3 = 128; u_cptr=(c1-u_n1)*32*64+(c2-u_n2)*64+c3-u_n3;
      if ((u_cptr<0)||(u_cptr>=2048)) u_err=1;
    }
    else if ((c1>=238)&&(c1<=239))
    {
      u_ptr = 5; u_nb = 3; u_n1 = 238; u_n2 = 128; u_n3 = 128; u_cptr=(c1-u_n1)*64*64+(c2-u_n2)*64+c3-u_n3;
      if ((u_cptr<0)||(u_cptr>=8192)) u_err=1;
    }
    else if ((c1>=240)&&(c1<=244))
    {
      u_nb = 4;
      u_char_size_x=14;
      u_err2=1;
    }
    else
    {
      u_nb = 1;
      u_err=1;
    }
    
    if (u_err==0) 
    {
      if (u_err2==0)
      {
        k=0;
        l=0;
        
        if (u_ptr==0) { k=u_b1s[u_cptr][0]; l=u_b1s[u_cptr][1]; }
        if (u_ptr==1) { k=u_b2s[u_cptr][0]; l=u_b2s[u_cptr][1]; }
        if (u_ptr==2) { k=u_b3s[u_cptr][0]; l=u_b3s[u_cptr][1]; }
        if (u_ptr==3) { k=u_b4s[u_cptr][0]; l=u_b4s[u_cptr][1]; }
        if (u_ptr==4) { k=u_b5s[u_cptr][0]; l=u_b5s[u_cptr][1]; }
        if (u_ptr==5) { k=u_b6s[u_cptr][0]; l=u_b6s[u_cptr][1]; }
        
        if (k<7)  k=7;
        if (k>14) k=14;
        if (l<14) l=14;
        if (l>14) l=14;
        
        u_char_size_x=k;
        u_char_size_y=l;
    
        for (m=0;m<k;m++)
        {
          for (n=0;n<l;n++)
          {
            if ((m<0)||(m>=14)) continue;
            if ((n<0)||(n>=14)) continue;
        
            i1=255;
            
            if (u_ptr==0) i1=u_b1p[u_cptr][m][n];
            if (u_ptr==1) i1=u_b2p[u_cptr][m][n];
            if (u_ptr==2) i1=u_b3p[u_cptr][m][n];
            if (u_ptr==3) i1=u_b4p[u_cptr][m][n];
            if (u_ptr==4) i1=u_b5p[u_cptr][m][n];
            if (u_ptr==5) i1=u_b6p[u_cptr][m][n];
        
            u_char_bmp[m][n]=i1;
          }
        }
    
      }
    }
    
    return(0);
}

static int deb_echo_str4screenstring(int xx,int yy,const char *str,int str_size,int st,int vid)
{
  int			i,j,m,n;
  int			x,y;
  unsigned char		c1,c2,c3;
  int  			i1;
  int			len;

  if (deb_str_has_null(str,str_size)!=1) return(0);

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);

  len=strlen(str);
  x=xx;
  y=yy;
  i=len;
  j=0;

  while(j<i)
  {
    if (strlen(str)>j+0) c1=(unsigned char)str[j+0];
    else c1=0;
    
    if (strlen(str)>j+1) c2=(unsigned char)str[j+1];
    else c2=0;
    
    if (strlen(str)>j+2) c3=(unsigned char)str[j+2];
    else c3=0;
    
    u_get_char_bmp(c1,c2,c3);
    
    if ((u_err==0)&&(u_err2==0))
    {
        for (m=0;m<u_char_size_x;m++)
        {
          for (n=0;n<u_char_size_y;n++)
          {
            if ((m<0)||(m>=14)) continue;
            if ((n<0)||(n>=14)) continue;
        
            i1=u_char_bmp[m][n];
            
            if (st==0)  // normal
            {
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, i1, i1, i1, i1);

                  fill_rectangle(x+m, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m,y+n,i1,i1,i1);
                }
            }
            else if (st==1) //black
            {
              deb_set_dot(x+m,y+n,255-i1,255-i1,255-i1);
            }
            else  // green
            {
              deb_set_dot(x+m,y+n,0,255-i1,0);
            }
          }
        }
    
        x=x+u_char_size_x;
        j=j+u_nb;
        continue;
    }
    else
    {
        x=x+u_char_size_x;
        j=j+u_nb;
        continue;
    }
  }
        
  return(0);
}

static int u_strlen(char *str,int str_size)
{
  int			i,j;
  int			x;
  unsigned char		c1,c2,c3;
  int                   len;

  if (deb_str_has_null(str,str_size)!=1) return(0);

  len=strlen(str);
  i=len;
  j=0;
  x=0;

  while(j<i)
  {
    if (strlen(str)>j+0) c1=(unsigned char)str[j+0];
    else c1=0;
    
    if (strlen(str)>j+1) c2=(unsigned char)str[j+1];
    else c2=0;
    
    if (strlen(str)>j+2) c3=(unsigned char)str[j+2];
    else c3=0;
    
    u_get_char_bmp(c1,c2,c3);
    
    if ((u_err==0)&&(u_err2==0))
    {
        x=x+u_char_size_x;
        j=j+u_nb;
        continue;
    }
    else
    {
        x=x+u_char_size_x;
        j=j+u_nb;
        continue;
    }
  }
        
  return(x/7);
}

char m81_str[1500][5];
int  m81_len[1500];
int  m81_size[1500];

static int u_strcut(char *instr,int instr_size,char *outstr,int outstr_size,int fldlen,int st)
{
  int				i,j,m,n,o,p,q;
  int				x;
  unsigned char		c1,c2,c3,c4;
  int                   len;
  int                   sptr,mi;

  if (deb_str_has_null(instr,instr_size)!=1) return(0);

  len=strlen(instr);
  i=len;
  j=0;
  x=0;
  sptr=0;

  while(j<i)
  {
    if (strlen(instr)>j+0) c1=(unsigned char)instr[j+0];
    else c1=0;
    
    if (strlen(instr)>j+1) c2=(unsigned char)instr[j+1];
    else c2=0;
    
    if (strlen(instr)>j+2) c3=(unsigned char)instr[j+2];
    else c3=0;
    
    u_get_char_bmp(c1,c2,c3);

    if (u_err==0)
    {
      if (u_err2==0)
      {
        if (u_nb==1)
        {
          m81_str[sptr][0]=c1;
          m81_str[sptr][1]=0;
          m81_len[sptr]   =1;
          m81_size[sptr]  =1;
        }
        else if (u_nb==2)
        {
          m81_str[sptr][0]=c1;
          m81_str[sptr][1]=c2;
          m81_str[sptr][2]=0;
          m81_len[sptr]   =u_char_size_x/7;
          m81_size[sptr]  =2;
        }
        else if (u_nb==3)
        {
          m81_str[sptr][0]=c1;
          m81_str[sptr][1]=c2;
          m81_str[sptr][2]=c3;
          m81_str[sptr][3]=0;
          m81_len[sptr]   =u_char_size_x/7;
          m81_size[sptr]  =3;
        }
    
        sptr++;
        x=x+u_char_size_x;
        j=j+u_nb;
        continue;
      }
      else
      {
        m81_str[sptr][0]=c1;
        m81_str[sptr][1]=c2;
        m81_str[sptr][2]=c3;
        m81_str[sptr][3]=c4;
        m81_str[sptr][4]=0;
        m81_len[sptr]   =14/7;
        m81_size[sptr]=4;

        sptr++;
        x=x+14;
        j=j+u_nb;
        continue;
      }
    }
    else
    {
      m81_str[sptr][0]=c1;
      m81_str[sptr][1]=0;
      m81_len[sptr]   =7/7;
      m81_size[sptr]  =1;

      sptr++;
      x=x+7;
      j=j+u_nb;
      continue;
    }
  }

  n=0;
  p=0;
  mi=0;
  
  q=u_strlen(instr,instr_size);
  if (q>fldlen) mi=1;

  for (m=0;m<sptr;m++)
  {
    if (mi==1)
    {
      if (m81_len[m]+n>=fldlen)
      {
        break;
      }
    }
    else
    {
      if (m81_len[m]+n>fldlen)
      {
        break;
      }
    }
    
    if (m81_size[m]+p>=outstr_size-1)
    {
      break;
    }

    n=n+m81_len[m];
    p=p+m81_size[m];
  }

  outstr[0]=0;

  for (o=0;o<m;o++) strcat(outstr,m81_str[o]);

  if (st==0)
  {
    if (mi!=0)
    {
      if (p+1<outstr_size-1)
      {
        strcat(outstr,"-");
        p++;
      }
    }

    for (o=n+mi;o<fldlen;o++)
    {
      if (p+1<outstr_size-1)
      {
        strcat(outstr," ");
        p++;
      }
      else break;
    }
  }

  return(0);
}

static int u_test(void)
{
  int   n1,n1a,n2,n2a,n3,n3a/*,n4*/;
  int   i,j;
  int   lp;
  unsigned char sn[10];
  char *str;
  int   page_ptr;

  for (i=0;i<64;i++)
  {
    for (j=0;j<300;j++)
    {
      u_test_buf[i][j]=0;
    }
  }

  page_ptr=0;
  
  for (n1=0;n1<4;n1++)
  {
    for (n1a=0;n1a<32;n1a++)
    {
      sn[0]=n1*32+n1a;
      sn[1]=0;
      str=(char *)sn;
      
      if (u_test_ptr==page_ptr) strcat(u_test_buf[n1],str);
    }
  }

  if (u_test_ptr==page_ptr)
  {
    u_test_disp_win();
    u_test_disp_ter();
    return(0);
  }
  
  page_ptr=1;
  lp=0;
  
  for (n1=194;n1<=223;n1=n1+16)
  {
    for (n1a=0;n1a<16;n1a++)
    {
      if ((n1>=210)&&(n1a>=14)) break;
      
      for (n2=4;n2<6;n2++)
      {
        for (n2a=0;n2a<32;n2a++)
        {
          sn[0]=n1+n1a;
          sn[1]=n2*32+n2a;
          sn[2]=0;
          str=(char *)sn;
          
          if (u_test_ptr==page_ptr) strcat(u_test_buf[lp],str);
        }
        
        if (u_test_ptr==page_ptr) lp++;
      }
    }

    if (u_test_ptr==page_ptr)
    {
      u_test_disp_win();
      u_test_disp_ter();
      return(0);
    }
    
    page_ptr++;
  }
  
  page_ptr=3;
  lp=0;
  n1=224;
  
  for (n2=160;n2<=191;n2=n2+16)
  {
    for (n2a=0;n2a<16;n2a++)
    {
      for (n3=4;n3<6;n3++)
      {
        for (n3a=0;n3a<32;n3a++)
        {
          sn[0]=n1;
          sn[1]=n2+n2a;
          sn[2]=n3*32+n3a;
          sn[3]=0;
          str=(char *)sn;
          
          if (u_test_ptr==page_ptr) strcat(u_test_buf[lp],str);
        }
      
        if (u_test_ptr==page_ptr) lp++;
      }
    }

    if (u_test_ptr==page_ptr)
    {
      u_test_disp_win();
      u_test_disp_ter();
      return(0);
    }
    
    page_ptr++;
  }

  page_ptr=5;
  lp=0;  
  
  for (n1=225;n1<=236;n1++)
  {
    for (n2=128;n2<=191;n2=n2+16)
    {
      for (n2a=0;n2a<16;n2a++)
      {
        for (n3=4;n3<6;n3++)
        {
          for (n3a=0;n3a<32;n3a++)
          {
            sn[0]=n1;
            sn[1]=n2+n2a;
            sn[2]=n3*32+n3a;
            sn[3]=0;
            str=(char *)sn;
          
            if (u_test_ptr==page_ptr) strcat(u_test_buf[lp],str);
          }
      
          if (u_test_ptr==page_ptr) lp++;
        }
      }

      if (u_test_ptr==page_ptr)
      {
        u_test_disp_win();
        u_test_disp_ter();
        return(0);
      }
    
      page_ptr++;
    }
  }

  page_ptr=53;
  lp=0;  
  
  for (n1=237;n1<=237;n1++)
  {
    for (n2=128;n2<=159;n2=n2+16)
    {
      for (n2a=0;n2a<16;n2a++)
      {
        for (n3=4;n3<6;n3++)
        {
          for (n3a=0;n3a<32;n3a++)
          {
            sn[0]=n1;
            sn[1]=n2+n2a;
            sn[2]=n3*32+n3a;
            sn[3]=0;
            str=(char *)sn;
          
            if (u_test_ptr==page_ptr) strcat(u_test_buf[lp],str);
          }
      
          if (u_test_ptr==page_ptr) lp++;
        }
      }

      if (u_test_ptr==page_ptr)
      {
        u_test_disp_win();
        u_test_disp_ter();
        return(0);
      }
    
      page_ptr++;
    }
  }

  page_ptr=55;
  lp=0;  
  
  for (n1=238;n1<=239;n1++)
  {
    for (n2=128;n2<=191;n2=n2+16)
    {
      for (n2a=0;n2a<16;n2a++)
      {
        for (n3=4;n3<6;n3++)
        {
          for (n3a=0;n3a<32;n3a++)
          {
            sn[0]=n1;
            sn[1]=n2+n2a;
            sn[2]=n3*32+n3a;
            sn[3]=0;
            str=(char *)sn;
          
            if (u_test_ptr==page_ptr) strcat(u_test_buf[lp],str);
          }
      
          if (u_test_ptr==page_ptr) lp++;
        }
      }

      if (u_test_ptr==page_ptr)
      {
        u_test_disp_win();
        u_test_disp_ter();
        return(0);
      }
    
      page_ptr++;
    }
  }
  
  return(0);
}

static int u_test_disp_win(void)
{
    int i;
    
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);

    for (i=0;i<32;i++)
    {
      deb_echo_str4screenstring(0,14*i,u_test_buf[i],300,0,1);
    }

    SDL_RenderPresent(renderer);
    
    return(0);
}

static int u_test_disp_ter(void)
{
    int i;

    if (u_test_ptr==0)
    {
      for (i=0;i<32;i++) u_test_buf[0][i]=' ';
      
      u_test_buf[3][31]=' ';
    }
    
    for (i=0;i<32;i++)
    {
      printf("%s\n",u_test_buf[i]);
    }
    
    printf("page %d,\n",u_test_ptr);
    
    return(0);
}

// daipozhi modified 
static int deb_get_dir_ini(void)
{
  int           i;
  long long int j;

  for (i=0;i<MAX_FILE_NUM;i++) deb_filenamebuff[i][0]=0;

  j=(long long int)getcwd(deb_currentpath,FN_SIZE);
  if (j==0) return(1);

  deb_filenamebuff_n=0;
  deb_filenameplay=0;
  deb_filenamebuff_ptr=0;

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"C:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"D:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"E:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"F:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"G:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"H:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"I:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"J:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"K:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamebuff_ptr],"L:/");
  //deb_filenamebuffptr[deb_filenamebuff_ptr]=2;
  deb_filenamebuff_type[deb_filenamebuff_ptr]=0;
  deb_filenamebuff_icon[deb_filenamebuff_ptr]=1;
  deb_filenamebuff_len[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_len2[deb_filenamebuff_ptr]=30;
  deb_filenamebuff_ptr++;
  if (deb_filenamebuff_ptr>=MAX_FILE_NUM) return(0);

  return(0);
}

static int deb_dir_opened(int ptr )
{
  int i,j;

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamebuff_ptr) return(0);

  while ((deb_filenamebuff_subline[ptr+1]==1)&&(ptr+1<deb_filenamebuff_ptr))
  {
    ptr++;
  }

  i=deb_get_space(ptr);

  if (ptr+1>=deb_filenamebuff_ptr) return(0);
  else
  {
    j=deb_get_space(ptr+1);
    if (j>i) return(1);
    else return(0);
  }
}

static int deb_get_space(int ptr)
{
  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamebuff_ptr) return(0);

  return(deb_filenamebuff_space[ptr]);
}

static int deb_get_space2(int ptr)
{
  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamebuff2_ptr) return(0);

  return(deb_filenamebuff2_space[ptr]);
}

//static char m101_s1[3000];
//static char m101_s2[100];

static int deb_dir_add_after(int ptr)
{
  int i,k,l,w;
  int leftspace,dirlen,filelen;

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamebuff_ptr) return(0);

  ptr++;
  
  while ((deb_filenamebuff_subline[ptr]==1)&&(ptr<deb_filenamebuff_ptr))
  {
    ptr++;
  }

  ptr--;


    w = (g_dirview_posi[0][2]-3-4) /*cur_stream->width*/  /deb_ch_w;
    leftspace=deb_get_space(ptr);
    dirlen=deb_m_info_len;

    if (leftspace+dirlen+36<=w-2-2-2)
    {
      filelen=dirlen;
    }
    else
    {
      filelen=w-2-2-2-36-leftspace;
    }



    deb_filenamebuff2_ptr=0;
    
    for (i=0;i<bt_out_ptr;i++)
    {
        if (i<0) continue;
        if (i>=BTREE1_SIZE) continue;

        if (deb_str_has_null(bt_out_buff[i],FN_SIZE)!=1) continue;

	ar_conv(bt_out_buff[i],FN_SIZE,filelen);

	if (deb_str_has_null(ar_buff4[0],FN_SIZE)!=1) continue;

      	strcpy(deb_filenamebuff2[deb_filenamebuff2_ptr],ar_buff4[0]);
        
      	strcpy(deb_filenamebuff2_ext[ deb_filenamebuff2_ptr],bt_out_buff3[i]);
      	strcpy(deb_filenamebuff2_size[deb_filenamebuff2_ptr],bt_out_buff4[i]);
      	strcpy(deb_filenamebuff2_date[deb_filenamebuff2_ptr],bt_out_buff5[i]);

	deb_filenamebuff2_len[    deb_filenamebuff2_ptr]=filelen;
	deb_filenamebuff2_len2[   deb_filenamebuff2_ptr]=deb_m_info_len;
	deb_filenamebuff2_type[   deb_filenamebuff2_ptr]=bt_out_buff2[i];
	deb_filenamebuff2_icon[   deb_filenamebuff2_ptr]=bt_out_buff6[i];
	deb_filenamebuff2_space[  deb_filenamebuff2_ptr]=leftspace+2;
	deb_filenamebuff2_subline[deb_filenamebuff2_ptr]=0;
	
	deb_filenamebuff2_ptr++;
	if (deb_filenamebuff2_ptr>=MAX_FILE_NUM) deb_filenamebuff2_ptr=MAX_FILE_NUM-1;
	
	for (k=1;k<ar_buff4_ptr;k++)
	{
	    if (deb_str_has_null(ar_buff4[k],FN_SIZE)!=1) continue;
	
      	    strcpy(deb_filenamebuff2[deb_filenamebuff2_ptr],ar_buff4[k]);
        
      	    deb_filenamebuff2_ext[ deb_filenamebuff2_ptr][0]=0;
      	    deb_filenamebuff2_size[deb_filenamebuff2_ptr][0]=0;
      	    deb_filenamebuff2_date[deb_filenamebuff2_ptr][0]=0;

	    deb_filenamebuff2_len[    deb_filenamebuff2_ptr]=filelen;
	    deb_filenamebuff2_len2[   deb_filenamebuff2_ptr]=deb_m_info_len;
	    deb_filenamebuff2_type[   deb_filenamebuff2_ptr]=bt_out_buff2[i];
	    deb_filenamebuff2_icon[   deb_filenamebuff2_ptr]=0;
	    deb_filenamebuff2_space[  deb_filenamebuff2_ptr]=leftspace+2;
	    deb_filenamebuff2_subline[deb_filenamebuff2_ptr]=1;
	
	    deb_filenamebuff2_ptr++;
	    if (deb_filenamebuff2_ptr>=MAX_FILE_NUM) deb_filenamebuff2_ptr=MAX_FILE_NUM-1;
	}
    }



  if (deb_filenamebuff2_ptr>0)
  {
    l=2;

    for (i=deb_filenamebuff_ptr-1;i>ptr;i--)
    {
        if (i<0) continue;
        if (i>=MAX_FILE_NUM) continue;

        if (i+deb_filenamebuff2_ptr<0) continue;
        if (i+deb_filenamebuff2_ptr>=MAX_FILE_NUM) continue;

        if (deb_str_has_null(deb_filenamebuff[i],FN_SIZE)!=1) continue;

        //if (strlen(deb_filenamebuff[i])>=FN_SIZE) continue;

      	strcpy(deb_filenamebuff[i+deb_filenamebuff2_ptr],deb_filenamebuff[i]);

      	strcpy(deb_filenamebuff_ext[ i+deb_filenamebuff2_ptr],deb_filenamebuff_ext[ i]);
      	strcpy(deb_filenamebuff_size[i+deb_filenamebuff2_ptr],deb_filenamebuff_size[i]);
      	strcpy(deb_filenamebuff_date[i+deb_filenamebuff2_ptr],deb_filenamebuff_date[i]);

	deb_filenamebuff_len[ i+deb_filenamebuff2_ptr]=deb_filenamebuff_len[ i];
	deb_filenamebuff_len2[i+deb_filenamebuff2_ptr]=deb_filenamebuff_len2[i];
	deb_filenamebuff_type[i+deb_filenamebuff2_ptr]=deb_filenamebuff_type[i];
	deb_filenamebuff_icon[i+deb_filenamebuff2_ptr]=deb_filenamebuff_icon[i];
	deb_filenamebuff_space[i+deb_filenamebuff2_ptr]=deb_filenamebuff_space[i];
	deb_filenamebuff_subline[i+deb_filenamebuff2_ptr]=deb_filenamebuff_subline[i];

      	deb_filenamebuff[i][0]=0;

	deb_filenamebuff_ext[i][0]=0;
	deb_filenamebuff_size[i][0]=0;
	deb_filenamebuff_date[i][0]=0;

	deb_filenamebuff_len[i]=0;
	deb_filenamebuff_len2[i]=0;
	deb_filenamebuff_type[i]=0;
	deb_filenamebuff_icon[i]=0;
	deb_filenamebuff_space[i]=0;
	deb_filenamebuff_subline[i]=0;
    }

    for (i=0;i<deb_filenamebuff2_ptr;i++)
    {
        if (i<0) continue;
        if (i>=MAX_FILE_NUM) continue;

        if (ptr+1+i<0) continue;
        if (ptr+1+i>=MAX_FILE_NUM) continue;

        if (deb_str_has_null(deb_filenamebuff2[i],FN_SIZE)!=1) continue;
        //if (deb_str_has_null(m101_s1,3000)!=1) continue;
        //if (deb_str_has_null(m101_s2,100)!=1) continue;

        //if (strlen(bt_out_buff[i])>=FN_SIZE) continue;

      	strcpy(deb_filenamebuff[ptr+1+i],deb_filenamebuff2[i]);
        
      	strcpy(deb_filenamebuff_ext[ ptr+1+i],deb_filenamebuff2_ext[i]);
      	strcpy(deb_filenamebuff_size[ptr+1+i],deb_filenamebuff2_size[i]);
      	strcpy(deb_filenamebuff_date[ptr+1+i],deb_filenamebuff2_date[i]);

	deb_filenamebuff_len[ ptr+1+i]=deb_filenamebuff2_len[i];
	deb_filenamebuff_len2[ptr+1+i]=deb_filenamebuff2_len2[i];
	deb_filenamebuff_type[ptr+1+i]=deb_filenamebuff2_type[i];
	deb_filenamebuff_icon[ptr+1+i]=deb_filenamebuff2_icon[i];
	deb_filenamebuff_space[ptr+1+i]=deb_filenamebuff2_space[i];
	deb_filenamebuff_subline[ptr+1+i]=deb_filenamebuff2_subline[i];
    }
    
    deb_filenamebuff_ptr=deb_filenamebuff_ptr+deb_filenamebuff2_ptr;
    if (deb_filenamebuff_ptr>=MAX_FILE_NUM) deb_filenamebuff_ptr=MAX_FILE_NUM-1;

    if (ptr>=deb_filenameplay)
    {
    }
    else
    {
      deb_filenameplay=deb_filenameplay+deb_filenamebuff2_ptr;
    }
  }
  else
  {
    l=2;

    for (i=deb_filenamebuff_ptr-1;i>ptr;i--)
    {
        if (i<0) continue;
        if (i>=MAX_FILE_NUM) continue;

        if (i+1<0) continue;
        if (i+1>=MAX_FILE_NUM) continue;

        if (deb_str_has_null(deb_filenamebuff[i],FN_SIZE)!=1) continue;

        //if (strlen(deb_filenamebuff[i])>=FN_SIZE) continue;

      	strcpy(deb_filenamebuff[i+1],deb_filenamebuff[i]);

      	strcpy(deb_filenamebuff_ext[ i+1],deb_filenamebuff_ext[ i]);
      	strcpy(deb_filenamebuff_size[i+1],deb_filenamebuff_size[i]);
      	strcpy(deb_filenamebuff_date[i+1],deb_filenamebuff_date[i]);

	deb_filenamebuff_len[ i+1]=deb_filenamebuff_len[ i];
	deb_filenamebuff_len2[i+1]=deb_filenamebuff_len2[i];
	deb_filenamebuff_type[i+1]=deb_filenamebuff_type[i];
	deb_filenamebuff_icon[i+1]=deb_filenamebuff_icon[i];
	deb_filenamebuff_space[i+1]=deb_filenamebuff_space[i];
	deb_filenamebuff_subline[i+1]=deb_filenamebuff_subline[i];

      	deb_filenamebuff[i][0]=0;

	deb_filenamebuff_ext[ i][0]=0;
	deb_filenamebuff_size[i][0]=0;
	deb_filenamebuff_date[i][0]=0;

	deb_filenamebuff_len[i]=0;
	deb_filenamebuff_len2[i]=0;
	deb_filenamebuff_type[i]=0;
	deb_filenamebuff_icon[i]=0;
	deb_filenamebuff_space[i]=0;
	deb_filenamebuff_subline[i]=0;
    }

    strcpy(deb_filenamebuff[ptr+1],"Empty Fold");

    strcpy(deb_filenamebuff_ext[ptr+1],"    ");
    strcpy(deb_filenamebuff_size[ptr+1],"      ");
    strcpy(deb_filenamebuff_date[ptr+1],"                    ");
    
    deb_filenamebuff_len[ptr+1] =12;
    deb_filenamebuff_len2[ptr+1] =12;
    deb_filenamebuff_type[ptr+1]=2;
    deb_filenamebuff_icon[ptr+1]=0;
    deb_filenamebuff_space[ptr+1]=leftspace+l;
    deb_filenamebuff_subline[ptr+1]=0;

    deb_filenamebuff_ptr=deb_filenamebuff_ptr+1;
    if (deb_filenamebuff_ptr>=MAX_FILE_NUM) deb_filenamebuff_ptr=MAX_FILE_NUM-1;

    if (ptr>=deb_filenameplay)
    {
    }
    else
    {
      deb_filenameplay=deb_filenameplay+1;
    }
  }

  return(0);
}

static int deb_dir_remove_after(int ptr)
{
  int i,j,k,l;
  int p1;

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamebuff_ptr) return(0);

  ptr++;
  
  while ((deb_filenamebuff_subline[ptr]==1)&&(ptr<deb_filenamebuff_ptr))
  {
    ptr++;
  }

  ptr--;

  i=deb_get_space(ptr);

  k=0;

  for (j=ptr+1;j<deb_filenamebuff_ptr;j++)
  {
    if (j<0) continue;
    if (j>=MAX_FILE_NUM) continue;

    l=deb_get_space(j);
    if (l<=i)
    {
      k=1;
      p1=j;
      break;
    }
  }

  if (k==0)
  {
    for (j=ptr+1;j<deb_filenamebuff_ptr;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_len2[j]=0;
      deb_filenamebuff_type[j]=0;
      deb_filenamebuff_icon[j]=0;
      deb_filenamebuff_space[j]=0;
      deb_filenamebuff_subline[j]=0;
    }

    deb_filenamebuff_ptr=ptr+1;

    deb_filenameplay=ptr;
  }
  else
  {
    for (j=p1;j<deb_filenamebuff_ptr;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      if (ptr+1+j-p1<0) continue;
      if (ptr+1+j-p1>=MAX_FILE_NUM) continue;

      if (deb_str_has_null(deb_filenamebuff[j],FN_SIZE)!=1) continue;

      //if (strlen(deb_filenamebuff[j])>=FN_SIZE) continue;

      strcpy(deb_filenamebuff[ptr+1+j-p1],deb_filenamebuff[j]);

      strcpy(deb_filenamebuff_ext[ptr+1+j-p1],deb_filenamebuff_ext[j]);
      strcpy(deb_filenamebuff_size[ptr+1+j-p1],deb_filenamebuff_size[j]);
      strcpy(deb_filenamebuff_date[ptr+1+j-p1],deb_filenamebuff_date[j]);

      deb_filenamebuff_len[ptr+1+j-p1] =deb_filenamebuff_len[j];
      deb_filenamebuff_len2[ptr+1+j-p1] =deb_filenamebuff_len2[j];
      deb_filenamebuff_type[ptr+1+j-p1]=deb_filenamebuff_type[j];
      deb_filenamebuff_icon[ptr+1+j-p1]=deb_filenamebuff_icon[j];
      deb_filenamebuff_space[ptr+1+j-p1]=deb_filenamebuff_space[j];
      deb_filenamebuff_subline[ptr+1+j-p1]=deb_filenamebuff_subline[j];

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_len2[j]=0;
      deb_filenamebuff_type[j]=0;
      deb_filenamebuff_icon[j]=0;
      deb_filenamebuff_space[j]=0;
      deb_filenamebuff_subline[j]=0;
    }

    for (j=ptr+deb_filenamebuff_ptr-p1+1;j<p1;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_len2[j]=0;
      deb_filenamebuff_type[j]=0;
      deb_filenamebuff_icon[j]=0;
      deb_filenamebuff_space[j]=0;
      deb_filenamebuff_subline[j]=0;
    }

    deb_filenamebuff_ptr=deb_filenamebuff_ptr-(p1-ptr-1);

    if ((ptr>=deb_filenameplay)&&(p1>=deb_filenameplay))
    {
    }
    else
    {
      if ((ptr<deb_filenameplay)&&(p1<=deb_filenameplay))
      {
        deb_filenameplay=deb_filenameplay-(p1-ptr-1);
      }
      else
      {
        deb_filenameplay=ptr;
      }
    }
  }

  return(0);
}

static char m5_buffer1[3000];
static char m5_buffer2[3000];
static char m5_buffer3[3000];
static char m5_buffer4[3000];

static int deb_get_path(int ptr)
{
  int  ns1,ns2;
  int  p1;
  int  i;

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamebuff_ptr) return(0);

  p1=ptr;

  m5_buffer4[0]=0;

  deb_dir_buffer[0]=0;

  ns1=deb_get_space(ptr);

  while (p1>=0)
  {
    if (ns1<2)
    {
      strcpy(m5_buffer1,deb_filenamebuff[p1]);
        
      i=p1+1;
      while ((deb_filenamebuff_subline[i]==1)&&(i<deb_filenamebuff_ptr))
      {
	if (strlen(m5_buffer1)+strlen(deb_filenamebuff[i])>=FN_SIZE) break;
		    
        strcat(m5_buffer1,deb_filenamebuff[i]);
        i++;
      }
        
      deb_get_path2(m5_buffer1,m5_buffer2);
      strcpy(m5_buffer3,m5_buffer2);
    }
    else
    {
      if (ns1<4)
      {
        strcpy(m5_buffer1,deb_filenamebuff[p1]);
        
        i=p1+1;
        while ((deb_filenamebuff_subline[i]==1)&&(i<deb_filenamebuff_ptr))
        {
	  if (strlen(m5_buffer1)+strlen(deb_filenamebuff[i])>=FN_SIZE) break;
		    
          strcat(m5_buffer1,deb_filenamebuff[i]);
          i++;
        }
        
        deb_get_path2(m5_buffer1,m5_buffer2);
        strcpy(m5_buffer3,m5_buffer2);
      }
      else
      {
        strcpy(m5_buffer1,deb_filenamebuff[p1]);
        
        i=p1+1;
        while ((deb_filenamebuff_subline[i]==1)&&(i<deb_filenamebuff_ptr))
        {
	  if (strlen(m5_buffer1)+strlen(deb_filenamebuff[i])>=FN_SIZE) break;
		    
          strcat(m5_buffer1,deb_filenamebuff[i]);
          i++;
        }
        
        deb_get_path1(m5_buffer1,m5_buffer2);
        strcpy(m5_buffer3,m5_buffer2);
      }
    }

    strcat(m5_buffer3,m5_buffer4);
    strcpy(m5_buffer4,m5_buffer3);

    while(1)
    {
      p1--;
      if (p1<0) break;
      ns2=deb_get_space(p1);
	
      if (ns2>=ns1) continue;
      else
      {
        ns1=ns2;
        
        while ((deb_filenamebuff_subline[p1]==1)&&(p1>0))
        {
          p1--;
        }
        
        break;
      }
    }
  }

  strcpy(deb_dir_buffer,m5_buffer4);
  
  return(0);
}

static char m102_buffer3[3000];
static char m102_buffer4[3000];

static int deb_get_path1(char *buffer1,char *buffer2)
{
  //int  i,j,k;

  strcpy(m102_buffer3,buffer1);
  strcpy(m102_buffer4,m102_buffer3);
/*
  j=0;
  k=0;

  for (i=0;i<(int)strlen(m102_buffer3);i++)
  {
    if (m102_buffer3[i]==' ')
    {
      if (k==0)
      {
        continue;
      }
      else
      {
        m102_buffer4[j+0]=m102_buffer3[i];
        m102_buffer4[j+1]=0;

        j++;
      }
    }
    else
    {
      k=1;

      m102_buffer4[j+0]=m102_buffer3[i];
      m102_buffer4[j+1]=0;

      j++;
    }
  }
*/
  strcpy(buffer2,"/");
  strcat(buffer2,m102_buffer4);

  return(0);
}

static char m103_buffer3[3000];
static char m103_buffer4[3000];

static int deb_get_path2(char *buffer1,char *buffer2)
{
  //int  i,j,k;

  strcpy(m103_buffer3,buffer1);
  strcpy(m103_buffer4,m103_buffer3);
/*
  j=0;
  k=0;

  for (i=0;i<(int)strlen(m103_buffer3);i++)
  {
    if (m103_buffer3[i]==' ')
    {
      if (k==0) // 
      {
        continue;
      }
      else
      {
        m103_buffer4[j+0]=m103_buffer3[i];
        m103_buffer4[j+1]=0;

        j++;
      }
    }
    else
    {
      k=1;

      m103_buffer4[j+0]=m103_buffer3[i];
      m103_buffer4[j+1]=0;

      j++;
    }
  }
*/
  strcpy(buffer2,m103_buffer4);

  return(0);
}

static int deb_filename_ext(char *name,int name_size,char *fext,int fext_size)
{
  int  i,j,k,l;

  if (deb_str_has_null(name,name_size)!=1)
  {
    fext[0]=0;
    return(1);
  }
  
  i=(int)strlen(name);
  if (i<=0)
  {
    fext[0]=0;
    return(1);
  }
  //if (i>name_size) i=name_size;
  
  k=(-1);

  for (j=i-1;j>=0;j--)
  {
    if (name[j]=='.')
    {
      k=j;
      break;
    }
  }

  if (k<0)
  {
    fext[0]=0;
    return(1);
  }
  else
  {
    fext[0]=0;

    for (l=k+1;l<i;l++)
    {
      if (l-k-0>=fext_size) break;
      
      fext[l-k-1]=name[l];
      fext[l-k-0]=0;
    }
  }
	
  deb_lower_string(fext,fext_size);
	
  return(0);
}
/*
static int deb_filenameext2(char *path,int path_size,char *fext,int fext_size)
{
  int  i,j,k,l;
  struct stat info;

  if (deb_str_has_null(path,path_size)!=1)
  {
    fext[0]=0;
    return(0);
  }

  i=(int)strlen(path);
  if (i<=0)
  {
    fext[0]=0;
    return(0);
  }
  //if (i>path_size) i=path_size;
  
  k=(-1);

  for (j=i-1;j>=0;j--)
  {
    if (path[j]=='.')
    {
      k=j;
      break;
    }
  }

  stat(path,&info);

  if (!S_ISREG(info.st_mode))
  {
    fext[0]=0;
  }
  else
  {
    if (k<0)
    {
      fext[0]=0;
    }
    else
    {
      fext[0]=0;

      for (l=k+1;l<i;l++)
      {
        if (l-k-0>=fext_size) break;
        
        fext[l-k-1]=path[l];
        fext[l-k-0]=0;
      }
    }
  }

  return(0);
}
*/

static int deb_cmp_dir(char *buffer1,char *buffer2)
{ 
  int  i,j,k;
  char c1,c2;
  char str1[20],str2[20];

  i=(int)strlen(buffer1);
  j=(int)strlen(buffer2);
  if (i!=j) return(1);

  for (k=0;k<i;k++)
  {
    c1=buffer1[k];
    c2=buffer2[k];
    
    if (c1==c2) continue;
    else
    {
      str1[0]=c1;
      str1[1]=0;
    
      str2[0]=c2;
      str2[1]=0;
    
      if (  ((strcmp(str1,"/")==0)||(strcmp(str1,"\\")==0)) && 
            ((strcmp(str2,"/")==0)||(strcmp(str2,"\\")==0))  ) continue;
      else return(1);
    }

  }

  return(0);
}

static char m105_str1[3000];

static int deb_filename_dir(char *path,char *name)
{
  strcpy(m105_str1,path);
  av_strlcat(m105_str1,"/" ,3000);
  av_strlcat(m105_str1,name,3000);

#if !defined(_WIN32)
  stat(m105_str1,&deb_m_info);
#else
  stat(m105_str1,&deb_m_info);
#endif

  if (S_ISDIR(deb_m_info.st_mode)) return(1);
  else return(0);
}

static int cpy_string(char *p_outstr,int p_outstr_size,char *p_instr,int p_instr_size)
{
  int i;

  p_outstr[0]=0;

  for (i=0;i<p_instr_size;i++)
  {
    if (p_instr[i]==0) break;
    if ((i<0)||(i>=p_outstr_size)) continue;
    if ((i+1<0)||(i+1>=p_outstr_size)) continue;
    
    p_outstr[i+0]=p_instr[i];
    p_outstr[i+1]=0;
  }

  return(0);
}

static char  m501_str1[6000];
static char *m501_str1_ptr1[2];

static char  m501_str2[6000];
static char *m501_str2_ptr1[2];

iconv_t cd;

static char** in_buffer_tmp;
static int    in_buffer_tmp_len;   
static char** out_buffer_tmp;
static int    out_buffer_tmp_len;
  
static int deb_utf8_to_gb18030(char *in_buffer,char *out_buffer,int out_buffer_len)
{
    int i;
    //iconv_t cd;
    //char* in_buffer_tmp[]= { m501_str1,NULL };
    //int   in_buffer_tmp_len;   
    //char* out_buffer_tmp[]= { m501_str2,m501_str3,NULL };
    //int   out_buffer_tmp_len;
    // if these codes inside sub function , it may be bug

    cd = iconv_open("gb18030//TRANSLIT", "utf-8");  

    strcpy(m501_str1,in_buffer);  
    in_buffer_tmp_len=strlen(in_buffer);

    m501_str1_ptr1[0]=m501_str1;
    m501_str1_ptr1[1]=NULL;

    in_buffer_tmp=m501_str1_ptr1;

    for (i=0;i<6000;i++)
    {
      m501_str2[i]=0;
    }

    out_buffer_tmp_len = 6000;

    m501_str2_ptr1[0]=m501_str2;
    m501_str2_ptr1[1]=NULL;

    out_buffer_tmp=m501_str2_ptr1;

    iconv(cd, in_buffer_tmp, (size_t *)&in_buffer_tmp_len, out_buffer_tmp, (size_t *)&out_buffer_tmp_len);

    iconv_close(cd);

    cpy_string(out_buffer,out_buffer_len,m501_str2,5000);    

    return(0);  
}

static int deb_gb18030_to_utf8(char *in_buffer,char *out_buffer,int out_buffer_len)
{
    int i,j;
    
    cd = iconv_open("utf-8","gb18030//TRANSLIT");  
    
    strcpy(m501_str1,in_buffer);  
    in_buffer_tmp_len=strlen(in_buffer);

    m501_str1_ptr1[0]=m501_str1;
    m501_str1_ptr1[1]=NULL;

    in_buffer_tmp=m501_str1_ptr1;

    j=in_buffer_tmp_len*3+1;
    if (j<1)    j=1;
    if (j>6000) j=6000;

    for (i=0;i<j;i++)
    {
      m501_str2[i]=0;
    }
    out_buffer_tmp_len = 6000;

    m501_str2_ptr1[0]=m501_str2;
    m501_str2_ptr1[1]=NULL;

    out_buffer_tmp=m501_str2_ptr1;

    iconv(cd, in_buffer_tmp, (size_t *)&in_buffer_tmp_len, out_buffer_tmp, (size_t *)&out_buffer_tmp_len);  

    iconv_close(cd);  

    cpy_string(out_buffer,out_buffer_len,m501_str2,5000);    

    return(0);  
}

int string_trim_nos(char *pstr) // no space
{
  int i,j,k,l,m;

  i=(int)strlen(pstr);
  j=0;
  k=0;
  l=0;

  while (j<i)
  {
    if (pstr[j]<0)
    {
      j=j+2;
      l=0;
    }
    else
    {
      if (pstr[j]>' ')
      {
	j++;
        l=0;
      }
      else
      {
        if (l==0)
        {
          k=j;
          l=1;

          j++;
        }
        else j++;
      }
    }
  }

  if (l==1)
  {
    for (m=k;m<i;m++) pstr[m]=0;
  }

  return(0);
}

static char disp_buff[350][6000];
static int  disp_leftspace[350];
static int  disp_icon[350];
static char m11_str1[6000];
//static char m11_str2[6000];
//static char m11_str3[6000];
//static char m11_str4[6000];

static int deb_disp_dir(VideoState *cur_stream)
{
  int  i,j,k,l;
  int  h,w;
  int  leftspace,dirlen,filelen;
  int  n1,n4,n5,n6;
  char *strptr;
  int  icon;
  int  x,y;

  if ((screen_w<640)||(screen_w>7680)) return(1);
  if ((screen_h<480)||(screen_h>4320)) return(1);
  
  //start=(-1);

  for (n1=0;n1<350;n1++)
  {
    disp_buff[n1][0]=0;
    disp_leftspace[n1]=0;
    disp_icon[n1]=0;
  }

  x = g_dirview_posi[0][0];
  y = g_dirview_posi[0][1];
  h = g_dirview_posi[0][3] /*cur_stream->height*/ /deb_ch_h;
  w = (g_dirview_posi[0][2]-3-4) /*cur_stream->width*/  /deb_ch_w;

  if (h>=350 ) return(0);
  if (w>=6000) return(0);

  deb_ch_m = w;
      


  deb_tx_locked=0;

  if (realloc_texture(&cur_stream->vis_texture, SDL_PIXELFORMAT_ARGB8888, cur_stream->width, cur_stream->height, SDL_BLENDMODE_NONE, 1) < 0)
            return(0);

  cur_stream->ytop    = 0;
  cur_stream->xleft   = 0;

  deb_tx_rect.x = 0;
  deb_tx_rect.y = 0;
  deb_tx_rect.w = cur_stream->width; 
  deb_tx_rect.h = cur_stream->height;

  if (!SDL_LockTexture(cur_stream->vis_texture, &deb_tx_rect, (void **)&deb_tx_pixels, &deb_tx_pitch)) 
  {
                deb_tx_pitch >>= 2;
  }
  else return(0);

  deb_tx_locked=1;


  clr_rect_white();

  for (n4=0;n4<h;n4++)
  {
    if (deb_filenamebuff_n+n4<0) continue;
    if (deb_filenamebuff_n+n4>=MAX_FILE_NUM) continue;
    if (deb_filenamebuff_n+n4>=deb_filenamebuff_ptr) continue;

    if (deb_str_has_null(deb_filenamebuff[deb_filenamebuff_n+n4],FN_SIZE)!=1) continue;

    strcpy(m11_str1,deb_filenamebuff[deb_filenamebuff_n+n4]);

    if (deb_str_has_null(m11_str1,6000)!=1) continue;

    icon=deb_filenamebuff_icon[deb_filenamebuff_n+n4];

    leftspace=deb_get_space(deb_filenamebuff_n+n4);
    dirlen =deb_filenamebuff_len[deb_filenamebuff_n+n4];
    
    if (leftspace+dirlen+36<=w-2-2)
    {
      filelen=dirlen;
    }
    else
    {
      filelen=w-2-2-36-leftspace;
    }
    
    if (filelen<2) continue;
    
    disp_leftspace[n4]=leftspace;
    disp_icon[n4]=icon;
    
    strcpy(disp_buff[n4],m11_str1);
    
    for (l=u_strlen(m11_str1,6000);l<filelen;l++) strcat(disp_buff[n4]," ");

    if (deb_filenamebuff_subline[deb_filenamebuff_n+n4]==0)
    {
      strcat(disp_buff[n4],"  ");
      strcat(disp_buff[n4],deb_filenamebuff_ext[ deb_filenamebuff_n+n4]);
      strcat(disp_buff[n4],"  ");
      strcat(disp_buff[n4],deb_filenamebuff_size[deb_filenamebuff_n+n4]);
      strcat(disp_buff[n4],"  ");
      strcat(disp_buff[n4],deb_filenamebuff_date[deb_filenamebuff_n+n4]);
    }
  }

  //SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  //bgcolor = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);//daipozhi modified

  //fill_rectangle( 0, 0, cur_stream->width, deb_ch_h*h); // daipozhi modified 

  for (n4=0;n4<h;n4++)
  {
    if (n4>=350) continue;

    strptr=disp_buff[n4];
    n5=disp_leftspace[n4];
    n6=disp_icon[n4];
    
    if (deb_filenamebuff_subline[deb_filenamebuff_n+n4]==0)
    {
      g_paint_icon(x+3+n5*7,y+n4*deb_ch_h ,n6);
    }
    
    deb_echo_str4screenstring(x+3+n5*7+2*deb_ch_w+4 ,y+n4*deb_ch_h ,strptr ,6000,0,0);
  }

  i=deb_filenameplay+1;
  j=0;
  
  while ((deb_filenamebuff_subline[i]==1)&&(i<deb_filenamebuff_ptr))
  {
    j++;
    i++;
  }
  
  for (k=deb_filenameplay;k<=deb_filenameplay+j;k++)
  {
    n4=k-deb_filenamebuff_n;
    if ((n4>=0)&&(n4<h)&&(n4<350)) 
    {
        strptr=disp_buff[n4];
        n5=disp_leftspace[n4];
        n6=disp_icon[n4];
    
        //g_paint_icon(x+3+n5*7,y+n4*deb_ch_h ,n6);
    
        deb_echo_str4screenstring(x+3+n5*7+2*deb_ch_w+4 ,y+n4*deb_ch_h ,strptr ,6000,1,0);
    }
  }
  
  deb_m_ref=1;

  return(0);
}

// dir_view resize
static int deb_conv_dir(void)
{
    int  i,j,k,l,m,n,o,p,w;
    char str1[FN_SIZE];
    int  dirlen,filelen,leftspace;
    
    if ((screen_w<640)||(screen_w>7680)) return(1);
    if ((screen_h<480)||(screen_h>4320)) return(1);
      
    deb_filenamebuff2_ptr=0;
    w = (g_dirview_posi[0][2]-3-4) /*cur_stream->width*/  /deb_ch_w;
    i=0;
    
    while (i<deb_filenamebuff_ptr)
    {
        k=i;
        strcpy(str1,deb_filenamebuff[i]);

        leftspace=deb_get_space(i);
	dirlen =deb_filenamebuff_len2[i];
        
        if (leftspace+dirlen+36<=w-2-2)
        {
            filelen=dirlen;
        }
        else
        {
            filelen=w-2-2-36-leftspace;
        }

        i++;
        
        while ((deb_filenamebuff_subline[i]==1)&&(i<deb_filenamebuff_ptr))
        {
            if (strlen(str1)+strlen(deb_filenamebuff[i])>=FN_SIZE) break;
            
            strcat(str1,deb_filenamebuff[i]);
            i++;
        }

        if (filelen<2) continue;

	ar_conv(str1,FN_SIZE,filelen);
	
	if (deb_str_has_null(ar_buff4[0],FN_SIZE)!=1) continue;

      	strcpy(deb_filenamebuff2[deb_filenamebuff2_ptr],ar_buff4[0]);

      	strcpy(deb_filenamebuff2_ext[ deb_filenamebuff2_ptr],deb_filenamebuff_ext[k]);
      	strcpy(deb_filenamebuff2_size[deb_filenamebuff2_ptr],deb_filenamebuff_size[k]);
      	strcpy(deb_filenamebuff2_date[deb_filenamebuff2_ptr],deb_filenamebuff_date[k]);

	deb_filenamebuff2_len[    deb_filenamebuff2_ptr]=filelen;
	deb_filenamebuff2_len2[   deb_filenamebuff2_ptr]=deb_filenamebuff_len2[k];
	deb_filenamebuff2_type[   deb_filenamebuff2_ptr]=deb_filenamebuff_type[k];
	deb_filenamebuff2_icon[   deb_filenamebuff2_ptr]=deb_filenamebuff_icon[k];
	deb_filenamebuff2_space[  deb_filenamebuff2_ptr]=deb_filenamebuff_space[k];
	deb_filenamebuff2_subline[deb_filenamebuff2_ptr]=0;

	deb_filenamebuff2_ptr++;
	if (deb_filenamebuff2_ptr>=MAX_FILE_NUM) deb_filenamebuff2_ptr=MAX_FILE_NUM-1;
	
	for (l=1;l<ar_buff4_ptr;l++)
	{
	    if (deb_str_has_null(ar_buff4[l],FN_SIZE)!=1) continue;
	    
      	    strcpy(deb_filenamebuff2[deb_filenamebuff2_ptr],ar_buff4[l]);

      	    deb_filenamebuff2_ext[ deb_filenamebuff2_ptr][0]=0;
      	    deb_filenamebuff2_size[deb_filenamebuff2_ptr][0]=0;
      	    deb_filenamebuff2_date[deb_filenamebuff2_ptr][0]=0;

	    deb_filenamebuff2_len[    deb_filenamebuff2_ptr]=filelen;
	    deb_filenamebuff2_len2[   deb_filenamebuff2_ptr]=deb_filenamebuff_len2[k];
	    deb_filenamebuff2_type[   deb_filenamebuff2_ptr]=deb_filenamebuff_type[k];
	    deb_filenamebuff2_icon[   deb_filenamebuff2_ptr]=0;
	    deb_filenamebuff2_space[  deb_filenamebuff2_ptr]=deb_filenamebuff_space[k];
	    deb_filenamebuff2_subline[deb_filenamebuff2_ptr]=1;
	
	    deb_filenamebuff2_ptr++;
	    if (deb_filenamebuff2_ptr>=MAX_FILE_NUM) deb_filenamebuff2_ptr=MAX_FILE_NUM-1;
	}
    }
    
    
    // find new window top
    i=deb_filenamebuff_n;
    
    while ((deb_filenamebuff_subline[i]==1)&&(i>0))
    {
      i--;
    }
    
    deb_filenamebuff_n2=deb_filenamebuff_n-i;
    
    j=0;
    
    for (i=0;i<=deb_filenamebuff_n;i++)
    {
        if (deb_filenamebuff_subline[i]==0) j++;
    }
    
    k=0;
    
    for (i=0;i<deb_filenamebuff2_ptr;i++)
    {
        if (deb_filenamebuff2_subline[i]==0) k++;
        if (k>=j) break;
    }
        
    l=i;
    m=l+1;
    n=0;
    
    while ((deb_filenamebuff2_subline[m]==1)&&(m<deb_filenamebuff2_ptr))
    {
        n++;
        m++;
    }
    
    if (n>deb_filenamebuff_n2) o=deb_filenamebuff_n2;
    else o=n;
    
    deb_filenamebuff_n =l+o;
    deb_filenamebuff_n2=o;

    // find new play id
    i=deb_filenameplay;
    
    while ((deb_filenamebuff_subline[i]==1)&&(i>0))
    {
      i--;
    }
    
    j=0;
    
    for (i=0;i<=deb_filenameplay;i++)
    {
        if (deb_filenamebuff_subline[i]==0) j++;
    }
    
    k=0;
    
    for (i=0;i<deb_filenamebuff2_ptr;i++)
    {
        if (deb_filenamebuff2_subline[i]==0) k++;
        if (k>=j) break;
    }
        
    deb_filenameplay=i;
    
    //copy to main buff
    for (i=0;i<deb_filenamebuff2_ptr;i++)
    {
        if (deb_str_has_null(deb_filenamebuff2[i],FN_SIZE)!=1) continue;
        
      	strcpy(deb_filenamebuff[i],deb_filenamebuff2[i]);

      	strcpy(deb_filenamebuff_ext[ i],deb_filenamebuff2_ext[ i]);
      	strcpy(deb_filenamebuff_size[i],deb_filenamebuff2_size[i]);
      	strcpy(deb_filenamebuff_date[i],deb_filenamebuff2_date[i]);

	deb_filenamebuff_len[    i]=deb_filenamebuff2_len[    i];
	deb_filenamebuff_len2[   i]=deb_filenamebuff2_len2[   i];
	deb_filenamebuff_type[   i]=deb_filenamebuff2_type[   i];
	deb_filenamebuff_icon[   i]=deb_filenamebuff2_icon[   i];
	deb_filenamebuff_space[  i]=deb_filenamebuff2_space[  i];
	deb_filenamebuff_subline[i]=deb_filenamebuff2_subline[i];
    }

    p=deb_filenamebuff_ptr;
    deb_filenamebuff_ptr=deb_filenamebuff2_ptr;
    
    //clear memory
    for (j=deb_filenamebuff_ptr;j<p;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_len2[j]=0;
      deb_filenamebuff_type[j]=0;
      deb_filenamebuff_icon[j]=0;
      deb_filenamebuff_space[j]=0;
      deb_filenamebuff_subline[j]=0;
    }
    
    return(0);
}
/*
static int deb_get_dir_len(int ptr)
{
    int i,j,k;

    if (ptr<0) return(0);
    if (ptr>=MAX_FILE_NUM) return(0);
    if (ptr>=deb_filenamebuff_ptr) return(0);

    while ((deb_filenamebuff_subline[ptr]==1)&&(ptr>0))
    {
      ptr--;
    }

    i=deb_get_space(ptr);

    while(ptr>0)
    {
	ptr--;
	j=deb_get_space(ptr);
	if (j<i)
	{
	  while ((deb_filenamebuff_subline[ptr]==1)&&(ptr>0))
	  {
	    ptr--;
	  }
	  
	  break;
	}
    }

    k=ptr;

    return(k);
}
*/
/*
static int deb_get_dir_len2(int ptr)
{
    int i,j,k;

    if (ptr<0) return(0);
    if (ptr>=MAX_FILE_NUM) return(0);
    if (ptr>=deb_filenamebuff2_ptr) return(0);

    while ((deb_filenamebuff2_subline[ptr]==1)&&(ptr>0))
    {
      ptr--;
    }

    i=deb_get_space2(ptr);

    while(ptr>0)
    {
	ptr--;
	j=deb_get_space2(ptr);
	if (j<i)
	{
	  while ((deb_filenamebuff2_subline[ptr]==1)&&(ptr>0))
	  {
	    ptr--;
	  }
	  
	  break;
	}
    }

    k=ptr;

    return(k);
}
*/
static int deb_disp_bar(VideoState *cur_stream2)
{
  int   uns, uhh, umm, uss;
  int   tns, thh, tmm, tss;
  
  if ((screen_w<640)||(screen_w>7680)) return(1);
  if ((screen_h<480)||(screen_h>4320)) return(1);
  
  if (  (  (cur_stream2->show_mode != SHOW_MODE_VIDEO)  &&  ((deb_thr_r!=1)||(deb_thr_a!=1)||(deb_thr_a2!=1)||(!cur_stream2->audio_st))  )                 ||
        (  (cur_stream2->show_mode == SHOW_MODE_VIDEO)  &&  ((deb_thr_r!=1)||(deb_thr_a!=1)||(deb_thr_a2!=1)||(deb_thr_v!=1)||(!cur_stream2->video_st))  ) ||
        (  (seek_by_bytes || cur_stream2->ic->duration<=0)   )   )
  {
    uns = 0;
    uhh = 0;
    umm = 0;
    uss = 0;

    tns = 0;
    thh = 0;
    tmm = 0;
    tss = 0;

    snprintf(deb_scrn_str,300," %2d:%2d:%2d / %2d:%2d:%2d ",uhh,umm,uss,thh,tmm,tss);

    g_set_label_text(0,deb_scrn_str,300);
    
    strcpy(deb_scrn_str,"Play");
    
    g_set_button_text(0,deb_scrn_str,300);

  }
  else
  {
    if (deb_st_play==1)
    {
      uns = get_master_clock(cur_stream2);
      uhh = uns/3600;
      umm = (uns%3600)/60;
      uss = (uns%60);

      if (uns<0)
      {
	uns = 0;
	uhh = 0;
	umm = 0;
        uss = 0;
      }

      tns = cur_stream2->ic->duration/1000000LL;
      thh = tns/3600;
      tmm = (tns%3600)/60;
      tss = (tns%60);
    }
    else
    {
      uns = 0;
      uhh = 0;
      umm = 0;
      uss = 0;

      tns = 0;
      thh = 0;
      tmm = 0;
      tss = 0;
    }

    snprintf(deb_scrn_str,300," %2d:%2d:%2d / %2d:%2d:%2d ",uhh,umm,uss,thh,tmm,tss);

    g_set_label_text(0,deb_scrn_str,300);
    
    if ((deb_st_play==1)&&(cur_stream2->paused==0)) 
    {
      strcpy(deb_scrn_str,"Pause");
    }
    else
    {
      strcpy(deb_scrn_str,"Play");
    }
    
    g_set_button_text(0,deb_scrn_str,300);

    if (deb_cover==1)
    {
      if (deb_cover_close==0)
      {
	if (deb_sr_show==1)
        {
	  strcpy(deb_scrn_str2,"River On");
	}
	else
	{
	  strcpy(deb_scrn_str2,"Video Off");
	}
      }
      else
      {
	if (deb_sr_show==1)
	{
	  if (deb_sr_show_nodisp==0)
	  {
	    strcpy(deb_scrn_str2,"Video Off");
	  }
	  else
	  {
	    strcpy(deb_scrn_str2,"Video On");
	  }
	}
	else
	{
	  strcpy(deb_scrn_str2,"Video On");
	}
      }
    }
    else
    {
      if (deb_sr_show==1)
      {
	if (deb_sr_show_nodisp==0)
	{
	  strcpy(deb_scrn_str2,"River Off");
	}
	else
	{
	  strcpy(deb_scrn_str2,"River On");
	}
      }
    }

    g_set_button_text(1,deb_scrn_str2,300);
    
  }

  return(0);
}

static char deb_lower(char c1)
{
  if ((c1>='A')&&(c1<='Z')) return(c1-'A'+'a');
  else return(c1);
}

static int deb_lower_string(char *p_instr,int p_instr_size)
{
  int i;

  for (i=0;i<p_instr_size;i++)
  {
    if (p_instr[i]==0) break;
    p_instr[i]=deb_lower(p_instr[i]);
  }
  
  return(0);
}

static char deb_upper(char c1)
{
  if ((c1>='a')&&(c1<='z')) return(c1-'a'+'A');
  else return(c1);
}

static int deb_upper_string(char *p_instr,int p_instr_size)
{
  int i;

  for (i=0;i<p_instr_size;i++)
  {
    if (p_instr[i]==0) break;
    p_instr[i]=deb_upper(p_instr[i]);
  }
  
  return(0);
}

static int deb_string2int(char *string,int p1,int p2)
{
  int val;
  int i;
  int sin;
  int num;
  val=0;
  sin=1;
  num=0;
  for (i=p1;i<=p2;i++)
  {
   if (*(string+i)<=' ') continue;
   if (*(string+i)=='-')
   {
     if (num==0)
     {
       sin= -1;
       continue;
     }
     else
     {
       val=0;
       break;
     }
   }
   if (*(string+i)=='+')
   {
     if (num==0)
     {
       sin=1;
       continue;
     }
     else
     {
       val=0;
       break;
     }
   }
   if ((*(string+i)>='0')&&(*(string+i)<='9'))
   {
     num=1;
     val=val*10+(*(string+i)-48)*sin;
     continue;
   }
   val=0;
   break;
  }
  return(val);
}

static int deb_ini_is(VideoState *is)
{
  memset(is, 0, sizeof(VideoState));
  return(0);
}

static int deb_record_init(void)
{
  deb_record_fp=fopen("deb_record.txt","w");
  if (deb_record_fp==NULL) return(1);

  fclose(deb_record_fp);

  return(0);
}

static int deb_record_close(void)
{
  fclose(deb_record_fp);

  return(0);
}

static int deb_record(const char *p_str1)
{
  deb_record_fp=fopen("deb_record.txt","a");
  if (deb_record_fp==NULL) return(1);

  fputs(p_str1,deb_record_fp);
  fputs("\n",deb_record_fp);

  fclose(deb_record_fp);

  return(0);
}

// ---- binary tree -----------------------------------------------------------

static int bt_init_tree(void)
{
  int i,j;

  for (i=0;i<BTREE1_SIZE;i++)
  {
    bt_node_mark[i]=(-1);
  }

  bt_root_ptr=(-1);

  j=BTREE1_SIZE-1;  // init stack

  for (i=0;i<BTREE1_SIZE;i++)
  {
    bt_stack[i]=j;
    j--;
  }

  bt_stack_ptr=BTREE1_SIZE;

  return(0);
}

static int bt_new_node(void)
{
  int i,j;

  i=(-1);

  if (bt_stack_ptr>0)
  {
    bt_stack_ptr--;
    j=bt_stack[bt_stack_ptr];
    bt_node_mark[j]=0;
    i=j;
  }

  return(i);
}

static int bt_old_node(int ptr)
{
   if (bt_stack_ptr>=BTREE1_SIZE)
   {
     printf("In btree1,error at tn_old_node()\n");
     return(-1);
   }
   else
   {
     bt_stack[bt_stack_ptr]=ptr;
     bt_stack_ptr++;
     bt_node_mark[ptr]=(-1);
     return(0);
   }
}

static int bt_clear_node(int ptr)
{
  int i;
  
  bt_node_ptr[ptr][0]=(-1);
  bt_node_ptr[ptr][1]=(-1);
  bt_node_ptr[ptr][2]=(-1);

  for (i=0;i<FN_SIZE;i++)
  {
    bt_node_val[ptr][i]=0;
  }

  bt_node_val2[ptr]=0;
  bt_node_val3[ptr][0]=0;
  bt_node_val4[ptr][0]=0;
  bt_node_val5[ptr][0]=0;



  bt_fnc_ptr1[ptr]=0;
  bt_fnc_ptr2[ptr]=0;
  bt_fnc_str[ptr][0]=0;
  bt_fnc_msk[ptr][0]=0;
  bt_fnc_istr[ptr][0]=0;
  
  
  
  return(0);
}

static char m201_str1[3000];
static char m201_str2[3000];
static char m201_str3[3000];
static char m201_str4[3000];
//static char m201_str5[3000];

static int bt_search_node(char *pstr,char ptype)
{
  int i,j;
  //int res;

  if (deb_str_has_null(pstr,FN_SIZE)!=1) return(1);

  if (strlen(pstr)>=FN_SIZE) return(1);

  if (bt_root_ptr<0)
  {
     bt_find_ptr=(-1);
     return(1);
  }

  i=bt_root_ptr;

  bt_parent=(-1);
  bt_parent_side=2;

  bt_current=i;
  bt_child_left =bt_node_ptr[i][1];
  bt_child_right=bt_node_ptr[i][2];

  //printf("root into addr=%d,val=%d,\n",bt_current,bt_node_val[bt_current]);

  while (1)
  {

    //str_lower_string(bt_node_val[i],m201_str1);
    //str_lower_string(pstr,m201_str2);
    
    strcpy(m201_str1,bt_node_val[i]);
    strcpy(m201_str2,pstr);

    for (j=0;j<3000;j++)
    {
	m201_str3[j]=0;
	m201_str4[j]=0;
    }

//#if !defined(_WIN32)
//        deb_utf8_to_gb18030(m201_str1,m201_str3,3000);
//        deb_utf8_to_gb18030(m201_str2,m201_str4,3000);
//#else
	strcpy(m201_str3,m201_str1);
	strcpy(m201_str4,m201_str2);
//#endif

    if ((ptype==bt_node_val2[i])&&(fnc_comp(i)==0)&&(string_comp(m201_str3,m201_str4)==0))
    {
      bt_find_ptr=i;
      return(0);
    }

    if (    (ptype<bt_node_val2[i])                                          ||
	 (  (ptype==bt_node_val2[i])&&(fnc_comp(i)>0)  )                     ||
	 (  (ptype==bt_node_val2[i])&&(fnc_comp(i)==0)&&(string_comp(m201_str3,m201_str4)>0)  )  )
    {
      if (bt_node_ptr[i][2]<0)
      {
        bt_find_ptr=i;
        bt_find_side=2;
        return(1);
      }
      else
      {
        bt_parent=i;
        bt_parent_side=2;

        i=bt_node_ptr[i][2];

        bt_current=i;
        bt_child_left =bt_node_ptr[i][1];
        bt_child_right=bt_node_ptr[i][2];

        //printf("right into addr=%d,val=%d,\n",bt_current,bt_node_val[bt_current]);

        continue;
      }
    }
    
    if (     (ptype>bt_node_val2[i])                                       ||
	  (  (ptype==bt_node_val2[i])&&(fnc_comp(i)<0)  )                   ||
	  (  (ptype==bt_node_val2[i])&&(fnc_comp(i)==0)&&(string_comp(m201_str3,m201_str4)<0)  )  )
    {
      if (bt_node_ptr[i][1]<0)
      {
        bt_find_ptr=i;
        bt_find_side=1;
        return(1);
      }
      else
      {
        bt_parent=i;
        bt_parent_side=1;

        i=bt_node_ptr[i][1];

        bt_current=i;
        bt_child_left =bt_node_ptr[i][1];
        bt_child_right=bt_node_ptr[i][2];

        //printf("left into addr=%d,val=%d,\n",bt_current,bt_node_val[bt_current]);

        continue;
      }
    }
    
  }

}

static int string_comp(char *ps1,char *ps2)
{
  int 		l1,/*l2,*/i;
  unsigned char c1,c2;

  if (deb_str_has_null(ps1,FN_SIZE)!=1) return(0);
  if (deb_str_has_null(ps2,FN_SIZE)!=1) return(0);

  if (strlen(ps1)>=FN_SIZE) return(0);
  if (strlen(ps2)>=FN_SIZE) return(0);

  l1=strlen(ps1);
  //l2=strlen(ps2);

  for (i=0;i<=l1;i++) {
    c1=ps1[i];
    c2=ps2[i];
    if (c1==0)        {
      if (c2==0) return(0);
      else return(-1); }
    else {
      if (c2==0) return(1);
      else {
        if (c1<c2) return(-1);
        else {
          if (c1>c2) return(1); }}}}

  return(0);
}

static int bt_insert_node(char *pstr,char ptype)
{
  int i,j;

  if (deb_str_has_null(pstr,FN_SIZE)!=1) return(1);

  if (strlen(pstr)>=FN_SIZE) return(1);

  i=bt_search_node(pstr,ptype);

  if (i==0)
  {
    bt_find_ptr2=bt_find_ptr;
    return(0);
  }
  else
  {
    if (bt_find_ptr<0)
    {
      j=bt_new_node();
      if (j<0)
      {
        printf("In btree1,error at insert_node() when call new_node()\n");
        return(1);
      }
      else
      {
        bt_root_ptr=j;
        bt_clear_node(j);

        strcpy(bt_node_val[j],pstr);
	bt_node_val2[j]=ptype;

	bt_find_ptr2=j;
	
	fnc_save(j);
	
        return(0);
      }
    }
    else
    {
      j=bt_new_node();
      if (j<0)
      {
        printf("In btree1,error at insert_node() when call new_node()\n");
        return(1);
      }
      else
      {
        bt_clear_node(j);

        strcpy(bt_node_val[j],pstr);
	bt_node_val2[j]=ptype;

        bt_node_ptr[j][0]=bt_find_ptr;

        if (bt_find_side==2) bt_node_ptr[bt_find_ptr][2]=j;
        else bt_node_ptr[bt_find_ptr][1]=j;

	bt_find_ptr2=j;

	fnc_save(j);
	
        return(0);
      }
    }
  }
}

static int bt_delete_node(char *pstr,char ptype)
{
  int i;
  int s1,sp;
  //char str[300];

  i=bt_search_node(pstr,ptype);

  if (i==0)
  {
    if ((bt_child_left<0)&&(bt_child_right<0))  // no child
    {
      //printf("0 child\n");

      if (bt_parent<0)  // it is root
      {
	bt_root_ptr=(-1);
      }
      else
      {
        if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=(-1);
        if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=(-1);
      }
      bt_old_node(bt_find_ptr);

      //printf("0 child end\n");
    }

    if (((bt_child_left>=0)&&(bt_child_right<0))||  // one child
        ((bt_child_left<0)&&(bt_child_right>=0)))
    {
      //printf("1 child\n");

      if (bt_parent<0) // it is root;
      {
        if (bt_child_left>=0)
        {
          bt_root_ptr=bt_child_left;
        }

        if (bt_child_right>=0)
        {
          bt_root_ptr=bt_child_right;
        }
      }
      else
      {
        if (bt_child_left>=0)
        {
	  if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=bt_child_left;
	  if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=bt_child_left;
        }

        if (bt_child_right>=0)
        {
	  if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=bt_child_right;
	  if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=bt_child_right;
        }
      }
      bt_old_node(bt_find_ptr);

      //printf("1 child end\n");
    }

    if ((bt_child_left>=0)&&(bt_child_right>=0))  // two child
    {
      //printf("2 child\n");

      s1=bt_child_right;  // right sub tree
      sp=(-1);

      while (1)  // right sub tree's leftest node
      {
	if (bt_node_ptr[s1][1]>=0)
        {
          sp=s1;
          s1=bt_node_ptr[s1][1];
        }
	else break;
      }

      if (bt_node_ptr[s1][2]<0) // this node has no child
      {
	if (sp<0) // this node's parent is current node
        {
          if (bt_parent<0) // current node is root;
          {
	    bt_root_ptr=s1;
          }
          else
          {
	    if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=s1;
	    if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=s1;
          }
	  bt_node_ptr[s1][1]=bt_child_left;
	  //bt_node_ptr[s1][2]=(-1);
        }
	else
        {
	  bt_node_ptr[sp][1]=(-1);

          if (bt_parent<0) // current node is root;
          {
	    bt_root_ptr=s1;
          }
          else
          {
	    if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=s1;
	    if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=s1;
          }
	  bt_node_ptr[s1][1]=bt_child_left;
	  bt_node_ptr[s1][2]=bt_child_right;
        }
      }
      else // this node has one child
      {
	if (sp<0) // this node's parent is current node
        {
          if (bt_parent<0) // current node is root;
          {
	    bt_root_ptr=s1;
          }  
          else
          {
	    if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=s1;
	    if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=s1;
          }
	  bt_node_ptr[s1][1]=bt_child_left;
	  //bt_node_ptr[s1][2]=(-1);
        }
	else
        {
	  bt_node_ptr[sp][1]=bt_node_ptr[s1][2];

          if (bt_parent<0) // current node is root;
          {
	    bt_root_ptr=s1;
          }
          else
          {
	    if (bt_parent_side==1) bt_node_ptr[bt_parent][1]=s1;
	    if (bt_parent_side==2) bt_node_ptr[bt_parent][2]=s1;
          }
	  bt_node_ptr[s1][1]=bt_child_left;
	  bt_node_ptr[s1][2]=bt_child_right;
        }
      }

      bt_old_node(bt_find_ptr);

      //printf("2 child end\n");
    }

    return(0);
  }
  else return(1);

}

static int bt_smallest(void)
{
  int i;

  if (bt_root_ptr<0)
  {
     bt_find_ptr=(-1);
     return(1);
  }

  i=bt_root_ptr;

  while (1)
  {
    if (bt_node_ptr[i][1]<0)
    {
      bt_find_ptr=i;
      return(0);
    }
    else
    {
      i=bt_node_ptr[i][1];
      continue;
    }    
  }

}

static int bt_after_list(void)
{
  int  i,j,k;

  bt_list_ptr=0;
  bt_out_ptr=0;
  bt_err=0;
  
  i=bt_root_ptr;
  if (i<0) return(0);

  if (bt_node_ptr[i][1]>=0)
  {
    bt_list_stack[bt_list_ptr]=bt_node_ptr[i][1];
    bt_list_stack_type[bt_list_ptr]=1;
    bt_list_ptr++;
  }

  bt_list_stack[bt_list_ptr]=i;
  bt_list_stack_type[bt_list_ptr]=2;
  bt_list_ptr++;
  
  if (bt_node_ptr[i][2]>=0)
  {
    bt_list_stack[bt_list_ptr]=bt_node_ptr[i][2];
    bt_list_stack_type[bt_list_ptr]=1;
    bt_list_ptr++;
  }

  while (bt_list_ptr>0)
  {
    bt_list_ptr--;
    j=bt_list_ptr;

    if (bt_list_stack_type[j]==1)
    {
      k=bt_list_stack[j];
      
      if (bt_node_ptr[k][1]>=0)
      {
        //sprintf(str1,"add left tree %s,list_ptr=%d,",node_val[node_ptr[k][1]],list_ptr);
        if (bt_list_ptr>=BTREE1_LSIZE)
        {
          printf("In btree1,error in after_list(),BTREE1_LSIZE too small.\n");
          continue;
        }

        bt_list_stack[bt_list_ptr]=bt_node_ptr[k][1];
        bt_list_stack_type[bt_list_ptr]=1;
        bt_list_ptr++;
      }

      //sprintf(str1,"add mid tree %s,list_ptr=%d,",node_val[k],list_ptr);
      if (bt_list_ptr>=BTREE1_LSIZE)
      {
        printf("In btree1,error in after_list(),BTREE1_LSIZE too small.\n");
        continue;
      }

      bt_list_stack[bt_list_ptr]=k;
      bt_list_stack_type[bt_list_ptr]=2;
      bt_list_ptr++;

      if (bt_node_ptr[k][2]>=0)
      {
        //sprintf(str1,"add right tree %s,list_ptr=%d,",node_val[node_ptr[k][2]],list_ptr);
        if (bt_list_ptr>=BTREE1_LSIZE)
        {
          printf("In btree1,error in after_list(),BTREE1_LSIZE too small.\n");
          continue;
        }

        bt_list_stack[bt_list_ptr]=bt_node_ptr[k][2];
        bt_list_stack_type[bt_list_ptr]=1;
        bt_list_ptr++;
      }
    }
    else
    {
      k=bt_list_stack[j];

      bt_out_list(bt_node_val[k],bt_node_val2[k],k);

      //sprintf(str1,"out val %s,",node_val[k]);
      //MessageBoxNow(0,str1,"message",MB_OK);
    }
  }

  return(0);
}

static int bt_out_list(char *pstr,char ptype,int ptr)
{
  if (bt_out_ptr<0) return(0);
  if (bt_out_ptr>=BTREE1_SIZE) return(0);

  if (deb_str_has_null(pstr,FN_SIZE)!=1) return(0);

  if (strlen(pstr)>=FN_SIZE) return(0);
  
  strcpy(bt_out_buff[bt_out_ptr],pstr);
  bt_out_buff2[bt_out_ptr]=ptype;
  strcpy(bt_out_buff3[bt_out_ptr],bt_node_val3[ptr]);
  strcpy(bt_out_buff4[bt_out_ptr],bt_node_val4[ptr]);
  strcpy(bt_out_buff5[bt_out_ptr],bt_node_val5[ptr]);
  bt_out_buff6[bt_out_ptr]=bt_node_val6[ptr];

  bt_out_ptr++;

  if (bt_out_ptr>=BTREE1_SIZE) bt_out_ptr=BTREE1_SIZE-1;

  return(0);
}

// ---- end of binary tree ---------------------------------------


static char          m202_buffer1[3000];
static char          m202_buffer2[3000];
static char          m202_buffer4[300];
static char          m202_buffer5[3000];
static char          m202_buffer7[3000];
static char          m202_type;
static char          m202_ext[6];
static char          m202_size[7];
static char          m202_date[21];
static char          m202_icon;

// 1.23MB
// 123456

// 2017-01-01 08:15:01
// 1234567890123456789

static int  deb_get_dir(void)
{
  DIR           *dirp;
  struct dirent *entry;

  int  	i,j;
  long long int k;
  char 	buffer3[20];

  deb_m_info_len =0;
  deb_m_info_type=0;

  bt_init_tree();

  k=(long long int)getcwd(deb_currentpath,FN_SIZE);
  if (k==0) return(1);

  if (dirp = opendir(deb_currentpath))
  {
    while (1)
    {
      if (entry = readdir(dirp))
      {
	if (strcmp(entry->d_name,".") ==0) continue;
	if (strcmp(entry->d_name,"..")==0) continue;

	i=deb_filename_dir(deb_currentpath,entry->d_name);

	if (i==1)
	{
	  if (deb_str_has_null(entry->d_name,255)!=1) continue;

	  //if (strlen(entry->d_name)>=255) continue;

	  //strcpy(m202_buffer2,"<");
	  //av_strlcat(m202_buffer2,entry->d_name,255);
	  //av_strlcat(m202_buffer2,">",FN_SIZE);
	  strcpy(m202_buffer2,entry->d_name);

	  //printf("buffer2=%s,\n",m202_buffer2);

	  m202_type=0;
	  m202_ext[0] =0;
	  m202_size[0]=0;
	  m202_date[0]=0;
	  m202_icon=g_icon_id(i,entry->d_name,255);
	  
	  fnc_conv(m202_buffer2);

	  bt_insert_node(m202_buffer2,m202_type);
	  strcpy(bt_node_val3[bt_find_ptr2],m202_ext);
	  strcpy(bt_node_val4[bt_find_ptr2],m202_size);
	  strcpy(bt_node_val5[bt_find_ptr2],m202_date);
	  bt_node_val6[bt_find_ptr2]=m202_icon;
	  
	  j=(int)u_strlen(m202_buffer2,3000);

	  if (deb_m_info_len<j) deb_m_info_len=j;
	}
	else
	{
	  if (deb_str_has_null(entry->d_name,255)!=1) continue;

	  //if (strlen(entry->d_name)>=255) continue;

	  strcpy(m202_buffer2,entry->d_name);

	  deb_m_info_type=1;

	  // type
	  m202_type=1;

	  // ext
	  deb_filename_ext(entry->d_name,256,m202_buffer1,3000);
	  if (((int)strlen(m202_buffer1)>4)||
	      ((int)strlen(m202_buffer1)<=0))
	  {
	    strcpy(m202_ext,"    ");
	  }
	  else
	  {
	    strcat(m202_buffer1,"    ");
	    m202_buffer1[4]=0;
	    //str_lower_string(m202_buffer1,m202_ext);
	    strcpy(m202_ext,m202_buffer1);
	    deb_lower_string(m202_ext,6);
	  }

	  //size
						    //          t  g  m  k  b
	  if ((deb_m_info.st_size<0)||(deb_m_info.st_size>=1000000000000000))
	  {
	    strcpy(m202_size,"****  ");
	  }                    //      t  g  m  k  b
	  else if (deb_m_info.st_size>=1000000000000)
	  {
	    strcpy(buffer3,"TB");
			     //    g  m  k  b
	    j=deb_m_info.st_size/10000000000;
	    deb_size_format(j,m202_buffer4);
	    if (((int)strlen(m202_buffer4)<=4)&&((int)strlen(m202_buffer4)>=0)) 
	    {
	      strcpy(m202_size,m202_buffer4);
	      strcat(m202_size,buffer3);
	    }
	    else
	    {
	      strcpy(m202_size,"****");
	      strcat(m202_size,buffer3);
	    }
	  }                    //      g  m  k  b
	  else if (deb_m_info.st_size>=1000000000)
	  {
	    strcpy(buffer3,"GB");
				//    m  k  b
	    j=deb_m_info.st_size/10000000;
	    deb_size_format(j,m202_buffer4);
	    if (((int)strlen(m202_buffer4)<=4)&&((int)strlen(m202_buffer4)>=0)) 
	    {
	      strcpy(m202_size,m202_buffer4);
	      strcat(m202_size,buffer3);
	    }
	    else
	    {
	      strcpy(m202_size,"****");
	      strcat(m202_size,buffer3);
	    }
	  }         	        //      m  k  b
	  else if (deb_m_info.st_size>=1000000)
	  {
	    strcpy(buffer3,"MB");
				  //    k  b
	    j=deb_m_info.st_size/10000;
	    deb_size_format(j,m202_buffer4);
	    if (((int)strlen(m202_buffer4)<=4)&&((int)strlen(m202_buffer4)>=0)) 
	    {
	      strcpy(m202_size,m202_buffer4);
	      strcat(m202_size,buffer3);
	    }
	    else
	    {
	      strcpy(m202_size,"****");
	      strcat(m202_size,buffer3);
	    }
	  }           	        //      k  b
	  else if (deb_m_info.st_size>=1000)
	  {
	    strcpy(buffer3,"KB");
						//    b
	    j=deb_m_info.st_size/10;
	    deb_size_format(j,m202_buffer4);
	    if (((int)strlen(m202_buffer4)<=4)&&((int)strlen(m202_buffer4)>=0)) 
	    {
	      strcpy(m202_size,m202_buffer4);
	      strcat(m202_size,buffer3);
	    }
	    else
	    {
	      strcpy(m202_size,"****");
	      strcat(m202_size,buffer3);
	    }
	  }
	  else
	  {
	    strcpy(buffer3,"B ");

	    j=deb_m_info.st_size;
	    sprintf(m202_buffer4,"%4d",j);
	    if (((int)strlen(m202_buffer4)<=4)&&((int)strlen(m202_buffer4)>=0)) 
	    {
	      strcpy(m202_size,m202_buffer4);
	      strcat(m202_size,buffer3);
	    }
	    else
	    {
	      strcpy(m202_size,"****");
	      strcat(m202_size,buffer3);
	    }
	  }

	  //time
	  deb_m_info_tm=localtime(&(deb_m_info.st_mtime));
	  sprintf(m202_buffer5,"%4d-%2d-%2d  %2d:%2d:%2d",1900+deb_m_info_tm->tm_year,1+deb_m_info_tm->tm_mon,deb_m_info_tm->tm_mday,
						       deb_m_info_tm->tm_hour,deb_m_info_tm->tm_min,deb_m_info_tm->tm_sec);
	  if ((int)strlen(m202_buffer5)!=20) strcpy(m202_buffer5,"****                ");
	  strcpy(m202_date,m202_buffer5);
	  m202_icon=g_icon_id(i,entry->d_name,255);

	  fnc_conv(m202_buffer2);

	  bt_insert_node(m202_buffer2,m202_type);
	  strcpy(bt_node_val3[bt_find_ptr2],m202_ext);
	  strcpy(bt_node_val4[bt_find_ptr2],m202_size);
	  strcpy(bt_node_val5[bt_find_ptr2],m202_date);
	  bt_node_val6[bt_find_ptr2]=m202_icon;

	  for (j=0;j<3000;j++)
	  {
	    m202_buffer7[j]=0;
	  }

	  //#if !defined(_WIN32)
	  //  deb_utf8_to_gb18030(m202_buffer2,m202_buffer7,3000);
	  //#else
	    strcpy(m202_buffer7,m202_buffer2);
	  //#endif

	  j=(int)u_strlen(m202_buffer7,3000);

	  if (deb_m_info_len<j) deb_m_info_len=j;
	}
      }
      else break;
    }

    bt_after_list();
  }
  else return(-1);

  closedir(dirp);
	
  return(0);
}

static int deb_size_format(int pn,char *buffer)
{
  int   i;
  float f1;

  if ((pn<0)||(pn>=100000))
  {
    strcpy(buffer,"****");
  }
  else if (pn>=10000)
  {
    i=pn/100;
    sprintf(buffer,"%4d",i);
  }
  else if (pn>=1000)
  {
    i=pn/10;
    f1=(float)i/(float)10;
    sprintf(buffer,"%4.1f",f1);
  }
  else
  {
    f1=(float)pn/(float)100;
    sprintf(buffer,"%4.2f",f1);
  }

  if ((int)strlen(buffer)>4)
  {
    strcpy(buffer,"****");
  }
  else if ((int)strlen(buffer)<4)
  {
    strcat(buffer,"    ");
    buffer[4]=0;
  }

  return(0);
}

static int deb_str_has_null(const char *str,int str_size)
{
	int i;
	for (i=0;i<str_size;i++) if (str[i]==0) return(1);
	return(0);
}


/* before
 m5
 m11
 m101
 m201
 m301
 m401
 m501
 m601
 m701
 m801
 m901
*/

// ---- frequency separate ------------------------------------------------------

static int deb_sr_fft_trans_all(VideoState *is,long pcm)
{
  float  d1;
  long   i,j,l,m,n,p;

  pcm=44100;
  l=deb_sr_fft_start;

  // transform for each part
  while(1)
  {
    if (((deb_sr_sample_over2==0)&&(l<deb_sr_fft_end))||
	((deb_sr_sample_over2==1)&&(l<deb_sr_sample_size)))
    {
        // trans for left and right channel to many
        for (j=0;j<deb_sr_ch;j=j+deb_sr_ch)
	{
          for (m=0;m<FFT_BUFFER_SIZE;m++)
	  {
		// fetch wave data to int -- low byte at 1st , high byte at 2nd
		/*
		if ((l*2+m*4+j+0<0)||(l*2+m*4+j+0>=SAMPLE_ARRAY_SIZE*2)) continue;
		if ((l*2+m*4+j+1<0)||(l*2+m*4+j+1>=SAMPLE_ARRAY_SIZE*2)) continue;

		k=l*2+m*4+j;

        	c1=s_wave_data[k+0];
	        c2=s_wave_data[k+1];
        	n=deb_sr_cc2i(c1,c2);
		*/
		
		p=l+m*deb_sr_ch+j/deb_sr_ch;

		if ((p<0)||(p>=deb_sr_sample_size)) continue;

		n=is->sample_array[p];
		
		//n=n/3;   // reduce volumn , prevent 16bit int overflow

	        put_dlp_real_in1(m,(float)n);
	  }

	  // transform to freq
	  i=deb_sr_fft_float(FFT_BUFFER_SIZE,dlp_real_in1,dlp_real_ou1,dlp_imag_ou1);
	  if (i!=0) return(1);

	  // store to buffer for multi get
	  for (m=0;m<FFT_BUFFER_SIZE;m++)
	  {
		d1=get_dlp_real_ou1(m);
        	put_dlp_real_ou2(m,d1);

		d1=get_dlp_imag_ou1(m);
        	put_dlp_imag_ou2(m,d1);
	  }

#if DPZ_DEBUG2
	  // 1 chn 
          m=deb_sr_fft_cx(i,pcm,l);
          if (m!=0) return(1);
#else
	  for (i=0;i<FFT_BUFFER_SIZE/2;i++)
	  {
	    // 1st to 256th chn 
            m=deb_sr_fft_cx(i,pcm,l);
            if (m!=0) return(1);
	  }
#endif
	}

	deb_sr_river_ptr++;

	if (deb_sr_river_ptr>=396) //ring buffer
	{
	  deb_sr_river_ptr=0;
	  deb_sr_river_over=1;
	}

	l=l+FFT_BUFFER_SIZE*deb_sr_ch;

	if (deb_sr_sample_over2==1)  //ring buffer
	{
	  if (l>=deb_sr_sample_size)
	  {
	    l=l-deb_sr_sample_size;
	    deb_sr_sample_over2=0;
	  }
	}
    }
    else
    {
	break;
    }
  }


  return(0);
}

#if DPZ_DEBUG1
static char m601_str1[300];
#endif

static int deb_sr_fft_cx(int chn,int pcm,int mark)
{
  float  d1;
  long   i,m,n;
#if DPZ_DEBUG2
  float  f1,f2;
  int    j,p;
#endif
  float  lp,lj;

  if ((chn<0)||(chn>=FFT_BUFFER_SIZE/2)) return(0);

#if DPZ_DEBUG2

  // get 1 of 3 chn data each times
  for (m=0;m<FFT_BUFFER_SIZE;m++)
  {
    f1=(float)m/(float)FFT_BUFFER_SIZE;

    if (deb_sr_fft_deb_chn==0) //to playback high freq,low freq,middle freq audio,to sure fft is ok,need click '[river on]',set DPZ_DEBUG2 to 1,press key 'm' to switch chanel
    {
      if (((f1>=0    )&&(f1< 0.017))||
          ((f1> 0.983)&&(f1<=1    )))  // low freq
      {
	  d1=get_dlp_real_ou2(m);
	  put_dlp_real_in1(m,d1);
	  d1=get_dlp_imag_ou2(m);
	  put_dlp_imag_ou1(m,d1);
      }
      else
      {
	  put_dlp_real_in1(m,0);
          put_dlp_imag_ou1(m,0);
      }
    }


    if (deb_sr_fft_deb_chn==1)
    {
      if (((f1>=0.017)&&(f1< 0.1  ))||
          ((f1> 0.9  )&&(f1<=0.983)))  // middle freq
      {
	  d1=get_dlp_real_ou2(m);
	  put_dlp_real_in1(m,d1);
	  d1=get_dlp_imag_ou2(m);
	  put_dlp_imag_ou1(m,d1);
      }
      else
      {
	put_dlp_real_in1(m,0);
        put_dlp_imag_ou1(m,0);
      }
    }


    if (deb_sr_fft_deb_chn==2)
    {
      if ((f1>=0.1)&&(f1<=0.9))   // high freq
      {
	  d1=get_dlp_real_ou2(m);
	  put_dlp_real_in1(m,d1);
	  d1=get_dlp_imag_ou2(m);
	  put_dlp_imag_ou1(m,d1);
      }
      else
      {
	put_dlp_real_in1(m,0);
        put_dlp_imag_ou1(m,0);
      }
    }
  }

  // transform to pcm wave data
  i=deb_sr_ifft_float(FFT_BUFFER_SIZE,dlp_real_in1,dlp_real_ou1,dlp_imag_ou1);
  if (i!=0) return(1);

  // play back
  for (m=0;m<FFT_BUFFER_SIZE;m++)
  {
    f2=get_dlp_real_ou1(m);

    if (f2>=0) j=(int)(f2+0.5);
    else       j=(int)(f2-0.5);
    
    for (p=0;p<deb_sr_ch;p++)
    {
	if (p==0)  deb_sr_fft_deb[deb_sr_fft_deb_ptr2][m*deb_sr_ch+p]=j; // left channel
	else       deb_sr_fft_deb[deb_sr_fft_deb_ptr2][m*deb_sr_ch+p]=0; // other channel
    }
  }

  deb_sr_fft_deb_ptr2++;
  if (deb_sr_fft_deb_ptr2>=4) deb_sr_fft_deb_ptr2=0;

#else

  if (chn<118) // if freq more than 20khz don't show
  {
    // get 1 of 128 chn data each times
    for (m=0;m<FFT_BUFFER_SIZE;m++)
    {
      //f1=pcm*((float)m/(float)FFT_BUFFER_SIZE);

      //if (((f1>=deb_sr_frq[chn])&&(f1<deb_sr_frq[chn+1]))||
      //    ((f1>pcm-deb_sr_frq[chn+1])&&(f1<=pcm-deb_sr_frq[chn])))  // ??? > >= < <=

      //if (((m>=frq1)&&(m<frq2))||
      //    ((m>=FFT_BUFFER_SIZE-frq2)&&(m<FFT_BUFFER_SIZE-frq1)))

      if ((m-0==chn)||(FFT_BUFFER_SIZE-1-m==chn))
      {
	d1=get_dlp_real_ou2(m);
	put_dlp_real_in1(m,d1);

	d1=get_dlp_imag_ou2(m);
	put_dlp_imag_ou1(m,d1);
      }
      else
      {
	put_dlp_real_in1(m,0);
        put_dlp_imag_ou1(m,0);
      }
    }

    // transform to pcm wave data
    i=deb_sr_ifft_float(FFT_BUFFER_SIZE,dlp_real_in1,dlp_real_ou1,dlp_imag_ou1);
    if (i!=0) return(1);

    // sum volumn
    lp=0.0;

    for (m=0;m<FFT_BUFFER_SIZE;m++)
    {
      lj=get_dlp_real_ou1(m);
      if (lj<0) lj= 0-lj;
      lp=lp+lj;
    }

#if DPZ_DEBUG1
    //sprintf(m601_str1,"fft cx,lp=%lld,",lp);
    //deb_record(m601_str1);
#endif

    lp=lp/(float)FFT_BUFFER_SIZE;
    if (lp<0) lp=0;
    //lp=lp*lp;  // a sound's watt , nW=nV*nV/nR
    n=0;

    for (i=0;i<60;i++)  // a sound's db
    {
      if (lp>deb_sr_db[i])
      {
        n=i;
        continue;
      }
      else break;
    }

    if (n<0  ) n=0;
    if (n>=60) n=59;
  }
  else n=0;



  if ((deb_sr_river_ptr<0)||(deb_sr_river_ptr>=396)) return(0);

  deb_sr_river[deb_sr_river_ptr][chn]=n;
  deb_sr_river_mark[deb_sr_river_ptr]=mark;

  #if DPZ_DEBUG1
  //sprintf(m601_str1,"fft cx,river_ptr=%d,chn=%d,val=%d,mark=%d,",deb_sr_river_ptr,chn,n,mark);
  //deb_record(m601_str1);
  #endif

#endif
  return(0);
}

static int deb_sr_cc2i(char c1,char c2)
{
  /* error ---
  unsigned short i;
  short          j;
  i=(unsigned char)c2*256+(unsigned char)c1;
  j=(short)i;
  return((int)j);
  */

  /* error ---
  if (c2>=0) return(c2*256+(unsigned char)c1);
  else       return(c2*256-(unsigned char)c1);
  */

  return(c2*256+(unsigned char)c1);
}

static void deb_sr_i2cc(int k,char *cc)
{
  int  sign;
  long cvnl;

  if (k<0)
  {
    sign=(-1);
    cvnl=k;
    cvnl=(-1)*cvnl;
    if ((cvnl/256)*256-cvnl!=0)
    {
      cc[1]=(cvnl+256)/256;
      cc[0]=(char)(cc[1]*256-cvnl);
    }
    else
    {
      cc[1]=cvnl/256;
      cc[0]=0;
    }
  }
  else
  {
    sign=1;
    cc[1]=k/256;
    cc[0]=k-cc[1]*256;
  }

  cc[1]=sign*cc[1];
}

#if DPZ_DEBUG1
static char m602_str1[300];
#endif

static int deb_sr_fft_set_db(long pcm)
{
  int   i;
  float f1;

  deb_sr_db[0]=10.0;
  f1=10.0;
  
  for (i=1;i<60;i++)
  {
    f1=f1*1.12;
    deb_sr_db[i]=f1;
  }

#if DPZ_DEBUG1
  for (i=0;i<60;i++)
  {
    sprintf(m602_str1,"init db i=%d,db=%10.2f,",i,deb_sr_db[i]);
    deb_record(m602_str1);
  }
#endif

  return(0);
}

static float  get_dlp_real_in1(long addr)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	return(dlp_real_in1[addr]);
}
static float  get_dlp_real_ou1(long addr)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	return(dlp_real_ou1[addr]);
}
static float  get_dlp_imag_ou1(long addr)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	return(dlp_imag_ou1[addr]);
}
static float  get_dlp_real_ou2(long addr)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	return(dlp_real_ou2[addr]);
}
static float  get_dlp_imag_ou2(long addr)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	return(dlp_imag_ou2[addr]);
}

static int  put_dlp_real_in1(long addr,float val)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	dlp_real_in1[addr]=val;
	return(0);
}
static int  put_dlp_real_ou1(long addr,float val)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	dlp_real_ou1[addr]=val;
	return(0);
}
static int  put_dlp_imag_ou1(long addr,float val)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	dlp_imag_ou1[addr]=val;
	return(0);
}
static int  put_dlp_real_ou2(long addr,float val)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	dlp_real_ou2[addr]=val;
	return(0);
}
static int  put_dlp_imag_ou2(long addr,float val)
{
	if ((addr<0)||(addr>=FFT_BUFFER_SIZE)) return(0);
	dlp_imag_ou2[addr]=val;
	return(0);
}

//------  fft ----------------------------------------------------------------

/*============================================================================

       fourierf.c  -  Don Cross <dcross@intersrv.com>

       http://www.intersrv.com/~dcross/fft.html

       Contains definitions for doing Fourier transforms
       and inverse Fourier transforms.

       This module performs operations on arrays of 'float'.

============================================================================*/

// daipozhi modified after Aug 1998

static int deb_sr_fft_float( long    NumSamples,
			     float *RealIn,
		             float *RealOut,
		             float *ImagOut )
{
   long NumBits;    /* Number of bits needed to store indices */
   long i,j,k,n;
   long BlockSize, BlockEnd;

   float angle_numerator=(float)(2.0*DDC_PI);
   float delta_angle;
   float alpha, beta;  /* used in recurrence relation */
   float delta_ar;
   float tr, ti;       /* temp real, temp imaginary   */
   float ar, ai;       /* angle vector real, angle vector imaginary */

   if ( !deb_sr_IsPowerOfTwo(NumSamples) ) return(1);

   if (deb_sr_CheckPointer(RealIn )) return(1);
   if (deb_sr_CheckPointer(RealOut)) return(1);
   if (deb_sr_CheckPointer(ImagOut)) return(1);

   NumBits=deb_sr_NumberOfBitsNeeded(NumSamples);

   /*
   **   Do simultaneous data copy and bit-reversal ordering into outputs...
   */

   for ( i=0; i < NumSamples; i++ )
   {
      j = deb_sr_ReverseBits(i,NumBits );

      RealOut[j]=RealIn[i];
      ImagOut[j]=0;
   }

   /*
   **   Do the FFT itself...
   */

   BlockEnd = 1;
   for ( BlockSize = 2; BlockSize <= NumSamples; BlockSize <<= 1 )
   {
      delta_angle =angle_numerator/(float)BlockSize;
      alpha =sin((float)0.5*delta_angle);
      alpha =(float)2.0*alpha*alpha;
      beta  =sin(delta_angle);

      for ( i=0; i < NumSamples; i += BlockSize )
      {
	    ar = 1.0;   /* cos(0) */
	    ai = 0.0;   /* sin(0) */

	    for ( j=i, n=0; n < BlockEnd; j++, n++ )
		{
	      k = j + BlockEnd;
	      tr = ar*RealOut[k] - ai*ImagOut[k];
	      ti = ar*ImagOut[k] + ai*RealOut[k];

	      RealOut[k] = RealOut[j] - tr;
	      ImagOut[k] = ImagOut[j] - ti;

	      RealOut[j] += tr;
	      ImagOut[j] += ti;

	      delta_ar = alpha*ar + beta*ai;
	      ai -= (alpha*ai - beta*ar);
	      ar -= delta_ar;
		}
      }

      BlockEnd = BlockSize;
   }

   return(0);
}

static int deb_sr_ifft_float(long     NumSamples,
			     float  *RealIn,
		             float  *RealOut,
		             float  *ImagOut )
{
   long NumBits;    /* Number of bits needed to store indices */
   long i,j,k,n;
   long BlockSize, BlockEnd;

   float angle_numerator=(float)(2.0*DDC_PI);
   float delta_angle;
   float alpha, beta;  /* used in recurrence relation */
   float delta_ar;
   float tr, ti;       /* temp real, temp imaginary   */
   float ar, ai;       /* angle vector real, angle vector imaginary */
   float denom;

   if ( !deb_sr_IsPowerOfTwo(NumSamples) ) return(1);

   angle_numerator = -angle_numerator;

   if (deb_sr_CheckPointer(RealIn )) return(1);
   if (deb_sr_CheckPointer(RealOut)) return(1);
   if (deb_sr_CheckPointer(ImagOut)) return(1);

   NumBits=deb_sr_NumberOfBitsNeeded(NumSamples);

   /*
   **   Do simultaneous data copy and bit-reversal ordering into outputs...
   */

   for ( i=0; i < NumSamples; i++ )
   {
      j = deb_sr_ReverseBits(i,NumBits );

      RealOut[j]=RealIn[i];
   }

   for ( i=0; i < NumSamples; i++ ) RealIn[i]=ImagOut[i];

   for ( i=0; i < NumSamples; i++ )
   {
      j = deb_sr_ReverseBits(i,NumBits );

      ImagOut[j]=RealIn[i];
   }

   /*
   **   Do the FFT itself...
   */
    
   BlockEnd = 1;
   for ( BlockSize = 2; BlockSize <= NumSamples; BlockSize <<= 1 )
   {
      delta_angle =angle_numerator/(float)BlockSize;
      alpha =sin((float)0.5*delta_angle);
      alpha =(float)2.0*alpha*alpha;
      beta  =sin(delta_angle);

      for ( i=0; i < NumSamples; i += BlockSize )
      {
	    ar = 1.0;   /* cos(0) */
	    ai = 0.0;   /* sin(0) */

	    for ( j=i, n=0; n < BlockEnd; j++, n++ )
		{
	      k = j + BlockEnd;
	      tr = ar*RealOut[k] - ai*ImagOut[k];
	      ti = ar*ImagOut[k] + ai*RealOut[k];

	      RealOut[k] = RealOut[j] - tr;
	      ImagOut[k] = ImagOut[j] - ti;

	      RealOut[j] += tr;
	      ImagOut[j] += ti;

	      delta_ar = alpha*ar + beta*ai;
	      ai -= (alpha*ai - beta*ar);
	      ar -= delta_ar;
		}
      }

      BlockEnd = BlockSize;
   }

   /*
   **   Need to normalize if inverse transform...
   */

   denom=(float)NumSamples;

   for ( i=0; i < NumSamples; i++ )
   {
     RealOut[i] /= denom;
	 ImagOut[i] /= denom;
   }

   return(0);
}


static long deb_sr_IsPowerOfTwo (long x )
{
   long i, y;

   for ( i=1, y=2; i < BITS_PER_WORD; i++, y<<=1 )
   {
      if ( x == y ) return(1);
   }

   return(0);

}


static long deb_sr_NumberOfBitsNeeded(long PowerOfTwo )
{
   long i;

   if ( PowerOfTwo < 2 )
   {
      //fprintf ( stderr,
      //          ">>> Hosed in fftmisc.c: NumberOfBitsNeeded(%d)\n",
      //          PowerOfTwo );
      //
      //exit(1);
      return(1);
   }

   for ( i=0; ; i++ )
   {
      if ( PowerOfTwo & (1 << i) )
      {
	     return i;
      }
   }
}


static long deb_sr_ReverseBits(long index,long NumBits )
{
   long i, rev;

   for ( i=rev=0; i < NumBits; i++ )
   {
      rev = (rev << 1) | (index & 1);
      index >>= 1;
   }

   return rev;
}

static float deb_sr_Index_to_frequency (long NumSamples,long Index )
{
   if ( Index >= NumSamples )
   {
      return(0);
   }
   else
   {
     if ( Index <= NumSamples/2 )
	 {
       return((float)(Index/NumSamples));
	 }
     else
	 {
       return((float)(-(NumSamples-Index)/NumSamples));
	 }
   }
}

static int deb_sr_CheckPointer(float *p)
{
   if ( p == NULL )
   {
      return(1);
   }
   return(0);
}




// ---- river display -------------------------------------------------------------

static int  deb_sr_river_f_cons(void)
{
  int   i,j,k,l;
  float d1,d2,d3;
  float x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6;
  int   w,h;

  deb_sr_river_f_init     =1;
  deb_sr_river_f_init_fail=0;

  if ((screen_w<1204)||(screen_w>7680)) return(1);
  if ((screen_h<630 )||(screen_h>4320)) return(1);

  w=screen_w;
  if (w>=7680) w=7680-1;
  d1=(float)w/(150+1+1);

  h=screen_h;
  if (h>=4320) h=4320-1;
/*  
set  d2=(h-14*2-7-(d3*60))/118;
     d3=d2*0.55;
so   118*d2=h-14*2-7-(d2*0.55)*60
so   118*d2=h-14*2-7-d2*33
so   151*d2=h-14*2-7
so   d2=(h-14*2-7)/151
*/  
  d2=((float)screen_h-18*2-9-7-14-10-10)/151;

  //d3=d2*0.8;
  //d3=d2*0.6;
  d3=d2*0.55;

  x1=d1*76;                // mie dian   vanishing point   out of screen,at above
  y1=(d2*182-d3*60)*(-1);

  x2=d1;                   // start line
  y2=d3*60+d2*118;

  deb_sr_d_init();

  for (i=0;i<151;i++)
  {
    deb_sr_river_f[i][118-1][0][0]=x2;
    deb_sr_river_f[i][118-1][0][1]=y2;

    x2=x2+d1;
  }

  for (i=0;i<151;i++)     // start floor
  {
    x2=deb_sr_river_f[i][118-1][0][0];
    y2=deb_sr_river_f[i][118-1][0][1];

    if (x1>=x2)
    {
      x6=x1-((x1-x2)/(y2-y1))*(-1*y1);
      y6=0;
    }
    else
    {
      x6=((x2-x1)/(y2-y1))*(-1*y1)+x1;
      y6=0;
    }
    
    deb_sr_d_cache_clr();
    l=deb_sr_draw_line(x6,y6,x2,y2);
    if (l!=0) return(1);

    for (j=118-1-1;j>=0;j--)
    {
      x3=0;
      y3=y2-(118-1-j)*d2;

      x4=w;
      y4=y3;

      l=deb_sr_draw_line2(x3,y3,x4,y4);
      if (l!=0) return(1);

      deb_sr_river_f[i][j][0][0]=deb_sr_d_return[0];
      deb_sr_river_f[i][j][0][1]=deb_sr_d_return[1];
    }
  }


  for (i=0;i<151;i++)    // to 3d part 1
  {
    for (j=118-1;j>=0;j--)
    {
      x2=deb_sr_river_f[i][j][0][0];
      y2=deb_sr_river_f[i][j][0][1];

      if (j==118-1)  // front wall
      {
        for (k=1;k<60;k++)
        {
          x3=x2;
          y3=y2-d3*k;

          deb_sr_river_f[i][j][k][0]=x3;
          deb_sr_river_f[i][j][k][1]=y3;
        }
      }
      else
      {
        if (i==0) // left side wall
        {
	  x3=x2;
	  y3=0;

	  x4=x2;
	  y4=h;

	  deb_sr_d_cache_clr();
	  l=deb_sr_draw_line(x3,y3,x4,y4);
          if (l!=0) return(1);

          for (k=1;k<60;k++)
          {
            x5=deb_sr_river_f[i][118-1][k][0];
            y5=deb_sr_river_f[i][118-1][k][1];

            if (x1>=x5)
            {
              x6=x1-((x1-x5)/(y5-y1))*(-1*y1);
              y6=0;
            }
            else
            {
              x6=((x5-x1)/(y5-y1))*(-1*y1)+x1;
              y6=0;
            }
    
            l=deb_sr_draw_line2(x6,y6,x5,y5);
            if (l!=0) return(1);

            deb_sr_river_f[i][j][k][0]=deb_sr_d_return[0];
            deb_sr_river_f[i][j][k][1]=deb_sr_d_return[1];
          }

        }
      }
    }
  }

  // to 3d part 2

  for (k=1;k<60;k++)
  {
    for (j=118-1-1;j>=0;j--)
    {
      x2=0;
      y2=deb_sr_river_f[0][j][k][1];

      x3=w;
      y3=y2;

      deb_sr_d_cache_clr();
      l=deb_sr_draw_line(x2,y2,x3,y3);
      if (l!=0) return(1);

      for (i=1;i<151;i++)
      {
        x4=deb_sr_river_f[i][j][0][0];
        y4=0;

        x5=deb_sr_river_f[i][j][0][0];
        y5=h-1;

        l=deb_sr_draw_line2(x4,y4,x5,y5);
        if (l!=0) return(1);

        deb_sr_river_f[i][j][k][0]=deb_sr_d_return[0];
        deb_sr_river_f[i][j][k][1]=deb_sr_d_return[1];

      }
    }
  }

  return(0);
}

#if DPZ_DEBUG1
static char m604_str1[300];
#endif

static int  deb_sr_river_show(VideoState *cur_stream)
{
  int             i,j,k,l,m,n,p/*,q*/;
  struct timeval  tv;
  long long int   li,lj,lk;
  int  s_first,s_x1,s_x2,s_y1,s_y2;
  int  s_p2,s_p5,s_p8,s_p11;
  int  freq_x,freq_y;

#if DPZ_DEBUG2
  return(0);
#endif

#if DPZ_DEBUG1
  deb_record("<<< into river show");
#endif

  if (deb_sr_show_nodisp==1)
  {
#if DPZ_DEBUG1
    deb_record("no display ,return >>>");
#endif
    return(0);
  }

  if (cur_stream->paused)
  {
#if DPZ_DEBUG1
    deb_record("paused ,return >>>");
#endif
    return(0);
  }

  if (deb_sr_river_f_init==0)
  {
    i=deb_sr_river_f_cons();
    if (i!=0)
    {
      deb_sr_river_f_init_fail=1;
      return(0);
    }
#if DPZ_DEBUG1
    deb_record("show construction ok");
#endif
  }

  if (deb_sr_river_f_init_fail==1)
  {
#if DPZ_DEBUG1
    deb_record("show init fail ,return >>>");
#endif
    return(0);
  }

  gettimeofday(&tv,NULL/*,&tz*/);
	
  deb_sr_time1=tv.tv_sec;
  deb_sr_time2=tv.tv_usec;
  deb_sr_time5=deb_sr_time1*1000000+deb_sr_time2;

  li=deb_sr_time5-deb_sr_time3;	 // how many time have past , how many bytes played , how many left
  lj=2*deb_sr_ch*44100*((float)li/(float)1000000/*-0.3*/);// in bytes
  lk=deb_sr_river_adj+deb_sr_total_bytes-lj;  //bytes

  if (lk<0)				   //bytes , for pause
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show lk error1  lk=%lld, adj=%lld,bytes=%lld,now=%lld,not return",lk,deb_sr_river_adj,deb_sr_total_bytes,lj);
    deb_record(m604_str1);
#endif
    deb_sr_river_adj=deb_sr_river_adj-lk;
    lk=0;
  }

  if (lk>2*deb_sr_ch*44100)			   //bytes , more than 1s , -0.05s
  {
    deb_sr_river_adj=deb_sr_river_adj-deb_sr_ch*4096;
    lk=lk-deb_sr_ch*4096;
  }

  if (lk>deb_sr_sample_size*2)               //bytes
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show lk error2  lk=%lld, adj=%lld,bytes=%lld,now=%lld,return >>>",lk,deb_sr_river_adj,deb_sr_total_bytes,lj);
    deb_record(m604_str1);
#endif
    // error
    return(0);
  }

  lk=lk+2*deb_sr_ch*44100*(float)0.1;     // bytes    delay 0.1s

  k=lk/2;		          		// samples

  if (k<=cur_stream->sample_array_index)	// sample position of now
  {
    l=cur_stream->sample_array_index-k;
    l=FFT_BUFFER_SIZE*deb_sr_ch*(l/(FFT_BUFFER_SIZE*deb_sr_ch));
  }
  else
  {
    if (deb_sr_sample_over==1)
    {
      l=deb_sr_sample_size+cur_stream->sample_array_index-k; // ring buffer
      l=FFT_BUFFER_SIZE*deb_sr_ch*(l/(FFT_BUFFER_SIZE*deb_sr_ch));
    }
    else
    {
#if DPZ_DEBUG1
      deb_record("show k>index but not over, return >>>");
#endif
      // error
      return(0);
    }
  }

  i=0;
  k=0;
  for (j=deb_sr_river_ptr-1;j>=0;j--)
  {
    if (deb_sr_river_mark[j]==l)
    {
      k=1;
      break;
    }
    i++;
    if (i>=396) break;
  }

  if ((k==0)&&(i<396)&&(deb_sr_river_over==1))
  {
    for (j=396-1;j>deb_sr_river_ptr;j--)
    {
      if (deb_sr_river_mark[j]==l)
      {
        k=1;
        break;
      }
      i++;
      if (i>=396) break;
    }
  }

  if (k==0)
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show l=%d, not found at river buffer(river_mark[x]==l), return >>>",l);
    deb_record(m604_str1);
#endif
    return(0); // not found
  }

  if ((j/6)==(deb_sr_river_last/6)) //sr2
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show j=%d, already displayed(river[j]),return >>>",j);
    deb_record(m604_str1);
#endif
    return(0); // already displayed
  }

  m=j-6; //sr2
  if (m<0) m=396+m;
  if ((m/6)!=(deb_sr_river_last/6))  //sr2 // error,not continue;return
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show m=j-6=%d,j=%d, !=last_displayed ,    return >>>",m,j);
    deb_record(m604_str1);
#endif
    deb_sr_river_last=(j/6)*6;  //sr2
    return(0);
  }

#if DPZ_DEBUG1
  sprintf(m604_str1,"show j=%d,now display >>>",j);
  deb_record(m604_str1);
#endif

  for (n=0;n<150;n++)
    for (p=0;p<FFT_BUFFER_SIZE/2;p++) deb_sr_river2[n][p]=0;

  i=0;
  for (k=j;k>=0;k--)
  {
    for (n=0;n<FFT_BUFFER_SIZE/2;n++) deb_sr_river2[i][n]=deb_sr_river[k][n];

    i++;
    if (i>=150) break;
  }

  if ((i<150)&&(deb_sr_river_over==1))
  {
    for (k=396-1;k>deb_sr_river_ptr;k--)
    {
      for (n=0;n<FFT_BUFFER_SIZE/2;n++) deb_sr_river2[i][n]=deb_sr_river[k][n];

      i++;
      if (i>=150) break;
    }
  }



  deb_tx_locked=0;

  if (realloc_texture(&cur_stream->vis_texture, SDL_PIXELFORMAT_ARGB8888, cur_stream->width, cur_stream->height, SDL_BLENDMODE_NONE, 1) < 0)
            return(0);

  cur_stream->ytop    = 0;
  cur_stream->xleft   = 0;

  deb_tx_rect.x = 0;
  deb_tx_rect.y = 0;
  deb_tx_rect.w = cur_stream->width; 
  deb_tx_rect.h = cur_stream->height;

  if (!SDL_LockTexture(cur_stream->vis_texture, &deb_tx_rect, (void **)&deb_tx_pixels, &deb_tx_pitch)) 
  {
                deb_tx_pitch >>= 2;
  }
  else return(0);

  deb_tx_locked=1;



  // show start
  deb_sr_river_last=(j/6)*6;  //sr2

  //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  //bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

  clr_rect_black();

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][2][0][0]+10;
  freq_y=deb_sr_river_f[149][2][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"300Hz",5+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][6][0][0]+10;
  freq_y=deb_sr_river_f[149][6][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"1KHz",4+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][12][0][0]+10;
  freq_y=deb_sr_river_f[149][12][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"2KHz",4+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][29][0][0]+10;
  freq_y=deb_sr_river_f[149][29][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"5KHz",4+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][59][0][0]+10;
  freq_y=deb_sr_river_f[149][59][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"10KHz",5+1,2,0);

  // display comment
  deb_echo_str4screenstring(10,screen_h -18*2 -9-14-10,"Every Column Has 256 Samples(44.1KHz)(5.80ms)",45+1,2,0);

  //front face
  //s_p1=0;
  s_p2=255;
  //s_p3=0;

  //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
  //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);

  // right side face
  //s_p4=s_p1*0.6;
  s_p5=s_p2*0.6;
  //s_p6=s_p3*0.6;

  //bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);

  //up side face
  //s_p7=s_p1*0.42;
  s_p8=s_p2*0.42;
  //s_p9=s_p3*0.42;

  //bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);

  // left side face
  //s_p10=s_p1*0.216;
  s_p11=s_p2*0.216;
  //s_p12=s_p3*0.216;

  //bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

  // some at front , some at back , last showed at front
  for (i=0;i<=75-1;i++) // left half
  {
    for (k=0;k<118-1;k++)
    {
      n=deb_sr_river2[i][k];
      if (n<0)  n=0;
      if (n>59) n=59;

      // front face
      if (n>0)
      {
	if (deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
          //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],
			 deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],s_p2);

          //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
          //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],1,
			 deb_sr_river_f[i  ][k+1][0][1]-deb_sr_river_f[i  ][k+1][n][1],0);
	  fill_rect_green(deb_sr_river_f[i+1][k+1][n][0],
			 deb_sr_river_f[i+1][k+1][n][1],1,
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],0);
	}
/*
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][0][0],
				  deb_sr_river_f[i  ][k+1][0][1],
				  deb_sr_river_f[i+1][k+1][0][0],
				  deb_sr_river_f[i+1][k+1][0][1]);
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][n][0],
				  deb_sr_river_f[i  ][k+1][n][1],
				  deb_sr_river_f[i+1][k+1][n][0],
				  deb_sr_river_f[i+1][k+1][n][1]);
*/
        //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
        //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
        
	fill_rect_green(deb_sr_river_f[i  ][k+1][0][0],
		       deb_sr_river_f[i  ][k+1][0][1],
		       deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
		       1,s_p2);

	fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		       deb_sr_river_f[i  ][k+1][n][1],
		       deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0], 
		       1,s_p2);

	// right side face

	deb_sr_draw_line4_ini();

	deb_sr_draw_line4(deb_sr_river_f[i+1][k+1][n][0],
			  deb_sr_river_f[i+1][k+1][n][1],
			  deb_sr_river_f[i+1][k  ][n][0],
			  deb_sr_river_f[i+1][k  ][n][1],0);

	deb_sr_draw_line4(deb_sr_river_f[i+1][k+1][0][0],
			  deb_sr_river_f[i+1][k+1][0][1],
			  deb_sr_river_f[i+1][k  ][0][0],
			  deb_sr_river_f[i+1][k  ][0][1],1);

	s_first=1;

	while(1)
	{
	  s_x1=deb_sr_draw_line4_get(0);
	  if (s_x1<0) break;

	  s_x2=deb_sr_draw_line4_get(1);
	  if (s_x2<0) break;

	  if (s_x1!=s_x2) break;

	  if (s_first==1)
	  {
	    s_first=0;
	    continue;
	  }

	  if (deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1]>0)
	  {
            //SDL_SetRenderDrawColor(renderer, s_p4, s_p5, s_p6, 0);
            //bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);

	    fill_rect_green(deb_sr_d_line_dot[0][0],
			   deb_sr_d_line_dot[0][1],1,
			   deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1],s_p5);
	  }
	}

	if (deb_sr_river_f[i+1][k  ][0][1]-deb_sr_river_f[i+1][k  ][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p4, s_p5, s_p6, 0);
          //bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);
          
          // note ,for smaller moniter,bgcolor2 change to bgcolor3
	  fill_rect_green(deb_sr_river_f[i+1][k  ][n][0],
			 deb_sr_river_f[i+1][k  ][n][1],1,
			 deb_sr_river_f[i+1][k  ][0][1]-deb_sr_river_f[i+1][k  ][n][1],s_p5);
	}

	deb_sr_draw_line3(deb_sr_river_f[i+1][k+1][n][0],
			  deb_sr_river_f[i+1][k+1][n][1],
			  deb_sr_river_f[i+1][k  ][n][0],
			  deb_sr_river_f[i+1][k  ][n][1]);

	deb_sr_draw_line3(deb_sr_river_f[i+1][k+1][0][0],
			  deb_sr_river_f[i+1][k+1][0][1],
			  deb_sr_river_f[i+1][k  ][0][0],
			  deb_sr_river_f[i+1][k  ][0][1]);
      }

      //up side face

      deb_sr_draw_line4_ini();

      deb_sr_draw_line4(deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1],
			deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1],0);

      deb_sr_draw_line4(deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1],
			deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1],1);


      s_first=1;

      while(1)
      {
	s_y1=deb_sr_draw_line4_get2(0);
	if (s_y1<0) break;

	s_y2=deb_sr_draw_line4_get2(1);
	if (s_y2<0) break;

	if (s_y1!=s_y2) break;

	if (s_first==1)
	{
	  s_first=0;
	  continue;
	}


        //SDL_SetRenderDrawColor(renderer, s_p7, s_p8, s_p9, 0);
        //bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);
        
	fill_rect_green(deb_sr_d_line_dot[0][0],
		       deb_sr_d_line_dot[0][1],
		       deb_sr_d_line_dot[1][0]-deb_sr_d_line_dot[0][0],
		       1,s_p8);
      }

      //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
      //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
      
      fill_rect_green(deb_sr_river_f[i  ][k  ][n][0],
		     deb_sr_river_f[i  ][k  ][n][1],
		     deb_sr_river_f[i+1][k  ][n][0]-deb_sr_river_f[i  ][k  ][n][0],
		     1,0);
// / * repeated
      fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		     deb_sr_river_f[i  ][k+1][n][1],
		     deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0],
		     1,0);
// * /
      deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1],
			deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1]);

      deb_sr_draw_line3(deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1],
			deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1]);
    }
  }

  for (i=150-1;i>=75;i--) // right half
  { 
    for (k=0;k<118-1;k++)
    {
      n=deb_sr_river2[i][k];
      if (n<0)  n=0;
      if (n>59) n=59;

      // front face
      if (n>0)
      {
	if (deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
          //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],
			 deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],s_p2);

          //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
          //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],1,
			 deb_sr_river_f[i  ][k+1][0][1]-deb_sr_river_f[i  ][k+1][n][1],0);
	  fill_rect_green(deb_sr_river_f[i+1][k+1][n][0],
			 deb_sr_river_f[i+1][k+1][n][1],1,
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],0);
	}
/*
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][0][0],
				  deb_sr_river_f[i  ][k+1][0][1],
				  deb_sr_river_f[i+1][k+1][0][0],
				  deb_sr_river_f[i+1][k+1][0][1]);
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][n][0],
				  deb_sr_river_f[i  ][k+1][n][1],
				  deb_sr_river_f[i+1][k+1][n][0],
				  deb_sr_river_f[i+1][k+1][n][1]);
*/
        //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
        //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
          
	fill_rect_green(deb_sr_river_f[i  ][k+1][0][0],
		       deb_sr_river_f[i  ][k+1][0][1],
		       deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
		       1,s_p2);

	fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		       deb_sr_river_f[i  ][k+1][n][1],
		       deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0], 
		       1,s_p2);

	// left side face

	deb_sr_draw_line4_ini();

	deb_sr_draw_line4(deb_sr_river_f[i  ][k  ][n][0],
			  deb_sr_river_f[i  ][k  ][n][1],
			  deb_sr_river_f[i  ][k+1][n][0],
			  deb_sr_river_f[i  ][k+1][n][1],0);

	deb_sr_draw_line4(deb_sr_river_f[i  ][k  ][0][0],
			  deb_sr_river_f[i  ][k  ][0][1],
			  deb_sr_river_f[i  ][k+1][0][0],
			  deb_sr_river_f[i  ][k+1][0][1],1);

	s_first=1;

	while(1)
	{
	  s_x1=deb_sr_draw_line4_get(0);
	  if (s_x1<0) break;

	  s_x2=deb_sr_draw_line4_get(1);
	  if (s_x2<0) break;

	  if (s_x1!=s_x2) break;

	  if (s_first==1)
	  {
	    s_first=0;
	    continue;
	  }

	  if (deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1]>0)
	  {
            //SDL_SetRenderDrawColor(renderer, s_p10, s_p11, s_p12, 0);
            //bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

	    fill_rect_green(deb_sr_d_line_dot[0][0],
			   deb_sr_d_line_dot[0][1],1,
			   deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1],s_p11);
	  }
	}

	if (deb_sr_river_f[i  ][k  ][0][1]-deb_sr_river_f[i  ][k  ][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p10, s_p11, s_p12, 0);
          //bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

          // note ,for smaller moniter,bgcolor2 change to bgcolor5
	  fill_rect_green(deb_sr_river_f[i  ][k  ][n][0],
			 deb_sr_river_f[i  ][k  ][n][1],1,
			 deb_sr_river_f[i  ][k  ][0][1]-deb_sr_river_f[i  ][k  ][n][1],s_p11);
	}

	deb_sr_draw_line3(deb_sr_river_f[i  ][k  ][n][0],
			  deb_sr_river_f[i  ][k  ][n][1],
			  deb_sr_river_f[i  ][k+1][n][0],
			  deb_sr_river_f[i  ][k+1][n][1]);

	deb_sr_draw_line3(deb_sr_river_f[i  ][k  ][0][0],
			  deb_sr_river_f[i  ][k  ][0][1],
			  deb_sr_river_f[i  ][k+1][0][0],
			  deb_sr_river_f[i  ][k+1][0][1]);
      }

      //up side face

      deb_sr_draw_line4_ini();

      deb_sr_draw_line4(deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1],
			deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1],0);

      deb_sr_draw_line4(deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1],
			deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1],1);
 
      s_first=1;

      while(1)
      {
	s_y1=deb_sr_draw_line4_get2(0);
	if (s_y1<0) break;

	s_y2=deb_sr_draw_line4_get2(1);
	if (s_y2<0) break;

	if (s_y1!=s_y2) break;

	if (s_first==1)
	{
	  s_first=0;
	  continue;
	}

        //SDL_SetRenderDrawColor(renderer, s_p7, s_p8, s_p9, 0);
        //bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);
        
	fill_rect_green(deb_sr_d_line_dot[0][0],
		       deb_sr_d_line_dot[0][1],
		       deb_sr_d_line_dot[1][0]-deb_sr_d_line_dot[0][0],
		       1,s_p8);
      }

      //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
      //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
          
      fill_rect_green(deb_sr_river_f[i  ][k  ][n][0],
		     deb_sr_river_f[i  ][k  ][n][1],
		     deb_sr_river_f[i+1][k  ][n][0]-deb_sr_river_f[i  ][k  ][n][0],
		     1,0);
// / *  repeated
      fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		     deb_sr_river_f[i  ][k+1][n][1],
		     deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0],
		     1,0);
// * /
      deb_sr_draw_line3(deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1],
			deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1]);

      deb_sr_draw_line3(deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1],
			deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1]);
    }
  }

  deb_m_ref=1;

  return(0);
}

static int  deb_sr_river_f_cons_test(VideoState *cur_stream,int ptr)
{
  //int  bgcolor;
  int  x1,y1;
  int  i,j,k;

  if (deb_sr_river_f_init==0)
  {
    i=deb_sr_river_f_cons();
    if (i!=0) deb_sr_river_f_init_fail=1;
  }

  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  //bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

  fill_rectangle(0 ,0 , cur_stream->width , cur_stream->height -/*deb_ch_h*/ 18*2 -9 ); 

  for (i=0;i<151;i++)
  {
    for (j=0;j<118;j++)
    {
      for (k=0;k<=ptr;k++)
      {
	if (k>=60) continue;

        x1=deb_sr_river_f[i][j][k][0];
        y1=deb_sr_river_f[i][j][k][1];

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        //bgcolor = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);

        fill_rectangle(x1,y1, 1, 1);
      }
    }
  }

  // display comment
  deb_echo_str4screenstring(10,screen_h -18*2 -9 -14-10,"Every Column Has 256 Samples(44.1KHz)(5.80ms)",45+1,0,1);

  SDL_RenderPresent(renderer);

  return(0);
}

static int  deb_sr_d_init(void)
{
  int i,j;
  for (i=0;i<7680;i++)
    for (j=0;j<4320;j++)
      deb_sr_d_buff[i][j]=0;

  deb_sr_d_cache_ptr=0;

  return(0);
}

static int deb_sr_d_cache_clr(void)
{
  int i;
  int x,y;
  
  for (i=0;i<deb_sr_d_cache_ptr;i++)
  {
    x=deb_sr_d_cache[i][0];
    y=deb_sr_d_cache[i][1];
    deb_sr_d_buff[x][y]=0;
  }

  deb_sr_d_cache_ptr=0;

  return(0);
}

static int deb_sr_draw_line(int x1,int y1,int x2,int y2)
{
  // screen top and left is x=0;y=0;
  int x3,y3,x4,y4,x5,y5;
  int i,j;

  if ((x1<0)||(x1>=7680)) return(1);
  if ((x2<0)||(x2>=7680)) return(1);

  if ((y1<0)||(y1>=4320)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);

  if (x2<x1)
  {
    x3=x2;
    y3=y2;

    x2=x1;
    y2=y1;

    x1=x3;
    y1=y3;
  }

  if ((x1<0)||(x1>=7680)) return(0);
  if ((y1<0)||(y1>=4320)) return(0);
  deb_sr_d_buff[x1][y1]=1;

  deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x1;
  deb_sr_d_cache[deb_sr_d_cache_ptr][1]=y1;
  deb_sr_d_cache_ptr++;

  if ((x2<0)||(x2>=7680)) return(0);
  if ((y2<0)||(y2>=4320)) return(0);
  deb_sr_d_buff[x2][y2]=1;

  deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x2;
  deb_sr_d_cache[deb_sr_d_cache_ptr][1]=y2;
  deb_sr_d_cache_ptr++;

  if (x1==x2)
  {
    if (y1<y2)
    {
      for (i=y1+1;i<=y2-1;i++)// deb_sr_d_buff[x1][i]=1;
      {
	if ((x1<0)||(x1>=7680)) return(0);
	if ((i<0)||(i>=4320)) return(0);
	deb_sr_d_buff[x1][i]=1;

        deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x1;
        deb_sr_d_cache[deb_sr_d_cache_ptr][1]=i;
        deb_sr_d_cache_ptr++;
      }
    }
    else if (y2<y1)
    {
      for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--) //deb_sr_d_buff[x1][i]=1;
      {
	if ((x1<0)||(x1>=7680)) return(0);
	if ((i<0)||(i>=4320)) return(0);
	deb_sr_d_buff[x1][i]=1;

        deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x1;
        deb_sr_d_cache[deb_sr_d_cache_ptr][1]=i;
        deb_sr_d_cache_ptr++;
      }
    }
  }
  else
  {
    if (y1==y2)
    {
      for (i=x1+1;i<=x2-1;i++)// deb_sr_d_buff[i][y1]=1;
      {
	if ((i<0)||(i>=7680)) return(0);
	if ((y1<0)||(y1>=4320)) return(0);
	deb_sr_d_buff[i][y1]=1;

        deb_sr_d_cache[deb_sr_d_cache_ptr][0]=i;
        deb_sr_d_cache[deb_sr_d_cache_ptr][1]=y1;
        deb_sr_d_cache_ptr++;
      }
    }
    else if (y2>y1)
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1+(y2-y1)*i/(x2-x1);

	if ((x4<0)||(x4>=7680)) return(0);
	if ((y4<0)||(y4>=4320)) return(0);
        deb_sr_d_buff[x4][y4]=1;

        deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x4;
        deb_sr_d_cache[deb_sr_d_cache_ptr][1]=y4;
        deb_sr_d_cache_ptr++;

        if (y4>y5+1)
        {
          for (j=y5+1;j<y4;j++)// deb_sr_d_buff[x5][j]=1;
	  {
	    if ((x5<0)||(x5>=7680)) return(0);
	    if ((j<0)||(j>=4320)) return(0);
	    deb_sr_d_buff[x5][j]=1;

            deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x5;
            deb_sr_d_cache[deb_sr_d_cache_ptr][1]=j;
            deb_sr_d_cache_ptr++;
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
    else
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1-(y1-y2)*i/(x2-x1);

	if ((x4<0)||(x4>=7680)) return(0);
	if ((y4<0)||(y4>=4320)) return(0);
        deb_sr_d_buff[x4][y4]=1;

        deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x4;
        deb_sr_d_cache[deb_sr_d_cache_ptr][1]=y4;
        deb_sr_d_cache_ptr++;

        if (y4<y5-1)
        {
          for (j=y5-1;j>y4;j--) //deb_sr_d_buff[x5][j]=1;
	  {
	    if ((x5<0)||(x5>=7680)) return(0);
	    if ((j<0)||(j>=4320)) return(0);
	    deb_sr_d_buff[x5][j]=1;

            deb_sr_d_cache[deb_sr_d_cache_ptr][0]=x5;
            deb_sr_d_cache[deb_sr_d_cache_ptr][1]=j;
            deb_sr_d_cache_ptr++;
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
  }

  return(0);
}

static int deb_sr_draw_line2(int x1,int y1,int x2,int y2)
{
  // screen top and left is x=0;y=0;
  int x3,y3,x4,y4,x5,y5;
  int i,j;

  deb_sr_d_return[0]=0;
  deb_sr_d_return[1]=0;

  if ((x1<0)||(x1>=7680)) return(1);
  if ((x2<0)||(x2>=7680)) return(1);

  if ((y1<0)||(y1>=4320)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);

  if (x2<x1)
  {
    x3=x2;
    y3=y2;

    x2=x1;
    y2=y1;

    x1=x3;
    y1=y3;
  }

  //deb_sr_d_buff[x1][y1]=1;
  if ((x1<0)||(x1>=7680)) return(1);
  if ((y1<0)||(y1>=4320)) return(1);
  if (deb_sr_d_buff[x1][y1]==1)
  {
    deb_sr_d_return[0]=x1;
    deb_sr_d_return[1]=y1;
    return(0);
  }

  //deb_sr_d_buff[x2][y2]=1;
  if ((x2<0)||(x2>=7680)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);
  if (deb_sr_d_buff[x2][y2]==1)
  {
    deb_sr_d_return[0]=x2;
    deb_sr_d_return[1]=y2;
    return(0);
  }

  if (x1==x2)
  {
    if (y1<y2)
    {
      for (i=y1+1;i<=y2-1;i++)
      {
        //deb_sr_d_buff[x1][i]=1;
	  if ((x1<0)||(x1>=7680)) return(1);
	  if ((i<0)||(i>=4320)) return(1);
	  if (deb_sr_d_buff[x1][i]==1)
	  {
	    deb_sr_d_return[0]=x1;
	    deb_sr_d_return[1]=i;
	    return(0);
	  }
      }
    }
    else if (y2<y1)
    {
      for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--)
      {
	//deb_sr_d_buff[x1][i]=1;
	if ((x1<0)||(x1>=7680)) return(1);
	if ((i<0)||(i>=4320)) return(1);
	if (deb_sr_d_buff[x1][i]==1)
	{
	  deb_sr_d_return[0]=x1;
	  deb_sr_d_return[1]=i;
	  return(0);
	}
      }
    }
  }
  else
  {
    if (y1==y2)
    {
      for (i=x1+1;i<=x2-1;i++)
      {
	 //deb_sr_d_buff[i][y1]=1;
	  if ((i<0)||(i>=7680)) return(1);
	  if ((y1<0)||(y1>=4320)) return(1);
	  if (deb_sr_d_buff[i][y1]==1)
	  {
	    deb_sr_d_return[0]=i;
	    deb_sr_d_return[1]=y1;
	    return(0);
	  }
      }
    }
    else if (y2>y1)
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1+(y2-y1)*i/(x2-x1);

        //deb_sr_d_buff[x4][y4]=1;
	if ((x4<0)||(x4>=7680)) return(1);
	if ((y4<0)||(y4>=4320)) return(1);
	if (deb_sr_d_buff[x4][y4]==1)
	{
	  deb_sr_d_return[0]=x4;
	  deb_sr_d_return[1]=y4;
	  return(0);
	}

        if (y4>y5+1)
        {
          for (j=y5+1;j<y4;j++)
	  {
	    //deb_sr_d_buff[x5][j]=1;
	    if ((x5<0)||(x5>=7680)) return(1);
	    if ((j<0)||(j>=4320)) return(1);
	    if (deb_sr_d_buff[x5][j]==1)
	    {
	      deb_sr_d_return[0]=x5;
	      deb_sr_d_return[1]=j;
	      return(0);
	    }
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
    else
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1-(y1-y2)*i/(x2-x1);

        //deb_sr_d_buff[x4][y4]=1;
	if ((x4<0)||(x4>=7680)) return(1);
	if ((y4<0)||(y4>=4320)) return(1);
	if (deb_sr_d_buff[x4][y4]==1)
	{
	  deb_sr_d_return[0]=x4;
	  deb_sr_d_return[1]=y4;
	  return(0);
	}

        if (y4<y5-1)
        {
          for (j=y5-1;j>y4;j--)
	  {
	    //deb_sr_d_buff[x5][j]=1;
	    if ((x5<0)||(x5>=7680)) return(1);
	    if ((j<0)||(j>=4320)) return(1);
	    if (deb_sr_d_buff[x5][j]==1)
	    {
	      deb_sr_d_return[0]=x5;
	      deb_sr_d_return[1]=j;
	      return(0);
	    }
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
  }

  return(0);
}

static int deb_sr_draw_line3(int x1,int y1,int x2,int y2)
{
  // screen top and left is x=0;y=0;
  int x3,y3,x4,y4,x5,y5;
  int i,j;
  //int  bgcolor;

  //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  //bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

  if ((x1<0)||(x1>=7680)) return(1);
  if ((x2<0)||(x2>=7680)) return(1);

  if ((y1<0)||(y1>=4320)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);

  if (x2<x1)
  {
    x3=x2;
    y3=y2;

    x2=x1;
    y2=y1;

    x1=x3;
    y1=y3;
  }

  //deb_sr_d_buff[x1][y1]=1;
  //deb_sr_d_buff[x2][y2]=1;

  if ((x1<0)||(x1>=7680)) return(1);
  if ((y1<0)||(y1>=4320)) return(1);
  fill_rect_green(x1,y1,1,1,0);

  if ((x2<0)||(x2>=7680)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);
  fill_rect_green(x2,y2,1,1,0);

  if (x1==x2)
  {
    if (y1<y2)
    {
      for (i=y1+1;i<=y2-1;i++)
      {
	if ((x1<0)||(x1>=7680)) return(1);
	if ((i<0)||(i>=4320)) return(1);
	fill_rect_green(x1,i,1,1,0);//deb_sr_d_buff[x1][i]=1;
      }
    }
    else if (y2<y1)
    {
      for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--)
      {
	if ((x1<0)||(x1>=7680)) return(1);
	if ((i<0)||(i>=4320)) return(1);
	fill_rect_green(x1,i,1,1,0); //deb_sr_d_buff[x1][i]=1;
      }
    }
  }
  else
  {
    if (y1==y2)
    {
      for (i=x1+1;i<=x2-1;i++)
      {
	if ((i<0)||(i>=7680)) return(1);
	if ((y1<0)||(y1>=4320)) return(1);
	fill_rect_green(i,y1,1,1,0);//deb_sr_d_buff[i][y1]=1;
      }
    }
    else if (y2>y1)
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1+(y2-y1)*i/(x2-x1);

	if ((x4<0)||(x4>=7680)) return(1);
	if ((y4<0)||(y4>=4320)) return(1);
        fill_rect_green(x4,y4,1,1,0);//deb_sr_d_buff[x4][y4]=1;

        if (y4>y5+1)
        {
          for (j=y5+1;j<y4;j++)
	  {
	    if ((x5<0)||(x5>=7680)) return(1);
	    if ((j<0)||(j>=4320)) return(1);
	    fill_rect_green(x5,j,1,1,0);//deb_sr_d_buff[x5][j]=1;
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
    else
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1-(y1-y2)*i/(x2-x1);

	if ((x4<0)||(x4>=7680)) return(1);
	if ((y4<0)||(y4>=4320)) return(1);
        fill_rect_green(x4,y4,1,1,0);//deb_sr_d_buff[x4][y4]=1;

        if (y4<y5-1)
        {
          for (j=y5-1;j>y4;j--)
	  {
	    if ((x5<0)||(x5>=7680)) return(1);
	    if ((j<0)||(j>=4320)) return(1);
	    fill_rect_green(x5,j,1,1,0);//deb_sr_d_buff[x5][j]=1;
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
  }

  return(0);
}

static int deb_sr_draw_line4_ini(void)
{
	deb_sr_d_line_ptr[0]=0;
	deb_sr_d_line_ptr[1]=0;

	deb_sr_d_line_ptr2[0]=0;
	deb_sr_d_line_ptr2[1]=0;

	deb_sr_d_line_dot[0][0]=(-1);
	deb_sr_d_line_dot[0][1]=(-1);
	deb_sr_d_line_dot[1][0]=(-1);
	deb_sr_d_line_dot[1][1]=(-1);

	return(0);
}

static int deb_sr_draw_line4(int x1,int y1,int x2,int y2,int ptr)
{
  // screen top and left is x=0;y=0;
  int x3,y3,x4,y4,x5,y5;
  int i,j;
  int ptr2;

  if ((x1<0)||(x1>=7680)) return(1);
  if ((x2<0)||(x2>=7680)) return(1);

  if ((y1<0)||(y1>=4320)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);

  ptr2=0;

  if (x2<x1)
  {
    x3=x2;
    y3=y2;

    x2=x1;
    y2=y1;

    x1=x3;
    y1=y3;
  }

  //deb_sr_d_buff[x1][y1]=1;
  //deb_sr_d_buff[x2][y2]=1;

  //fill_rectangle(screen,x1,y1,1,1,bgcolor,0);
  //fill_rectangle(screen,x2,y2,1,1,bgcolor,0);

  if ((ptr <0)||(ptr >=2)) return(0);
  if ((ptr2<0)||(ptr2>=1000)) return(0);
  if ((x1<0)||(x1>=7680)) return(0);
  if ((y1<0)||(y1>=4320)) return(0);
  deb_sr_d_line[ptr][ptr2][0]=x1;
  deb_sr_d_line[ptr][ptr2][1]=y1;
  ptr2++;

  if (x1==x2)
  {
    if (y1<y2)
    {
      for (i=y1+1;i<=y2-1;i++) //fill_rectangle(screen,x1,i,1,1,bgcolor,0);//deb_sr_d_buff[x1][i]=1;
      {
	  if ((ptr <0)||(ptr >=2)) return(0);
	  if ((ptr2<0)||(ptr2>=1000)) return(0);
	  if ((x1<0)||(x1>=7680)) return(0);
	  if ((i<0)||(i>=4320)) return(0);
	  deb_sr_d_line[ptr][ptr2][0]=x1;
	  deb_sr_d_line[ptr][ptr2][1]=i;
	  ptr2++;
      }
    }
    else if (y2<y1)
    {
      for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--) //fill_rectangle(screen,x1,i,1,1,bgcolor,0); //deb_sr_d_buff[x1][i]=1;
      {
	if ((ptr <0)||(ptr >=2)) return(0);
	if ((ptr2<0)||(ptr2>=1000)) return(0);
	if ((x1<0)||(x1>=7680)) return(0);
	if ((i<0)||(i>=4320)) return(0);
	deb_sr_d_line[ptr][ptr2][0]=x1;
	deb_sr_d_line[ptr][ptr2][1]=i;
	ptr2++;
      }
    }
    else return(0);
  }
  else
  {
    if (y1==y2)
    {
      for (i=x1+1;i<=x2-1;i++) //fill_rectangle(screen,i,y1,1,1,bgcolor,0);//deb_sr_d_buff[i][y1]=1;
      {
	  if ((ptr <0)||(ptr >=2)) return(0);
	  if ((ptr2<0)||(ptr2>=1000)) return(0);
	  if ((i<0)||(i>=7680)) return(0);
	  if ((y1<0)||(y1>=4320)) return(0);
	  deb_sr_d_line[ptr][ptr2][0]=i;
	  deb_sr_d_line[ptr][ptr2][1]=y1;
	  ptr2++;
      }
    }
    else if (y2>y1)
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1+(y2-y1)*i/(x2-x1);

        //fill_rectangle(screen,x4,y4,1,1,bgcolor,0);//deb_sr_d_buff[x4][y4]=1;

        if (y4>y5+1)
        {
          for (j=y5+1;j<y4;j++) //fill_rectangle(screen,x5,j,1,1,bgcolor,0);//deb_sr_d_buff[x5][j]=1;
	  {
	    if ((ptr <0)||(ptr >=2)) return(0);
	    if ((ptr2<0)||(ptr2>=1000)) return(0);
	    if ((x5<0)||(x5>=7680)) return(0);
	    if ((j<0)||(j>=4320)) return(0);
	    deb_sr_d_line[ptr][ptr2][0]=x5;
	    deb_sr_d_line[ptr][ptr2][1]=j;
	    ptr2++;
	  }
        }

	if ((ptr <0)||(ptr >=2)) return(0);
	if ((ptr2<0)||(ptr2>=1000)) return(0);
	if ((x4<0)||(x4>=7680)) return(0);
	if ((y4<0)||(y4>=4320)) return(0);
	deb_sr_d_line[ptr][ptr2][0]=x4;
	deb_sr_d_line[ptr][ptr2][1]=y4;
	ptr2++;

        x5=x4;
	y5=y4;
      }
    }
    else
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1-(y1-y2)*i/(x2-x1);

        //fill_rectangle(screen,x4,y4,1,1,bgcolor,0);//deb_sr_d_buff[x4][y4]=1;

        if (y4<y5-1)
        {
          for (j=y5-1;j>y4;j--) //fill_rectangle(screen,x5,j,1,1,bgcolor,0);//deb_sr_d_buff[x5][j]=1;
	  {
	    if ((ptr <0)||(ptr >=2)) return(0);
	    if ((ptr2<0)||(ptr2>=1000)) return(0);
	    if ((x5<0)||(x5>=7680)) return(0);
	    if ((j<0)||(j>=4320)) return(0);
	    deb_sr_d_line[ptr][ptr2][0]=x5;
	    deb_sr_d_line[ptr][ptr2][1]=j;
	    ptr2++;
	  }
        }

	if ((ptr <0)||(ptr >=2)) return(0);
	if ((ptr2<0)||(ptr2>=1000)) return(0);
	if ((x4<0)||(x4>=7680)) return(0);
	if ((y4<0)||(y4>=4320)) return(0);
	deb_sr_d_line[ptr][ptr2][0]=x4;
	deb_sr_d_line[ptr][ptr2][1]=y4;
	ptr2++;

        x5=x4;
	y5=y4;
      }
    }
  }

  if ((ptr <0)||(ptr >=2)) return(0);
  if ((ptr2<0)||(ptr2>=1000)) return(0);
  if ((x2<0)||(x2>=7680)) return(0);
  if ((y2<0)||(y2>=4320)) return(0);
  deb_sr_d_line[ptr][ptr2][0]=x2;
  deb_sr_d_line[ptr][ptr2][1]=y2;
  ptr2++;

  deb_sr_d_line_ptr[ptr]=ptr2;

  return(0);
}

static int deb_sr_draw_line4_get(int ptr)
{
	int i;
	int x1,y1;

	i=deb_sr_d_line_ptr2[ptr];

	while(1)
	{
	        if (i<0) break;
		if (i>=1000) break;
                if (i>=deb_sr_d_line_ptr[ptr]) break;

		x1=deb_sr_d_line[ptr][i][0];
		y1=deb_sr_d_line[ptr][i][1];

		if (x1<0) return(x1);
  
		if (x1==deb_sr_d_line_dot[ptr][0])
		{
			i++;
			continue;
		}
		
		deb_sr_d_line_dot[ptr][0]=x1;
		deb_sr_d_line_dot[ptr][1]=y1;
		i++;
		deb_sr_d_line_ptr2[ptr]=i;
		return(x1);
	}

	return(-1);
}

static int deb_sr_draw_line4_get2(int ptr)
{
	int i;
	int x1,y1;

	i=deb_sr_d_line_ptr2[ptr];

	while(1)
	{
	        if (i<0) break;
		if (i>=1000) break;
                if (i>=deb_sr_d_line_ptr[ptr]) break;

		x1=deb_sr_d_line[ptr][i][0];
		y1=deb_sr_d_line[ptr][i][1];

		if (y1<0) return(y1);

		if (y1==deb_sr_d_line_dot[ptr][1])
		{
			i++;
			continue;
		}
		
		deb_sr_d_line_dot[ptr][0]=x1;
		deb_sr_d_line_dot[ptr][1]=y1;
		i++;
		deb_sr_d_line_ptr2[ptr]=i;
		return(y1);
	}

	return(-1);
}

#if DPZ_DEBUG1
static char m605_str1[300];
#endif

static int  deb_sr_river_show_pause(VideoState *cur_stream)
{
  int             i,k,n;
  int  s_first,s_x1,s_x2,s_y1,s_y2;
  int  s_p2,s_p5,s_p8,s_p11;
  int  freq_x,freq_y;

#if DPZ_DEBUG2
  return(0);
#endif

#if DPZ_DEBUG1
  deb_record("<<< into river_show_pause()");
#endif

  if (deb_sr_show_nodisp==1)
  {
#if DPZ_DEBUG1
    deb_record("no display ,return >>>");
#endif
    return(0);
  }
/*
  if (!cur_stream->paused)
  {
#if DPZ_DEBUG1
    deb_record("not paused ,return >>>");
#endif
    return(0);
  }
*/
  if (deb_sr_river_f_init==0)
  {
    i=deb_sr_river_f_cons();
    if (i!=0)
    {
      deb_sr_river_f_init_fail=1;
      return(0);
    }
#if DPZ_DEBUG1
    deb_record("show construction ok");
#endif
  }

  if (deb_sr_river_f_init_fail==1)
  {
#if DPZ_DEBUG1
    deb_record("show init fail ,return >>>");
#endif
    return(0);
  }



  deb_tx_locked=0;

  if (realloc_texture(&cur_stream->vis_texture, SDL_PIXELFORMAT_ARGB8888, cur_stream->width, cur_stream->height, SDL_BLENDMODE_NONE, 1) < 0)
            return(0);

  cur_stream->ytop    = 0;
  cur_stream->xleft   = 0;

  deb_tx_rect.x = 0;
  deb_tx_rect.y = 0;
  deb_tx_rect.w = cur_stream->width; 
  deb_tx_rect.h = cur_stream->height;

  if (!SDL_LockTexture(cur_stream->vis_texture, &deb_tx_rect, (void **)&deb_tx_pixels, &deb_tx_pitch)) 
  {
                deb_tx_pitch >>= 2;
  }
  else return(0);

  deb_tx_locked=1;



  // show start
  //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  //bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

  clr_rect_black();
  
  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][2][0][0]+10;
  freq_y=deb_sr_river_f[149][2][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"300Hz",5+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][6][0][0]+10;
  freq_y=deb_sr_river_f[149][6][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"1KHz",4+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][12][0][0]+10;
  freq_y=deb_sr_river_f[149][12][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"2KHz",4+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][29][0][0]+10;
  freq_y=deb_sr_river_f[149][29][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"5KHz",4+1,2,0);

  //SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);

  freq_x=deb_sr_river_f[149][59][0][0]+10;
  freq_y=deb_sr_river_f[149][59][0][1];
  fill_rect_green(freq_x,freq_y,15,1,255);
  deb_echo_str4screenstring(freq_x+20,freq_y-6,"10KHz",5+1,2,0);

  // display comment
  deb_echo_str4screenstring(10,screen_h -18*2 -9 -14-10,"Every Column Has 256 Samples(44.1KHz)(5.80ms)",45+1,2,0);

  //front face
  //s_p1=0;
  s_p2=255;
  //s_p3=0;

  //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
  //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);

  // right side face
  //s_p4=s_p1*0.6;
  s_p5=s_p2*0.6;
  //s_p6=s_p3*0.6;

  //bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);

  //up side face
  //s_p7=s_p1*0.42;
  s_p8=s_p2*0.42;
  //s_p9=s_p3*0.42;

  //bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);

  // left side face
  //s_p10=s_p1*0.216;
  s_p11=s_p2*0.216;
  //s_p12=s_p3*0.216;

  //bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

  // some at front , some at back , last showed at front
  for (i=0;i<=75-1;i++) // left half
  {
    for (k=0;k<118-1;k++)
    {
      n=deb_sr_river2[i][k];
      if (n<0)  n=0;
      if (n>59) n=59;

      // front face
      if (n>0)
      {
	if (deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
          //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],
			 deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],s_p2);

          //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
          //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],1,
			 deb_sr_river_f[i  ][k+1][0][1]-deb_sr_river_f[i  ][k+1][n][1],0);
	  fill_rect_green(deb_sr_river_f[i+1][k+1][n][0],
			 deb_sr_river_f[i+1][k+1][n][1],1,
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],0);
	}
/*
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][0][0],
				  deb_sr_river_f[i  ][k+1][0][1],
				  deb_sr_river_f[i+1][k+1][0][0],
				  deb_sr_river_f[i+1][k+1][0][1]);
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][n][0],
				  deb_sr_river_f[i  ][k+1][n][1],
				  deb_sr_river_f[i+1][k+1][n][0],
				  deb_sr_river_f[i+1][k+1][n][1]);
*/
        //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
        //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
        
	fill_rect_green(deb_sr_river_f[i  ][k+1][0][0],
		       deb_sr_river_f[i  ][k+1][0][1],
		       deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
		       1,s_p2);

	fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		       deb_sr_river_f[i  ][k+1][n][1],
		       deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0], 
		       1,s_p2);

	// right side face

	deb_sr_draw_line4_ini();

	deb_sr_draw_line4(deb_sr_river_f[i+1][k+1][n][0],
			  deb_sr_river_f[i+1][k+1][n][1],
			  deb_sr_river_f[i+1][k  ][n][0],
			  deb_sr_river_f[i+1][k  ][n][1],0);

	deb_sr_draw_line4(deb_sr_river_f[i+1][k+1][0][0],
			  deb_sr_river_f[i+1][k+1][0][1],
			  deb_sr_river_f[i+1][k  ][0][0],
			  deb_sr_river_f[i+1][k  ][0][1],1);

	s_first=1;

	while(1)
	{
	  s_x1=deb_sr_draw_line4_get(0);
	  if (s_x1<0) break;

	  s_x2=deb_sr_draw_line4_get(1);
	  if (s_x2<0) break;

	  if (s_x1!=s_x2) break;

	  if (s_first==1)
	  {
	    s_first=0;
	    continue;
	  }

	  if (deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1]>0)
	  {
            //SDL_SetRenderDrawColor(renderer, s_p4, s_p5, s_p6, 0);
            //bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);

	    fill_rect_green(deb_sr_d_line_dot[0][0],
			   deb_sr_d_line_dot[0][1],1,
			   deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1],s_p5);
	  }
	}

	if (deb_sr_river_f[i+1][k  ][0][1]-deb_sr_river_f[i+1][k  ][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p4, s_p5, s_p6, 0);
          //bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);
          
          // note ,for smaller moniter,bgcolor2 change to bgcolor3
	  fill_rect_green(deb_sr_river_f[i+1][k  ][n][0],
			 deb_sr_river_f[i+1][k  ][n][1],1,
			 deb_sr_river_f[i+1][k  ][0][1]-deb_sr_river_f[i+1][k  ][n][1],s_p5);
	}

	deb_sr_draw_line3(deb_sr_river_f[i+1][k+1][n][0],
			  deb_sr_river_f[i+1][k+1][n][1],
			  deb_sr_river_f[i+1][k  ][n][0],
			  deb_sr_river_f[i+1][k  ][n][1]);

	deb_sr_draw_line3(deb_sr_river_f[i+1][k+1][0][0],
			  deb_sr_river_f[i+1][k+1][0][1],
			  deb_sr_river_f[i+1][k  ][0][0],
			  deb_sr_river_f[i+1][k  ][0][1]);
      }

      //up side face

      deb_sr_draw_line4_ini();

      deb_sr_draw_line4(deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1],
			deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1],0);

      deb_sr_draw_line4(deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1],
			deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1],1);


      s_first=1;

      while(1)
      {
	s_y1=deb_sr_draw_line4_get2(0);
	if (s_y1<0) break;

	s_y2=deb_sr_draw_line4_get2(1);
	if (s_y2<0) break;

	if (s_y1!=s_y2) break;

	if (s_first==1)
	{
	  s_first=0;
	  continue;
	}


        //SDL_SetRenderDrawColor(renderer, s_p7, s_p8, s_p9, 0);
        //bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);
        
	fill_rect_green(deb_sr_d_line_dot[0][0],
		       deb_sr_d_line_dot[0][1],
		       deb_sr_d_line_dot[1][0]-deb_sr_d_line_dot[0][0],
		       1,s_p8);
      }

      //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
      //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
      
      fill_rect_green(deb_sr_river_f[i  ][k  ][n][0],
		     deb_sr_river_f[i  ][k  ][n][1],
		     deb_sr_river_f[i+1][k  ][n][0]-deb_sr_river_f[i  ][k  ][n][0],
		     1,0);
// / * repeated
      fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		     deb_sr_river_f[i  ][k+1][n][1],
		     deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0],
		     1,0);
// * /
      deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1],
			deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1]);

      deb_sr_draw_line3(deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1],
			deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1]);
    }
  }

  for (i=150-1;i>=75;i--) // right half
  { 
    for (k=0;k<118-1;k++)
    {
      n=deb_sr_river2[i][k];
      if (n<0)  n=0;
      if (n>59) n=59;

      // front face
      if (n>0)
      {
	if (deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
          //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],
			 deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],s_p2);

          //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
          //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
          
	  fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1],1,
			 deb_sr_river_f[i  ][k+1][0][1]-deb_sr_river_f[i  ][k+1][n][1],0);
	  fill_rect_green(deb_sr_river_f[i+1][k+1][n][0],
			 deb_sr_river_f[i+1][k+1][n][1],1,
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],0);
	}
/*
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][0][0],
				  deb_sr_river_f[i  ][k+1][0][1],
				  deb_sr_river_f[i+1][k+1][0][0],
				  deb_sr_river_f[i+1][k+1][0][1]);
	deb_sr_draw_line3(deb_sr_river_f[i  ][k+1][n][0],
				  deb_sr_river_f[i  ][k+1][n][1],
				  deb_sr_river_f[i+1][k+1][n][0],
				  deb_sr_river_f[i+1][k+1][n][1]);
*/
        //SDL_SetRenderDrawColor(renderer, s_p1, s_p2, s_p3, 0);
        //bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
          
	fill_rect_green(deb_sr_river_f[i  ][k+1][0][0],
		       deb_sr_river_f[i  ][k+1][0][1],
		       deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
		       1,s_p2);

	fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		       deb_sr_river_f[i  ][k+1][n][1],
		       deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0], 
		       1,s_p2);

	// left side face

	deb_sr_draw_line4_ini();

	deb_sr_draw_line4(deb_sr_river_f[i  ][k  ][n][0],
			  deb_sr_river_f[i  ][k  ][n][1],
			  deb_sr_river_f[i  ][k+1][n][0],
			  deb_sr_river_f[i  ][k+1][n][1],0);

	deb_sr_draw_line4(deb_sr_river_f[i  ][k  ][0][0],
			  deb_sr_river_f[i  ][k  ][0][1],
			  deb_sr_river_f[i  ][k+1][0][0],
			  deb_sr_river_f[i  ][k+1][0][1],1);

	s_first=1;

	while(1)
	{
	  s_x1=deb_sr_draw_line4_get(0);
	  if (s_x1<0) break;

	  s_x2=deb_sr_draw_line4_get(1);
	  if (s_x2<0) break;

	  if (s_x1!=s_x2) break;

	  if (s_first==1)
	  {
	    s_first=0;
	    continue;
	  }

	  if (deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1]>0)
	  {
            //SDL_SetRenderDrawColor(renderer, s_p10, s_p11, s_p12, 0);
            //bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

	    fill_rect_green(deb_sr_d_line_dot[0][0],
			   deb_sr_d_line_dot[0][1],1,
			   deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1],s_p11);
	  }
	}

	if (deb_sr_river_f[i  ][k  ][0][1]-deb_sr_river_f[i  ][k  ][n][1]>0)
	{
          //SDL_SetRenderDrawColor(renderer, s_p10, s_p11, s_p12, 0);
          //bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

          // note ,for smaller moniter,bgcolor2 change to bgcolor5
	  fill_rect_green(deb_sr_river_f[i  ][k  ][n][0],
			 deb_sr_river_f[i  ][k  ][n][1],1,
			 deb_sr_river_f[i  ][k  ][0][1]-deb_sr_river_f[i  ][k  ][n][1],s_p11);
	}

	deb_sr_draw_line3(deb_sr_river_f[i  ][k  ][n][0],
			  deb_sr_river_f[i  ][k  ][n][1],
			  deb_sr_river_f[i  ][k+1][n][0],
			  deb_sr_river_f[i  ][k+1][n][1]);

	deb_sr_draw_line3(deb_sr_river_f[i  ][k  ][0][0],
			  deb_sr_river_f[i  ][k  ][0][1],
			  deb_sr_river_f[i  ][k+1][0][0],
			  deb_sr_river_f[i  ][k+1][0][1]);
      }

      //up side face

      deb_sr_draw_line4_ini();

      deb_sr_draw_line4(deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1],
			deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1],0);

      deb_sr_draw_line4(deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1],
			deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1],1);
 
      s_first=1;

      while(1)
      {
	s_y1=deb_sr_draw_line4_get2(0);
	if (s_y1<0) break;

	s_y2=deb_sr_draw_line4_get2(1);
	if (s_y2<0) break;

	if (s_y1!=s_y2) break;

	if (s_first==1)
	{
	  s_first=0;
	  continue;
	}

        //SDL_SetRenderDrawColor(renderer, s_p7, s_p8, s_p9, 0);
        //bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);
        
	fill_rect_green(deb_sr_d_line_dot[0][0],
		       deb_sr_d_line_dot[0][1],
		       deb_sr_d_line_dot[1][0]-deb_sr_d_line_dot[0][0],
		       1,s_p8);
      }

      //SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
      //bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);
          
      fill_rect_green(deb_sr_river_f[i  ][k  ][n][0],
		     deb_sr_river_f[i  ][k  ][n][1],
		     deb_sr_river_f[i+1][k  ][n][0]-deb_sr_river_f[i  ][k  ][n][0],
		     1,0);
// / *  repeated
      fill_rect_green(deb_sr_river_f[i  ][k+1][n][0],
		     deb_sr_river_f[i  ][k+1][n][1],
		     deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0],
		     1,0);
// * /
      deb_sr_draw_line3(deb_sr_river_f[i+1][k  ][n][0],
			deb_sr_river_f[i+1][k  ][n][1],
			deb_sr_river_f[i+1][k+1][n][0],
			deb_sr_river_f[i+1][k+1][n][1]);

      deb_sr_draw_line3(deb_sr_river_f[i  ][k  ][n][0],
			deb_sr_river_f[i  ][k  ][n][1],
			deb_sr_river_f[i  ][k+1][n][0],
			deb_sr_river_f[i  ][k+1][n][1]);
    }
  }

  deb_m_ref=1;

  return(0);
}

int fill_rect_green(int x,int y,int w,int h,unsigned char c)
{
  int i,j;
  for (i=x;i<x+w;i++)
    for (j=y;j<y+h;j++)
      deb_set_dot(i,j,0,c,0);
  return(0);
}

int clr_rect_black(void)
{
	int       i,j,k;
        uint32_t *pixels;

        pixels=deb_tx_pixels;
	k=stream_open_is->width * stream_open_is->height;
	j=stream_open_is->width * (stream_open_is->height-18*2-9);
	for (i=0;i<k;i++)
	{
	  if (i<j)
	  {
            *pixels = 0;
             pixels++;
          }
          else
          {
            *pixels = (240<<24) + (240<<16) + (240<<8) + 240 ;
             pixels++;
          }
        }
        
        g_draw_line(0,stream_open_is->height-18*2-9,stream_open_is->width,stream_open_is->height-18*2-9,0,128,128,128);
        
	return(0);
}

int clr_rect_white(void)
{
	int       i,j,k;
        uint32_t *pixels;

        pixels=deb_tx_pixels;
	k=stream_open_is->width * stream_open_is->height;
	j=stream_open_is->width * (stream_open_is->height-18*2-9);
	for (i=0;i<k;i++)
	{
	  if (i<j)
	  {
            *pixels = (255<<24) + (255<<16) + (255<<8) + 255 ;
             pixels++;
          }
          else
          {
            *pixels = (240<<24) + (240<<16) + (240<<8) + 240 ;
             pixels++;
          }
        }
        
        g_draw_line(0,stream_open_is->height-18*2-9,stream_open_is->width,stream_open_is->height-18*2-9,0,128,128,128);
        
	return(0);
}

//file name compare

//static    char  fnc_str[FN_SIZE];
//static    char  fnc_msk[FN_SIZE];
//static    int   fnc_istr[FNC_NUM];
//static    int   fnc_ptr1,fnc_ptr2;

int fnc_conv(char *p_str)
{
  int  i,j,k,l,m,n;
  char c1;
  char str1[FN_SIZE];
  char str2[FN_SIZE];
  
  fnc_str[0]=0;
  fnc_msk[0]=0;
  fnc_istr[0]=0;
  
  fnc_ptr1=0;
  fnc_ptr2=0;
  
  if (deb_str_has_null(p_str,FN_SIZE)!=1) return(1);
  if (strlen(p_str)>=FN_SIZE) return(1);

  strcpy(str2,p_str);
  deb_lower_string(str2,FN_SIZE);
  
  i=strlen(str2)+1;
  j=0;
  
  while(j<i)
  {
    c1=str2[j];
    
    if ((c1<'0')||(c1>'9'))
    {
      fnc_str[fnc_ptr1]=c1;
      fnc_str[fnc_ptr1+1]=0;
      
      if (c1==0) fnc_msk[fnc_ptr1]=0;
      else fnc_msk[fnc_ptr1]=' ';
      
      fnc_msk[fnc_ptr1+1]=0;
      fnc_ptr1++;
      j++;
      if (c1==0) break;
    }
    else
    {
      k=j;
      l=k;
      
      while(k<i)
      {
        if ((str2[k]>='0')&&(str2[k]<='9')) 
        {
          k++;
          continue;
        }
        else break;
      }
      
      str1[0]=0;
      for (m=l;m<k;m++)
      {
        str1[m-l+0]=str2[m];
        str1[m-l+1]=0;
      }
      
      n=fnc_str2int(str1,FN_SIZE);
      if (n>=0)
      {
        if (fnc_ptr2>=FNC_NUM) break;

        fnc_str[fnc_ptr1]=' ';
        fnc_str[fnc_ptr1+1]=0;
        fnc_msk[fnc_ptr1]='1';
        fnc_msk[fnc_ptr1+1]=0;
        fnc_istr[fnc_ptr2]=n;
        fnc_ptr1++;
        fnc_ptr2++;
        j=k;
      }
      else
      {
        for (m=j;m<k;m++)
        {
          fnc_str[fnc_ptr1]=str2[m];
          fnc_str[fnc_ptr1+1]=0;
          fnc_msk[fnc_ptr1]=' ';
          fnc_msk[fnc_ptr1+1]=0;
          fnc_ptr1++;
        }
        
        j=k;
      }
    }
  }
/*  
  printf("inp str:%s,\n",p_str);
  printf("out str:%s,\n",fnc_str);
  printf("out msk:%s,\n",fnc_msk);
  
  printf("number :");
  for (i=0;i<fnc_ptr2;i++)
    printf("%d,",fnc_istr[i]);
  printf("\n");
*/  
  return(0);
}

int fnc_comp(int ptr)
{
  int           i1,i2,j,v1,v2;
  unsigned char c1,c2;
  
  i1=0;
  i2=0;
  
  for (j=0;j<fnc_ptr1;j++)
  {
    if ((fnc_msk[j]==' ')&&(bt_fnc_msk[ptr][j]==' '))
    {      
      c1=bt_fnc_str[ptr][j];
      c2=fnc_str[j];
      if (c1==0)         {
        if (c2==0) break;
        else return(-1); }
      else {
        if (c2==0) return(1);
        else {
          if (c1<c2) return(-1);
          else {
            if (c1>c2) return(1); }}}}
      
    if ((fnc_msk[j]==' ')&&(bt_fnc_msk[ptr][j]=='1'))
    {      
      c1=fnc_str[j];
      if (c1<'0') return(1);
      else if (c1>'9') return(-1);
      else  return(-1); // its number ,but more than 2G,can't proccess,treat it as bigger one
      
      i2++;
    }

    if ((fnc_msk[j]=='1')&&(bt_fnc_msk[ptr][j]==' '))
    {      
      c1=bt_fnc_str[ptr][j];
      if (c1<'0') return(-1);
      else if (c1>'9') return(1);
      else  return(1);  // its number ,but more than 2G,can't proccess,treat it as bigger one
      
      i1++;
    }

    if ((fnc_msk[j]=='1')&&(bt_fnc_msk[ptr][j]=='1'))
    {      
      if ((i1>=FNC_NUM)||(i2>=FNC_NUM)) break;

      v1=bt_fnc_istr[ptr][i2];
      v2=fnc_istr[i1];
      if (v1<v2) return(-1);
      if (v1>v2) return(1);
      
      i1++;
      i2++;
    }

    if (fnc_msk[j]==0)
    {
      if (bt_fnc_msk[ptr][j]==0) break;
      else return(1);
    }
    else if (bt_fnc_msk[ptr][j]==0)
    {
      return(-1);
    }
      
  }
  
  //printf("fnc_cmp: default return 0\n");
  
  return(0);
}

int fnc_save(int ptr)
{
  int i,j;
  
  bt_fnc_ptr1[ptr]=fnc_ptr1;
  bt_fnc_ptr2[ptr]=fnc_ptr2;
  bt_fnc_str[ptr][0]=0;
  bt_fnc_msk[ptr][0]=0;
  bt_fnc_istr[ptr][0]=0;
  
  for (i=0;i<fnc_ptr1;i++)
  {
    bt_fnc_str[ptr][i  ]=fnc_str[i];
    bt_fnc_str[ptr][i+1]=0;
    bt_fnc_msk[ptr][i  ]=fnc_msk[i];
    bt_fnc_msk[ptr][i+1]=0;
  }

  if (fnc_ptr2+1>FNC_NUM) j=FNC_NUM;
  else j=fnc_ptr2+1;
  
  for (i=0;i<j;i++)
  {
    bt_fnc_istr[ptr][i]=fnc_istr[i];
  }
  
  return(0);
}

int fnc_str2int(char *p_string,int p_string_size)
{
  long long int val;
  int i,j;
  int sin;
  int num;

  i=deb_str_has_null(p_string,p_string_size);
  if (i!=1) return(-1);
  
  val=0;
  sin=1;
  num=0;
  j=strlen(p_string);

  for (i=0;i<j;i++)
  {
    if (p_string[i]<=' ') continue;
    if (p_string[i]=='-')
    {
      if (num==0)
      {
        sin= -1;
        continue;
      }
      else
      {
        val=0;
        return(-1);
      }
    }
    if (p_string[i]=='+')
    {
      if (num==0)
      {
        sin=1;
        continue;
      }
      else
      {
        val=0;
        return(-1);
      }
    }
    if ((p_string[i]>='0')&&(p_string[i]<='9'))
    {
      num=1;
      val=val*10+(p_string[i]-'0')*sin;
      //      g  m  k  b
      if (val>2000000000) return(-1);
      continue;
    }
   
    val=0;
    return(-1);
  }

  return(val);
}

char m302_str1[300];

static int deb_no_support(char *p_str,int p_str_size)
{
  int i;
  
  i=deb_filename_ext(p_str,p_str_size,m302_str1,300);
  if (i!=0) return(1);
  if (strlen(m302_str1)>5) return(1);

  deb_lower_string(m302_str1,300);
  
  if (strcmp(m302_str1,"exe")==0) return(1);
  if (strcmp(m302_str1,"dll")==0) return(1);
  if (strcmp(m302_str1,"com")==0) return(1);
  if (strcmp(m302_str1,"bat")==0) return(1);

  if (strcmp(m302_str1,"zip")==0) return(1);
  if (strcmp(m302_str1,"gz")==0) return(1);
  if (strcmp(m302_str1,"xz")==0) return(1);
  if (strcmp(m302_str1,"rar")==0) return(1);
  if (strcmp(m302_str1,"bz2")==0) return(1);
  if (strcmp(m302_str1,"lzma")==0) return(1);
  if (strcmp(m302_str1,"tar")==0) return(1);
  if (strcmp(m302_str1,"cab")==0) return(1);
  
  if (strcmp(m302_str1,"txt")==0) return(1);
  if (strcmp(m302_str1,"doc")==0) return(1);
  if (strcmp(m302_str1,"log")==0) return(1);
  
  if (strcmp(m302_str1,"bmp")==0) return(1);
  if (strcmp(m302_str1,"jpg")==0) return(1);
  if (strcmp(m302_str1,"jpeg")==0) return(1);
  if (strcmp(m302_str1,"png")==0) return(1);
  if (strcmp(m302_str1,"gif")==0) return(1);
  
  if (strcmp(m302_str1,"lrc")==0) return(1);

  if (strcmp(m302_str1,"pdf")==0) return(1);
  if (strcmp(m302_str1,"ico")==0) return(1);

  if (strcmp(m302_str1,"c")==0) return(1);
  if (strcmp(m302_str1,"h")==0) return(1);
  if (strcmp(m302_str1,"cpp")==0) return(1);
  if (strcmp(m302_str1,"obj")==0) return(1);

  if (strcmp(m302_str1,"html")==0) return(1);
  if (strcmp(m302_str1,"htm")==0) return(1);
  if (strcmp(m302_str1,"php")==0) return(1);
  if (strcmp(m302_str1,"js")==0) return(1);
  if (strcmp(m302_str1,"css")==0) return(1);
  if (strcmp(m302_str1,"xhtml")==0) return(1);
  if (strcmp(m302_str1,"xml")==0) return(1);
  if (strcmp(m302_str1,"asp")==0) return(1);

  if (strcmp(m302_str1,"dbf")==0) return(1);
  if (strcmp(m302_str1,"lnk")==0) return(1);
  if (strcmp(m302_str1,"ini")==0) return(1);
  if (strcmp(m302_str1,"inf")==0) return(1);

  if (strcmp(m302_str1,"url")==0) return(1);
  if (strcmp(m302_str1,"m3u")==0) return(1);
  if (strcmp(m302_str1,"cue")==0) return(1);

  return(0);
}



static void init_opts(void)
{
    av_dict_set(&sws_dict, "flags", "bicubic", 0);
}




// a minimal GUI lib in gcc

int g_cut_init(void)
{
  strcpy(g_cut_map[2][0]," *");
  strcpy(g_cut_map[2][1],"**");
  
  strcpy(g_cut_map[3][0]," **");
  strcpy(g_cut_map[3][1],"***");
  strcpy(g_cut_map[3][2],"***");
  
  strcpy(g_cut_map[4][0],"  **");
  strcpy(g_cut_map[4][1]," ***");
  strcpy(g_cut_map[4][2],"****");
  strcpy(g_cut_map[4][3],"****");
  
  strcpy(g_cut_map[5][0],"  ***");
  strcpy(g_cut_map[5][1]," ****");
  strcpy(g_cut_map[5][2],"*****");
  strcpy(g_cut_map[5][3],"*****");
  strcpy(g_cut_map[5][4],"*****");
  
  strcpy(g_cut_map[6][0],"   ***");
  strcpy(g_cut_map[6][1]," *****");
  strcpy(g_cut_map[6][2]," *****");
  strcpy(g_cut_map[6][3],"******");
  strcpy(g_cut_map[6][4],"******");
  strcpy(g_cut_map[6][5],"******");

  return(0);  
}

int g_create_button(int x,int y,int w,int h,int black_button,int color1,int color2,int color3,char *cap,int cap_size,int cutcorner)
{
  int i,j,k;
  int find,find_ptr;
  
  find=0;
  find_ptr=(-1);
  
  for (k=0;k<g_button_ptr;k++)
  {
    if (g_button_delete[k]==1)
    {
      find=1;
      find_ptr=k;
      break;
    }
  }
  
  if (find==0)
  {
    if ((g_button_ptr<0)||(g_button_ptr>=G_BUTTON_NUM)) return(-1);
  
    find_ptr=g_button_ptr;
  }
  
  g_button_posi[find_ptr][0]=x;
  g_button_posi[find_ptr][1]=y;
  g_button_posi[find_ptr][2]=w;
  g_button_posi[find_ptr][3]=h;
  
  g_button_color[find_ptr][0]=color1;
  g_button_color[find_ptr][1]=color2;
  g_button_color[find_ptr][2]=color3;

  g_button_black[find_ptr]=black_button;
  g_button_enable[find_ptr]=1;
  
  if (cap_size<FN_SIZE) i=cap_size;
  else i=FN_SIZE;

  if (deb_str_has_null(cap,i)!=1) return(-1);
  
  strcpy(g_button_text[find_ptr],cap);
  
  g_button_cutcorner[find_ptr]=cutcorner;
  
  j=find_ptr;
  if (find==0) g_button_ptr++;
  g_button_delete[j]=0;
  
  return(j);
}

int g_move_button(int btn_ptr,int x,int y,int w,int h)
{
  if ((btn_ptr<0)||(btn_ptr>=G_BUTTON_NUM)||(btn_ptr>=g_button_ptr)) return(1);
  
  g_button_posi[btn_ptr][0]=x;
  g_button_posi[btn_ptr][1]=y;
  g_button_posi[btn_ptr][2]=w;
  g_button_posi[btn_ptr][3]=h;
  
  return(0);
}

int g_enable_button(int btn_ptr,int enable)
{
  if ((btn_ptr<0)||(btn_ptr>=G_BUTTON_NUM)||(btn_ptr>=g_button_ptr)) return(1);
  
  g_button_enable[btn_ptr]=enable;
  
  return(0);
}

int g_delete_button(int btn_ptr)
{
  if ((btn_ptr<0)||(btn_ptr>=G_BUTTON_NUM)||(btn_ptr>=g_button_ptr)) return(1);
  
  g_button_delete[btn_ptr]=1;
  
  return(0);
}

int g_set_button_text(int btn_ptr,char *str,int str_size)
{
  int i;

  if ((btn_ptr<0)||(btn_ptr>=G_BUTTON_NUM)||(btn_ptr>=g_button_ptr)) return(1);

  if (str_size<FN_SIZE) i=str_size;
  else i=FN_SIZE;

  if (deb_str_has_null(str,i)!=1) return(1);
  
  strcpy(g_button_text[btn_ptr],str);

  return(0);  
}

int g_set_button_text_color(int btn_ptr,int black_letter,int color1,int color2,int color3)
{
  if ((btn_ptr<0)||(btn_ptr>=G_BUTTON_NUM)||(btn_ptr>=g_button_ptr)) return(1);
  
  g_button_text_bl[btn_ptr]=black_letter;
  
  g_button_text_color[btn_ptr][0]=color1;
  g_button_text_color[btn_ptr][1]=color2;
  g_button_text_color[btn_ptr][2]=color3;
  
  return(0);
}

int g_get_button_val(int m)
{
  int i,j;
  
  g_button_paint_value_max=1.0;

  if ((m/2)*2==m)
  {
    j=m/2;
    
    for (i=0;i<j;i++)
    {
      g_button_paint_value[i]=sin(3.14*0.5-((float)(j-i-1)/(float)j)*3.14*0.5);
    }
  
    for (i=j;i<j*2;i++)
    {
      g_button_paint_value[i]=sin(3.14*0.5-((float)(i-j-0)/(float)j)*3.14*0.5);
    }
  }
  else
  {
    j=(m-1)/2;
    
    for (i=0;i<j;i++)
    {
      g_button_paint_value[i]=sin(3.14*0.5-((float)(j-i-1)/(float)j)*3.14*0.5);
    }

    g_button_paint_value[j]=sin(3.14*0.5-((float)(j-j)/(float)j)*3.14*0.5);
  
    for (i=j+1;i<j*2+1;i++)
    {
      g_button_paint_value[i]=sin(3.14*0.5-((float)(i-j-1)/(float)j)*3.14*0.5);
    }
  }

  //printf("paint button value max=%f,\n",g_button_paint_value_max);
  
  //for (i=0;i<m;i++)
  //    printf("paint button value=%f,\n",g_button_paint_value[i]);
  
  return(0);
}

int g_button_dist_to_edge(int j,int k,int l,int m,int cut,int n,int o) // distance to button's edge
{
  int i1,i2,i3,i4,i5;
  
  i1=m/2;
  i3=0;
  i4=0;
  i5=o;
  
  if ((n<i1)&&(o<i1))
  {
    for (i2=n-1;i2>=0;i2--)
    {
        if ((n<cut)&&(o<cut))
        {
          if (g_cut_map[cut][n][o]=='*') i3=n-i2;
        }
        else i3=n-i2;
    }

    for (i2=o-1;i2>=0;i2--)
    {
        if ((n<cut)&&(o<cut))
        {
          if (g_cut_map[cut][n][o]=='*') i4=o-i2;
        }
        else i4=o-i2;
    }
    
    if (i3>i4) i5=i4;
    else       i5=i3;
  }
  
  if ((n>l-i1-1)&&(o<i1))
  {
    for (i2=n+1;i2<l;i2++)
    {
        if ((l-n-1<cut)&&(o<cut))
        {
          if (g_cut_map[cut][l-n-1][o]=='*') i3=i2-n;
        }
        else i3=i2-n;
    }

    for (i2=o-1;i2>=0;i2--)
    {
        if ((l-n-1<cut)&&(o<cut))
        {
          if (g_cut_map[cut][l-n-1][o]=='*') i4=o-i2;
        }
        else i4=o-i2;
    }
    
    if (i3>i4) i5=i4;
    else       i5=i3;
  }
  
  if ((n>l-i1-1)&&(o>m-i1-1))
  {
    for (i2=n+1;i2<l;i2++)
    {
        if ((l-n-1<cut)&&(m-o-1<cut))
        {
          if (g_cut_map[cut][l-n-1][m-o-1]=='*') i3=i2-n;
        }
        else i3=i2-n;
    }

    for (i2=o+1;i2<m;i2++)
    {
        if ((l-n-1<cut)&&(m-o-1<cut))
        {
          if (g_cut_map[cut][l-n-1][m-o-1]=='*') i4=i2-o;
        }
        else i4=i2-o;
    }
    
    if (i3>i4) i5=i4;
    else       i5=i3;
    
    i5=m-i5-1;
  }
  
  if ((n<i1)&&(o>m-i1-1))
  {
    for (i2=n-1;i2>=0;i2--)
    {
        if ((n<cut)&&(m-o-1<cut))
        {
          if (g_cut_map[cut][n][m-o-1]=='*') i3=n-i2;
        }
        else i3=n-i2;
    }

    for (i2=o+1;i2<m;i2++)
    {
        if ((n<cut)&&(m-o-1<cut))
        {
          if (g_cut_map[cut][n][m-o-1]=='*') i4=i2-o;
        }
        else i4=i2-o;
    }
    
    if (i3>i4) i5=i4;
    else       i5=i3;
    
    i5=m-i5-1;
  }
  
  return(i5);
}

int g_paint_button(int vid,int mousedown)
{
  int			i,j,jb,k,l,m,m2,n,o,p,q,r,t;
  int			x,y;
  unsigned char		c1,c2,c3;
  int  			i0,i1,i2,i3,i4,i5,i6,i7;
  int                      j1,j2,j3,j4;
  int			cut;
  int			len;
  char 			str[FN_SIZE];

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);
  
  for (i=0;i<g_button_ptr;i++)
  {
    if (g_button_delete[i]==1) continue;
  
    j=g_button_posi[i][0];
    k=g_button_posi[i][1];
    l=g_button_posi[i][2];
    m=g_button_posi[i][3];
    
    i1=g_button_color[i][0];
    i2=g_button_color[i][1];
    i3=g_button_color[i][2];
    i4=255;
    
    cut=g_button_cutcorner[i];
    
    //g_draw_circle4button(m/2);
    
    g_get_button_val(m);
    
    for (n=0;n<l;n++)
    {
      for (o=0;o<m;o++)
      {
        if ((n<cut)&&(o<cut))
        {
          if (g_cut_map[cut][n][o]==' ') continue;
        }

        if ((l-n-1<cut)&&(o<cut))
        {
          if (g_cut_map[cut][l-n-1][o]==' ') continue;
        }

        if ((l-n-1<cut)&&(m-o-1<cut))
        {
          if (g_cut_map[cut][l-n-1][m-o-1]==' ') continue;
        }

        if ((n<cut)&&(m-o-1<cut))
        {
          if (g_cut_map[cut][n][m-o-1]==' ') continue;
        }
        
        t=g_button_dist_to_edge(j,k,l,m,cut,n,o); // distance to button's edge
        
        if (g_button_enable[i]==1)
        {
          if (g_button_black[i]==0)
          {
            if ((mousedown==1)&&(g_detect_ptr1==3)&&(g_detect_ptr2==i))
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, i1*7/10, i2*7/10, i3*7/10, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,k+o,i1*7/10,i2*7/10,i3*7/10);
              }
            }
            else
            {
              if (vid)
              {
                  i5=i1-(128-128*g_button_paint_value[t/*o*/]/g_button_paint_value_max)*i1/255;
                  i6=i2-(128-128*g_button_paint_value[t/*o*/]/g_button_paint_value_max)*i2/255;
                  i7=i3-(128-128*g_button_paint_value[t/*o*/]/g_button_paint_value_max)*i3/255;

                  SDL_SetRenderDrawColor(renderer, i5, i6, i7, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  i5=i1-(128-128*g_button_paint_value[t/*o*/]/g_button_paint_value_max)*i1/255;
                  i6=i2-(128-128*g_button_paint_value[t/*o*/]/g_button_paint_value_max)*i2/255;
                  i7=i3-(128-128*g_button_paint_value[t/*o*/]/g_button_paint_value_max)*i3/255;

                  deb_set_dot(j+n,k+o,i5,i6,i7);
              }
            }
          }
          else
          {
            if ((mousedown==1)&&(g_detect_ptr1==3)&&(g_detect_ptr2==i))
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, 255-(255-i1)*7/10, 255-(255-i2)*7/10, 255-(255-i3)*7/10, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,k+o,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10);
              }
            }
            else
            {
              if (vid)
              {
                  i5=i1+128*g_button_paint_value[t/*o*/]/g_button_paint_value_max;

                  SDL_SetRenderDrawColor(renderer, i5, i5, i5, i5);

                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  i5=i1+128*g_button_paint_value[t/*o*/]/g_button_paint_value_max;

                  deb_set_dot(j+n,k+o,i5,i5,i5);
              }
            }
          }
        }
        else
        {
          if (g_button_black[i]==0)
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, i1*7/10, i2*7/10, i3*7/10, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,i1*7/10,i2*7/10,i3*7/10);
            }
          }
          else
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, 255-(255-i1)*7/10, 255-(255-i2)*7/10, 255-(255-i3)*7/10, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10);
            }
          }
        }
      }
    }

    strcpy(str,g_button_text[i]);
    
    p=u_strlen(str,FN_SIZE);
    q=(l-p*7)/2;
    if (q<0) continue;
    
    r=(m-14)/2;
    if (r<0) continue;
    
    
    

    x=j+q;
    y=k+r;
    
    len=p;
    jb=0;

    while(jb<len)
    {
      if (strlen(str)>jb+0) c1=(unsigned char)str[jb+0];
      else c1=0;
    
      if (strlen(str)>jb+1) c2=(unsigned char)str[jb+1];
      else c2=0;
    
      if (strlen(str)>jb+2) c3=(unsigned char)str[jb+2];
      else c3=0;
    
      u_get_char_bmp(c1,c2,c3);

      if ((u_err==0)&&(u_err2==0))
      {
          for (m2=0;m2<u_char_size_x;m2++)
          {
            for (n=0;n<u_char_size_y;n++)
            {
              if ((m2<0)||(m2>=14)) continue;
              if ((n<0)||(n>=14)) continue;
        
              i0=u_char_bmp[m2][n];
        
              t=g_button_dist_to_edge(j,k,l,m,cut,x-j+m2,r+n); // distance to button's edge
        
              if (g_button_enable[i]==1)
              {
                if (g_button_text_bl[i]==0)  //white letter
                {
                  i0=255-i0;
        
		  if (g_button_black[i]==0)
		  {
                    if ((mousedown==1)&&(g_detect_ptr1==3)&&(g_detect_ptr2==i))
                    {
                      i5=i1*7/10;
                      i6=i2*7/10;
                      i7=i3*7/10;
                    }
                    else
                    {
                      i5=i1-(128-128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max)*i1/255;
                      i6=i2-(128-128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max)*i2/255;
                      i7=i3-(128-128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max)*i3/255;
                    }
                  }
                  else
                  {
                    if ((mousedown==1)&&(g_detect_ptr1==3)&&(g_detect_ptr2==i))
                    {
                      i5=255-(255-i1)*7/10;
                      i6=255-(255-i2)*7/10;
                      i7=255-(255-i3)*7/10;
                    }
                    else
                    {
                      i5=i1+128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max;
                      i6=i2+128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max;
                      i7=i3+128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max;
                    }
                  }

                  if (i0>=96)
                  {
                    j1=i0*g_button_text_color[i][0]/255;
                    j2=i0*g_button_text_color[i][1]/255;
                    j3=i0*g_button_text_color[i][2]/255;
                    j4=255;
                  }
                  else
                  {
                    j1=i5;
                    j2=i6;
                    j3=i7;
                    j4=255;
                  }
                }
                else //black letter
                {
                  if (i0<160)
                  {
                    j1=255-(255-i0)*(255-g_button_text_color[i][0])/255;
                    j2=255-(255-i0)*(255-g_button_text_color[i][1])/255;
                    j3=255-(255-i0)*(255-g_button_text_color[i][2])/255;
                    j4=i4;
                  }
                  else
                  {
                    if ((mousedown==1)&&(g_detect_ptr1==3)&&(g_detect_ptr2==i))
                    {
                      i5=i1*7/10;
                      i6=i2*7/10;
                      i7=i3*7/10;
                    }
                    else
                    {
                      i5=i1-(128-128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max)*i1/255;
                      i6=i2-(128-128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max)*i2/255;
                      i7=i3-(128-128*g_button_paint_value[t/*r+n*/]/g_button_paint_value_max)*i3/255;
                    }
                  
                    j1=i5;
                    j2=i6;
                    j3=i7;
                    j4=i4;
                  }
                }
        
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, j4);
                  fill_rectangle(x+m2, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m2,y+n,j1,j2,j3);
                }
              }
              else
              {
                if (g_button_text_bl[i]==0)  //white letter
                {
                  i0=255-i0;
        
                  if (i0>=96)
                  {
                    j1=(i0*g_button_text_color[i][0]/255)*7/10;
                    j2=(i0*g_button_text_color[i][1]/255)*7/10;
                    j3=(i0*g_button_text_color[i][2]/255)*7/10;
                    j4=255;
                  }
                  else
                  {
                    if (g_button_black[i]==0)
                    {
                      j1=i1*7/10;
                      j2=i2*7/10;
                      j3=i3*7/10;
                      j4=255;
                    }
                    else
                    {
                      j1=255-(255-i1)*7/10;
                      j2=255-(255-i2)*7/10;
                      j3=255-(255-i3)*7/10;
                      j4=255;
                    }
                  }
                }
                else //black letter
                {
                  if (i0<160)
                  {
                    j1=255-((255-i0)*(255-g_button_text_color[i][0])/255)*7/10;
                    j2=255-((255-i0)*(255-g_button_text_color[i][1])/255)*7/10;
                    j3=255-((255-i0)*(255-g_button_text_color[i][2])/255)*7/10;
                    j4=i4;
                  }
                  else
                  {
                    j1=i1*7/10;
                    j2=i2*7/10;
                    j3=i3*7/10;
                    j4=i4;
                  }
                }
        
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, j4);
                  fill_rectangle(x+m2, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m2,y+n,j1,j2,j3);
                }
              }
            }
          }
    
          x=x+u_char_size_x;
          jb=jb+u_nb;
          continue;
      }
      else
      {
          x=x+u_char_size_x;
          jb=jb+u_nb;
          continue;
      }
    }
    
    
    
    
  }

  return(0);
}

int g_create_scrollbar(int x,int y,int w,int h,int black_bar,int color1,int color2,int color3,int hori)
{
  int i,k;
  int find,find_ptr;
  
  find=0;
  find_ptr=(-1);
  
  for (k=0;k<g_scrollbar_ptr;k++)
  {
    if (g_scrollbar_delete[k]==1)
    {
      find=1;
      find_ptr=k;
      break;
    }
  }
  
  if (find==0)
  {
    if ((g_scrollbar_ptr<0)||(g_scrollbar_ptr>=G_SCROLLBAR_NUM)) return(-1);
  
    find_ptr=g_scrollbar_ptr;
  }
  
  g_scrollbar_posi[find_ptr][0]=x;
  g_scrollbar_posi[find_ptr][1]=y;
  g_scrollbar_posi[find_ptr][2]=w;
  g_scrollbar_posi[find_ptr][3]=h;
  
  g_scrollbar_color[find_ptr][0]=color1;
  g_scrollbar_color[find_ptr][1]=color2;
  g_scrollbar_color[find_ptr][2]=color3;

  g_scrollbar_black[find_ptr]=black_bar;
  g_scrollbar_enable[find_ptr]=1;

  g_scrollbar_cutcorner[find_ptr]=2;
  
  g_scrollbar_hori[find_ptr]=hori;
  
  i=find_ptr;
  if (find==0) g_scrollbar_ptr++;
  g_scrollbar_delete[i]=0;
  
  return(i);
}

int g_move_scrollbar(int bar_ptr,int x,int y,int w,int h)
{
  if ((bar_ptr<0)||(bar_ptr>=G_SCROLLBAR_NUM)||(bar_ptr>=g_scrollbar_ptr)) return(1);
  
  g_scrollbar_posi[bar_ptr][0]=x;
  g_scrollbar_posi[bar_ptr][1]=y;
  g_scrollbar_posi[bar_ptr][2]=w;
  g_scrollbar_posi[bar_ptr][3]=h;
  
  return(0);
}

int g_enable_scrollbar(int bar_ptr,int enable)
{
  if ((bar_ptr<0)||(bar_ptr>=G_SCROLLBAR_NUM)||(bar_ptr>=g_scrollbar_ptr)) return(1);
  
  g_scrollbar_enable[bar_ptr]=enable;
  
  return(0);
}

int g_delete_scrollbar(int bar_ptr)
{
  if ((bar_ptr<0)||(bar_ptr>=G_SCROLLBAR_NUM)||(bar_ptr>=g_scrollbar_ptr)) return(1);
  
  g_scrollbar_delete[bar_ptr]=1;
  
  return(0);
}

int g_set_scrollbar_value_max(int ptr,float v)
{
  if ((ptr<0)||(ptr>=G_SCROLLBAR_NUM)||(ptr>=g_scrollbar_ptr)) return(0);
  
  g_scrollbar_value_max[ptr]=v;
  
  return(0);
}

float g_get_scrollbar_value_max(int ptr)
{
  if ((ptr<0)||(ptr>=G_SCROLLBAR_NUM)||(ptr>=g_scrollbar_ptr)) return(0);
  
  return(g_scrollbar_value_max[ptr]);
}

int g_set_scrollbar_value_now(int ptr,float v)
{
  if ((ptr<0)||(ptr>=G_SCROLLBAR_NUM)||(ptr>=g_scrollbar_ptr)) return(0);
  
  g_scrollbar_value_now[ptr]=v;
  
  return(0);
}

float g_get_scrollbar_value_now(int ptr)
{
  if ((ptr<0)||(ptr>=G_SCROLLBAR_NUM)||(ptr>=g_scrollbar_ptr)) return(0);
  
  return(g_scrollbar_value_now[ptr]);
}

int g_paint_scrollbar(int vid)
{
  int			i,j,k,l,m,n,o,p,q,r,s,t;
  int  			i1,i2,i3,i4;
  int  			j1,j2,j3;
  //int			cut;
  int			hori;
  float                 f1,f2;

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);

  for (i=0;i<g_scrollbar_ptr;i++)
  {
    if (g_scrollbar_delete[i]==1) continue;
    
    j=g_scrollbar_posi[i][0];
    k=g_scrollbar_posi[i][1];
    l=g_scrollbar_posi[i][2];
    m=g_scrollbar_posi[i][3];
    
    i1=g_scrollbar_color[i][0];
    i2=g_scrollbar_color[i][1];
    i3=g_scrollbar_color[i][2];
    i4=255;
    
    //cut=g_scrollbar_cutcorner[i];
    hori=g_scrollbar_hori[i];
    
    if (hori==0)
    {
      p=(m-5)/2;
      q=k+p;
    
      for (n=0;n<l;n++)
      {
        for (o=0;o<5;o++)
        {
          if ((n==0  )&&(o==0)) continue;
          if ((n==l-1)&&(o==0)) continue;
          if ((n==l-1)&&(o==4)) continue;
          if ((n==0  )&&(o==4)) continue;

	  if      ((n==0)||(n==l-1)) { j1=128;j2=128;j3=128; }
	  else if ((o==0)||(o==4  )) { j1=128;j2=128;j3=128; }
	  else    { j1=i1;j2=i2;j3=i3; }
	  
          if (g_scrollbar_enable[i]==1)
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, i4);
                  fill_rectangle(j+n, q+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,q+o,j1,j2,j3);
            }
          }
          else
          {
            if (g_scrollbar_black[i]==0)
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, j1*7/10, j2*7/10, j3*7/10, i4);
                  fill_rectangle(j+n, q+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,q+o,j1*7/10,j2*7/10,j3*7/10);
              }
            }
            else
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, 255-(255-j1)*7/10, 255-(255-j2)*7/10, 255-(255-j3)*7/10, i4);
                  fill_rectangle(j+n, q+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,q+o,255-(255-j1)*7/10,255-(255-j2)*7/10,255-(255-j3)*7/10);
              }
            }
          }
        }
      }
      
      f1=g_get_scrollbar_value_max(i);
      f2=g_get_scrollbar_value_now(i);
      
      if (f1<=0) continue;
      
      r=l*(f2/f1);
      if (r+5>l) r=l-5;
      if (r<0) r=0;
      r=j+r;
      
      s=(m-12)/2;
      t=k+s;
      
      for (n=0;n<5;n++)
      {
        for (o=0;o<12;o++)
        {
          if ((n==0  )&&(o==0 )) continue;
          if ((n==4  )&&(o==0 )) continue;
          if ((n==4  )&&(o==11)) continue;
          if ((n==0  )&&(o==11)) continue;

	  if      ((n==0)||(n==4  )) { j1=128;j2=128;j3=128; }
	  else if ((o==0)||(o==11 )) { j1=128;j2=128;j3=128; }
	  else    { j1=i1;j2=i2;j3=i3; }
	  
        
          if (g_scrollbar_enable[i]==1)
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, i4);
                  fill_rectangle(r+n, t+o, 1, 1);
            }
            else
            {
                  deb_set_dot(r+n,t+o,j1,j2,j3);
            }
          }
          else
          {
            if (g_scrollbar_black[i]==0)
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, j1*7/10, j2*7/10, j3*7/10, i4);
                  fill_rectangle(r+n, t+o, 1, 1);
              }
              else
              {
                  deb_set_dot(r+n,t+o,j1*7/10,j2*7/10,j3*7/10);
              }
            }
            else
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, 255-(255-j1)*7/10, 255-(255-j2)*7/10, 255-(255-j3)*7/10, i4);
                  fill_rectangle(r+n, t+o, 1, 1);
              }
              else
              {
                  deb_set_dot(r+n,t+o,255-(255-j1)*7/10,255-(255-j2)*7/10,255-(255-j3)*7/10);
              }
            }
          }
        }
      }
      
    }
  }

  return(0);
}

int g_create_dirview(int x,int y,int w,int h)
{
  int i;
  
  if ((g_dirview_ptr<0)||(g_dirview_ptr>=G_DIRVIEW_NUM)) return(-1);
  
  g_dirview_posi[g_dirview_ptr][0]=x;
  g_dirview_posi[g_dirview_ptr][1]=y;
  g_dirview_posi[g_dirview_ptr][2]=w;
  g_dirview_posi[g_dirview_ptr][3]=h;
  
  g_dirview_cutcorner[g_dirview_ptr]=6;
  
  i=g_dirview_ptr;
  g_dirview_ptr++;
  
  return(i);
}

int g_move_dirview(int box_ptr,int x,int y,int w,int h)
{
  if ((box_ptr<0)||(box_ptr>=G_DIRVIEW_NUM)||(box_ptr>=g_dirview_ptr)) return(1);
  
  g_dirview_posi[box_ptr][0]=x;
  g_dirview_posi[box_ptr][1]=y;
  g_dirview_posi[box_ptr][2]=w;
  g_dirview_posi[box_ptr][3]=h;
  
  return(0);
}

int g_paint_dirview(void)
{
  int			i,j,k,l,m,n,o,p,q,r,s,t,u;
  int			cut;

  deb_disp_dir(stream_open_is);

  g_dirview_top=0;
  g_dirview_bottom=0;

  if (deb_tx_locked!=1) return(0);

  for (i=0;i<g_dirview_ptr;i++)
  {
    j=g_dirview_posi[i][0];
    k=g_dirview_posi[i][1];
    l=g_dirview_posi[i][2];
    m=g_dirview_posi[i][3];
    
    cut=g_dirview_cutcorner[i];
    
    p=j+l-14;
    q=k;
    
    for (n=0;n<14;n++)
    {
      for (o=0;o<m;o++)
      {
                  deb_set_dot(p+n,q+o,192,192,192);
      }
    }
      
    //deb_filenamebuff_ptr
    //deb_filenamebuff_n+1
    //deb_filenamebuff_n+m/14
      
    if (deb_filenamebuff_ptr<=m/14)
    {
      g_dirview_top=0;
      continue;
    }
      
    r=(m-2)*((float)deb_filenamebuff_n      )/(float)deb_filenamebuff_ptr;
    s=(m-2)*((float)deb_filenamebuff_n+m/14 )/(float)deb_filenamebuff_ptr;
      
    if ((r<0)||(s<0)) continue;
      
    t=k+1+r;
      
    u=s-r+1;
    if (u<5) u=5;

    if (t+u>=m)
    {
      t=m-u-1;
    }
      
    g_dirview_top=t;
    g_dirview_bottom=t+u-1;
      
    for (n=0;n<12;n++)
    {
      for (o=0;o<u;o++)
      {
        if ((n<cut)&&(o<cut))
        {
          if (g_cut_map[cut][n][o]==' ') continue;
        }

        if ((12-n-1<cut)&&(o<cut))
        {
          if (g_cut_map[cut][12-n-1][o]==' ') continue;
        }

        if ((12-n-1<cut)&&(u-o-1<cut))
        {
          if (g_cut_map[cut][12-n-1][u-o-1]==' ') continue;
        }

        if ((n<cut)&&(u-o-1<cut))
        {
          if (g_cut_map[cut][n][u-o-1]==' ') continue;
        }
        
        deb_set_dot(p+n+1,t+o,64,64,64);
      }
    }
    
  }

  return(0);
}

int g_init(void)
{
  g_dirview_ptr=0;
  g_scrollbar_ptr=0;
  g_button_ptr=0;
  g_label_ptr=0;
  g_lineedit_ptr=0;
  g_checkbox_ptr=0;
  g_radiobutton_ptr=0;
  
  g_cut_init();
  
  smg_get_read_ini();

  return(0);
}

int g_detect_click(int x,int y)
{
  int i,j;
  
  g_detect_ptr1=0;
  g_detect_ptr2=0;
  g_detect_ptr3=0;
  g_detect_ptr4=0;
  
  for (i=1;i<7;i++)
  {
    if (i==1)
    {
      for (j=0;j<g_dirview_ptr;j++)
      {
        if ((g_dirview_posi[j][0]<=x)&&(g_dirview_posi[j][1]<=y)&&(x<g_dirview_posi[j][0]+g_dirview_posi[j][2])&&(y<g_dirview_posi[j][1]+g_dirview_posi[j][3]))
        {
          if (x<g_dirview_posi[j][0]+g_dirview_posi[j][2]-14)
          {
            g_detect_ptr3=1;
            g_detect_ptr4=(y-g_dirview_posi[j][1])/14;
          }
          else if (g_dirview_top==0)
          {
            g_detect_ptr3=0;
          }
          else if (y<g_dirview_top)
          {
            g_detect_ptr3=2;
          }
          else if (y<g_dirview_bottom)
          {
            g_detect_ptr3=3;
          }
          else
          {
            g_detect_ptr3=4;
          }
          
          g_detect_ptr1=i;
          g_detect_ptr2=j;
          
          g_focus_ptr1=g_detect_ptr1;
          g_focus_ptr2=g_detect_ptr2;
          g_focus_ptr3=g_detect_ptr3;
          g_focus_ptr4=g_detect_ptr4;
          
          return(0);
        }
      }
    }
    
    if (i==2)
    {
      for (j=0;j<g_scrollbar_ptr;j++)
      {
        if ((g_scrollbar_posi[j][0]<=x)&&(g_scrollbar_posi[j][1]<=y)&&(x<g_scrollbar_posi[j][0]+g_scrollbar_posi[j][2])&&(y<g_scrollbar_posi[j][1]+g_scrollbar_posi[j][3]))
        {
          if (g_scrollbar_delete[j]!=0) continue;
          if (g_scrollbar_enable[j]!=1) continue;
          
          g_detect_ptr1=i;
          g_detect_ptr2=j;
          g_detect_ptr3=x-g_scrollbar_posi[j][0];

          g_focus_ptr1=g_detect_ptr1;
          g_focus_ptr2=g_detect_ptr2;
          g_focus_ptr3=g_detect_ptr3;

          return(0);
        }
      }
    }
    
    if (i==3)
    {
      for (j=0;j<g_button_ptr;j++)
      {
        if ((g_button_posi[j][0]<=x)&&(g_button_posi[j][1]<=y)&&(x<g_button_posi[j][0]+g_button_posi[j][2])&&(y<g_button_posi[j][1]+g_button_posi[j][3]))
        {
          if (g_button_delete[j]!=0) continue;
          if (g_button_enable[j]!=1) continue;
          
          g_detect_ptr1=i;
          g_detect_ptr2=j;

          g_focus_ptr1=g_detect_ptr1;
          g_focus_ptr2=g_detect_ptr2;

          return(0);
        }
      }
    }

    if (i==4)
    {
      for (j=0;j<g_lineedit_ptr;j++)
      {
        if ((g_lineedit_posi[j][0]<=x)&&(g_lineedit_posi[j][1]<=y)&&(x<g_lineedit_posi[j][0]+g_lineedit_posi[j][2])&&(y<g_lineedit_posi[j][1]+g_lineedit_posi[j][3]))
        {
          if (g_lineedit_delete[j]!=0) continue;
          if (g_lineedit_enable[j]!=1) continue;
          
          g_detect_ptr1=i;
          g_detect_ptr2=j;
          g_detect_ptr3=x - g_lineedit_posi[j][0] ;

          g_focus_ptr1=g_detect_ptr1;
          g_focus_ptr2=g_detect_ptr2;

          return(0);
        }
      }
    }

    if (i==5)
    {
      for (j=0;j<g_checkbox_ptr;j++)
      {
        if ((g_checkbox_posi[j][0]<=x)&&(g_checkbox_posi[j][1]<=y)&&(x<g_checkbox_posi[j][0]+g_checkbox_posi[j][2])&&(y<g_checkbox_posi[j][1]+g_checkbox_posi[j][3]))
        {
          if (g_checkbox_delete[j]!=0) continue;
          if (g_checkbox_enable[j]!=1) continue;
          
          g_detect_ptr1=i;
          g_detect_ptr2=j;

          g_focus_ptr1=g_detect_ptr1;
          g_focus_ptr2=g_detect_ptr2;

          return(0);
        }
      }
    }

    if (i==6)
    {
      for (j=0;j<g_radiobutton_ptr;j++)
      {
        if ((g_radiobutton_posi[j][0]<=x)&&(g_radiobutton_posi[j][1]<=y)&&(x<g_radiobutton_posi[j][0]+g_radiobutton_posi[j][2])&&(y<g_radiobutton_posi[j][1]+g_radiobutton_posi[j][3]))
        {
          if (g_radiobutton_delete[j]!=0) continue;
          if (g_radiobutton_enable[j]!=1) continue;
          
          g_detect_ptr1=i;
          g_detect_ptr2=j;

          g_focus_ptr1=g_detect_ptr1;
          g_focus_ptr2=g_detect_ptr2;

          return(0);
        }
      }
    }

  }
   
  return(0);     
}

int g_detect_motion(int x,int y)
{
  int i,j;
  
  g_detect_motion_ptr1=0;
  g_detect_motion_ptr2=0;
  g_detect_motion_ptr3=0;
  g_detect_motion_ptr4=0;
  
  for (i=1;i<5;i++)
  {
    if (i==1)
    {
      for (j=0;j<g_dirview_ptr;j++)
      {
        if ((g_dirview_posi[j][0]<=x)&&(g_dirview_posi[j][1]<=y)&&(x<g_dirview_posi[j][0]+g_dirview_posi[j][2])&&(y<g_dirview_posi[j][1]+g_dirview_posi[j][3]))
        {
          if (x<g_dirview_posi[j][0]+g_dirview_posi[j][2]-14)
          {
            g_detect_motion_ptr3=1;
            g_detect_motion_ptr4=(y-g_dirview_posi[j][1])/14;
          }
          else if (g_dirview_top==0)
          {
            g_detect_motion_ptr3=0;
          }
          else if (y<g_dirview_top)
          {
            g_detect_motion_ptr3=2;
          }
          else if (y<g_dirview_bottom)
          {
            g_detect_motion_ptr3=3;
          }
          else
          {
            g_detect_motion_ptr3=4;
          }
          
          g_detect_motion_ptr1=i;
          g_detect_motion_ptr2=j;
          
          return(0);
        }
      }
    }
    
    if (i==2)
    {
      for (j=0;j<g_scrollbar_ptr;j++)
      {
        if ((g_scrollbar_posi[j][0]<=x)&&(g_scrollbar_posi[j][1]<=y)&&(x<g_scrollbar_posi[j][0]+g_scrollbar_posi[j][2])&&(y<g_scrollbar_posi[j][1]+g_scrollbar_posi[j][3]))
        {
          if (g_scrollbar_delete[j]!=0) continue;

          g_detect_motion_ptr1=i;
          g_detect_motion_ptr2=j;
          g_detect_motion_ptr3=x-g_scrollbar_posi[j][0];

          return(0);
        }
      }
    }
    
    if (i==3)
    {
      for (j=0;j<g_button_ptr;j++)
      {
        if ((g_button_posi[j][0]<=x)&&(g_button_posi[j][1]<=y)&&(x<g_button_posi[j][0]+g_button_posi[j][2])&&(y<g_button_posi[j][1]+g_button_posi[j][3]))
        {
          if (g_button_delete[j]!=0) continue;

          g_detect_motion_ptr1=i;
          g_detect_motion_ptr2=j;

          return(0);
        }
      }
    }

    if (i==4)
    {
      for (j=0;j<g_lineedit_ptr;j++)
      {
        if ((g_lineedit_posi[j][0]<=x)&&(g_lineedit_posi[j][1]<=y)&&(x<g_lineedit_posi[j][0]+g_lineedit_posi[j][2])&&(y<g_lineedit_posi[j][1]+g_lineedit_posi[j][3]))
        {
          if (g_lineedit_delete[j]!=0) continue;

          g_detect_motion_ptr1=i;
          g_detect_motion_ptr2=j;

          return(0);
        }
      }
    }

  }
   
  return(0);     
}

int g_create_label(int x,int y,int w,int h,int black_label,int color1,int color2,int color3,char *text,int text_size)
{
  int i,j,k;
  int find,find_ptr;
  
  find=0;
  find_ptr=(-1);
  
  for (k=0;k<g_label_ptr;k++)
  {
    if (g_label_delete[k]==1)
    {
      find=1;
      find_ptr=k;
      break;
    }
  }
  
  if (find==0)
  {
    if ((g_label_ptr<0)||(g_label_ptr>=G_LABEL_NUM)) return(-1);
  
    find_ptr=g_label_ptr;
  }
  
  g_label_posi[find_ptr][0]=x;
  g_label_posi[find_ptr][1]=y;
  g_label_posi[find_ptr][2]=w;
  g_label_posi[find_ptr][3]=h;
  
  g_label_color[find_ptr][0]=color1;
  g_label_color[find_ptr][1]=color2;
  g_label_color[find_ptr][2]=color3;

  g_label_black[find_ptr]=black_label;
  g_label_enable[find_ptr]=1;

  if (text_size<FN_SIZE) i=text_size;
  else i=FN_SIZE;

  if (deb_str_has_null(text,i)!=1) return(-1);
  
  strcpy(g_label_text[find_ptr],text);
  
  j=find_ptr;
  if (find==0) g_label_ptr++;
  g_label_delete[j]=0;
  
  return(j);
}

int g_move_label(int lb_ptr,int x,int y,int w,int h)
{
  if ((lb_ptr<0)||(lb_ptr>=G_LABEL_NUM)||(lb_ptr>=g_label_ptr)) return(1);
  
  g_label_posi[lb_ptr][0]=x;
  g_label_posi[lb_ptr][1]=y;
  g_label_posi[lb_ptr][2]=w;
  g_label_posi[lb_ptr][3]=h;
  
  return(0);
}

int g_enable_label(int lb_ptr,int enable)
{
  if ((lb_ptr<0)||(lb_ptr>=G_LABEL_NUM)||(lb_ptr>=g_label_ptr)) return(1);
  
  g_label_enable[lb_ptr]=enable;
  
  return(0);
}

int g_delete_label(int lb_ptr)
{
  if ((lb_ptr<0)||(lb_ptr>=G_LABEL_NUM)||(lb_ptr>=g_label_ptr)) return(1);
  
  g_label_delete[lb_ptr]=1;
  
  return(0);
}

int g_set_label_text(int lb_ptr,char *str,int str_size)
{
  int i;

  if ((lb_ptr<0)||(lb_ptr>=G_LABEL_NUM)||(lb_ptr>=g_label_ptr)) return(1);

  if (str_size<FN_SIZE) i=str_size;
  else i=FN_SIZE;

  if (deb_str_has_null(str,i)!=1) return(1);
  
  strcpy(g_label_text[lb_ptr],str);

  return(0);  
}

int g_set_label_text_color(int lb_ptr,int black_or_white_letter,int color1,int color2,int color3)
{
  if ((lb_ptr<0)||(lb_ptr>=G_LABEL_NUM)||(lb_ptr>=g_label_ptr)) return(1);
  
  g_label_text_bl[lb_ptr]=black_or_white_letter;
  
  g_label_text_color[lb_ptr][0]=color1;
  g_label_text_color[lb_ptr][1]=color2;
  g_label_text_color[lb_ptr][2]=color3;
  
  return(0);
}

int g_paint_label(int vid)
{
  int			i,j,jb,k,l,m,m2,n,o,p,q,r;
  int			x,y;
  unsigned char		c1,c2,c3;
  int  			i0,i1,i2,i3,i4;
  int                      j1,j2,j3,j4;
  int			len;
  char 			str[FN_SIZE];

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);

  for (i=0;i<g_label_ptr;i++)
  {
    if (g_label_delete[i]==1) continue;
    
    j=g_label_posi[i][0];
    k=g_label_posi[i][1];
    l=g_label_posi[i][2];
    m=g_label_posi[i][3];
    
    i1=g_label_color[i][0];
    i2=g_label_color[i][1];
    i3=g_label_color[i][2];
    i4=255;


    for (n=0;n<l;n++)
    {
      for (o=0;o<m;o++)
      {
      
        if (g_label_enable[i]==1)
        {
          if (vid)
          {
                  SDL_SetRenderDrawColor(renderer, i1, i2, i3, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
          }
          else
          {
                  deb_set_dot(j+n,k+o,i1,i2,i3);
          }
        }
        else
        {
          if (g_label_black[i]==0)
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, i1*7/10, i2*7/10, i3*7/10, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,i1*7/10,i2*7/10,i3*7/10);
            }
          }
          else
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, 255-(255-i1)*7/10, 255-(255-i2)*7/10, 255-(255-i3)*7/10, i4);

                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10);
            }
          }
        }
    
    
      }
    }
    
    strcpy(str,g_label_text[i]);
    
    p=u_strlen(str,FN_SIZE);
    q=(l-p*7)/2;
    if (q<0) continue;
    
    r=(m-14)/2;
    if (r<0) continue;
    
    
    

    x=j+q;
    y=k+r;

    len=p;
    jb=0;

    while(jb<len)
    {
      if (strlen(str)>jb+0) c1=(unsigned char)str[jb+0];
      else c1=0;
    
      if (strlen(str)>jb+1) c2=(unsigned char)str[jb+1];
      else c2=0;
    
      if (strlen(str)>jb+2) c3=(unsigned char)str[jb+2];
      else c3=0;
    
      u_get_char_bmp(c1,c2,c3);

      if ((u_err==0)&&(u_err2==0))
      {
          for (m2=0;m2<u_char_size_x;m2++)
          {
            for (n=0;n<u_char_size_y;n++)
            {
              if ((m2<0)||(m2>=14)) continue;
              if ((n<0)||(n>=14)) continue;
        
              i0=u_char_bmp[m2][n];
        
        
        
              if (g_label_enable[i]==1)
              {
                if (g_label_text_bl[i]==0)  //white letter
                {
                  i0=255-i0;
        
                  if (i0>=96)
                  {
                    j1=i0*g_label_text_color[i][0]/255;
                    j2=i0*g_label_text_color[i][1]/255;
                    j3=i0*g_label_text_color[i][2]/255;
                    j4=255;
                  }
                  else
                  {
                    j1=i1;
                    j2=i2;
                    j3=i3;
                    j4=255;
                  }
                }
                else //black letter
                {
                  if (i0<160)
                  {
                    j1=255-(255-i0)*(255-g_label_text_color[i][0])/255;
                    j2=255-(255-i0)*(255-g_label_text_color[i][1])/255;
                    j3=255-(255-i0)*(255-g_label_text_color[i][2])/255;
                    j4=i4;
                  }
                  else
                  {
                    j1=i1;
                    j2=i2;
                    j3=i3;
                    j4=i4;
                  }
                }
        
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, j4);
                  fill_rectangle(x+m2, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m2,y+n,j1,j2,j3);
                }
              }
              else
              {
                if (g_label_text_bl[i]==0)  //white letter
                {
                  i0=255-i0;
        
                  if (i0>=96)
                  {
                    j1=(i0*g_label_text_color[i][0]/255)*7/10;
                    j2=(i0*g_label_text_color[i][1]/255)*7/10;
                    j3=(i0*g_label_text_color[i][2]/255)*7/10;
                    j4=255;
                  }
                  else
                  {
                    if (g_label_black[i]==0)
                    {
                      j1=i1*7/10;
                      j2=i2*7/10;
                      j3=i3*7/10;
                      j4=255;
                    }
                    else
                    {
                      j1=255-(255-i1)*7/10;
                      j2=255-(255-i2)*7/10;
                      j3=255-(255-i3)*7/10;
                      j4=255;
                    }
                  }
                }
                else //black letter
                {
                  if (i0<160)
                  {
                    j1=255-((255-i0)*(255-g_label_text_color[i][0])/255)*7/10;
                    j2=255-((255-i0)*(255-g_label_text_color[i][1])/255)*7/10;
                    j3=255-((255-i0)*(255-g_label_text_color[i][2])/255)*7/10;
                    j4=i4;
                  }
                  else
                  {
                    j1=i1*7/10;
                    j2=i2*7/10;
                    j3=i3*7/10;
                    j4=i4;
                  }
                }
        
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, j4);
                  fill_rectangle(x+m2, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m2,y+n,j1,j2,j3);
                }
              }
              
              
              
            }
          }
    
          x=x+u_char_size_x;
          jb=jb+u_nb;
          continue;
      }
      else
      {
          x=x+u_char_size_x;
          jb=jb+u_nb;
          continue;
      }
    }
    
    
    
    
  }

  return(0);
}

int g_load_icon(void)
{
  int   i,j,n;
  int   p1,p2,p3,p4,p5;    
  char  str1[300];
  char  *str2;
  FILE *fp1;

  fp1=fopen("./utf8_bmp/icons.txt","r");
  if (fp1==NULL)
  {
    printf("Open file './utf8_bmp/icons.txt' error\n");
    return(1);
  }

  p1=0;
  p2=0;
  p3=0;
  p4=0;
  p5=0;
  
  while (!feof(fp1))
  {
    str2=fgets(str1,300,fp1);

    if (str2==NULL) break;
    
    if (str1[0]=='/') continue;
    if (str1[0]<' ') continue;    

    string_trim_nos(str1);
    if (str1[0]==0) continue;
    
    i=strlen(str1);
    j=0;
    n=0;
    
    while(j<i)
    {
      if (str1[j]==' ')
      {
        j++;
        continue;
      }
      else if ((str1[j]>='0')&&(str1[j]<='9'))
      {
        n=n*10+str1[j]-'0';
        j++;
      }
      else if (str1[j]==',')
      {
        if (p5==0)
        {
          g_icons_mask[p1][p2][p3]=n;
        
          n=0;
        
          p3++;
          if (p3>13)
          {
            p3=0;
            p2++;
            if (p2>13)
            {
              p2=0;
              p5=1;
            }
          }
        
          j++;
        }
        else
        {
          g_icons[p1][p2][p3][p4]=n;
        
          n=0;
        
          p3++;
          if (p3>13)
          {
            p3=0;
            p4++;
            if (p4>2)
            {
              p4=0;
              p2++;
              if (p2>13)
              {
                p2=0;
                p1++;
                p5=0;
                if (p1>=8)
                {
                  fclose(fp1);
                  return(0);
                }
              }
            }
          }
        
          j++;
        }
      }
    }
  }
  
  fclose(fp1);

  return(0);
}

int g_paint_icon(int xx,int yy,int ic)
{
  unsigned char i1,i2,i3;
  int   m,n;

  if (deb_tx_locked!=1) return(0);
  if ((ic<1)||(ic>8)) return(1);

  for (m=0;m<14;m++)
  {
    for (n=0;n<14;n++)
    {
      i1=g_icons[ic-1][m][n][0];
      i2=g_icons[ic-1][m][n][1];
      i3=g_icons[ic-1][m][n][2];
            
      if (g_icons_mask[ic-1][m][n]>0) deb_set_dot(xx+n,yy+m,i1,i2,i3);
    }
  }
        
  return(0);
}

char m301_str1[300];

int g_icon_id(int dir,char *p_str,int p_str_size)
{
  if (dir==1) return(2);
  
  deb_filename_ext(p_str,p_str_size,m301_str1,300);

  if (strcmp(m301_str1,"aac")==0 ) return(4);
  if (strcmp(m301_str1,"ape")==0 ) return(4);
  if (strcmp(m301_str1,"asf")==0 ) return(4);
  if (strcmp(m301_str1,"flac")==0) return(4);
  if (strcmp(m301_str1,"m4a")==0 ) return(4);
  if (strcmp(m301_str1,"mp2")==0 ) return(4);
  if (strcmp(m301_str1,"mp3")==0 ) return(4);
  if (strcmp(m301_str1,"mpc")==0 ) return(4);
  if (strcmp(m301_str1,"ogg")==0 ) return(4);
  if (strcmp(m301_str1,"tta")==0 ) return(4);
  if (strcmp(m301_str1,"wav")==0 ) return(4);
  if (strcmp(m301_str1,"wma")==0 ) return(4);
  if (strcmp(m301_str1,"wv")==0  ) return(4);

  if (strcmp(m301_str1,"rm")==0  ) return(5);
  if (strcmp(m301_str1,"rmvb")==0) return(5);
  if (strcmp(m301_str1,"wmv")==0 ) return(5);
  if (strcmp(m301_str1,"d9")==0  ) return(5);
  if (strcmp(m301_str1,"mpg")==0 ) return(5);
  if (strcmp(m301_str1,"vob")==0 ) return(5);
  if (strcmp(m301_str1,"ts")==0  ) return(5);
  if (strcmp(m301_str1,"mov")==0 ) return(5);
  if (strcmp(m301_str1,"avi")==0 ) return(5);
  if (strcmp(m301_str1,"m4v")==0 ) return(5);
  if (strcmp(m301_str1,"mp4")==0 ) return(5);
  if (strcmp(m301_str1,"mpeg")==0) return(5);
  if (strcmp(m301_str1,"mkv")==0 ) return(5);
  
  return(3);
}

int g_create_lineedit(const char *type,int x,int y,int w,int h,int black_lineedit,int color1,int color2,int color3,char *txt,int len_screen,int len_real,int dec)
{
  int i,j,k;
  int find,find_ptr;
  
  find=0;
  find_ptr=(-1);
  
  for (k=0;k<g_lineedit_ptr;k++)
  {
    if (g_lineedit_delete[k]==1)
    {
      find=1;
      find_ptr=k;
      break;
    }
  }
  
  if (find==0)
  {
    if ((g_lineedit_ptr<0)||(g_lineedit_ptr>=G_LINEEDIT_NUM)) return(-1);
  
    find_ptr=g_lineedit_ptr;
  }
  
  g_lineedit_posi[find_ptr][0]=x;
  g_lineedit_posi[find_ptr][1]=y;
  g_lineedit_posi[find_ptr][2]=w;
  g_lineedit_posi[find_ptr][3]=h;
  
  g_lineedit_color[find_ptr][0]=color1;
  g_lineedit_color[find_ptr][1]=color2;
  g_lineedit_color[find_ptr][2]=color3;

  g_lineedit_black[find_ptr]=black_lineedit;
  g_lineedit_enable[find_ptr]=1;

  if (len_real<FN_SIZE) i=len_real;
  else i=FN_SIZE;

  if (deb_str_has_null(txt,i+1)!=1) return(-1);
  
  strcpy(g_lineedit_text[find_ptr],txt);
  
  if (strcmp(type,"text")==0)  //text
  {
    smg_get_read(0,0,0,"",
              txt,len_screen,'c',
	      'g',len_real,0,
	       0,0,0,
	       find_ptr);
	       
    g_lineedit_type[find_ptr]='c';
  }
  else if (strcmp(type,"number")==0) //number string
  {
    smg_get_read(0,0,0,"",
              txt,len_screen,'n',
	      'g',len_real,dec,
	       0,0,0,
	       find_ptr);
	       
    g_lineedit_type[find_ptr]='n';
  }
  else if (strcmp(type,"password")==0) //password
  {
    smg_get_read(0,0,0,"",
              txt,len_screen,'p',
	      'g',len_real,0,
	       0,0,0,
	       find_ptr);
	       
    g_lineedit_type[find_ptr]='p';
  }
  else return(-1);
  
  j=find_ptr;
  if (find==0) g_lineedit_ptr++;
  g_lineedit_delete[j]=0;
  
  return(j);
}

int g_move_lineedit(int le_ptr,int x,int y,int w,int h)
{
  if ((le_ptr<0)||(le_ptr>=G_LINEEDIT_NUM)||(le_ptr>=g_lineedit_ptr)) return(1);
  
  g_lineedit_posi[le_ptr][0]=x;
  g_lineedit_posi[le_ptr][1]=y;
  g_lineedit_posi[le_ptr][2]=w;
  g_lineedit_posi[le_ptr][3]=h;
  
  return(0);
}

int g_enable_lineedit(int ptr,int enable)
{
  if ((ptr<0)||(ptr>=G_LINEEDIT_NUM)||(ptr>=g_lineedit_ptr)) return(1);
  
  g_lineedit_enable[ptr]=enable;
  
  return(0);
}

int g_delete_lineedit(int ptr)
{
  if ((ptr<0)||(ptr>=G_LINEEDIT_NUM)||(ptr>=g_lineedit_ptr)) return(1);
  
  g_lineedit_delete[ptr]=1;
  
  if (g_lineedit_current_id==ptr) g_lineedit_current_id=(-1);
  
  return(0);
}

int g_set_lineedit_text(int le_ptr,char *str,int str_size)
{
  int i;

  if ((le_ptr<0)||(le_ptr>=G_LINEEDIT_NUM)||(le_ptr>=g_lineedit_ptr)) return(1);

  if (str_size<FN_SIZE) i=str_size;
  else i=FN_SIZE;

  if (deb_str_has_null(str,i)!=1) return(1);
  
  strcpy(g_lineedit_text[le_ptr],str);

  return(0);  
}

int g_set_lineedit_text_color(int le_ptr,int black_letter,int color1,int color2,int color3)
{
  if ((le_ptr<0)||(le_ptr>=G_LINEEDIT_NUM)||(le_ptr>=g_lineedit_ptr)) return(1);
  
  g_lineedit_text_bl[le_ptr]=black_letter;
  
  g_lineedit_text_color[le_ptr][0]=color1;
  g_lineedit_text_color[le_ptr][1]=color2;
  g_lineedit_text_color[le_ptr][2]=color3;
  
  return(0);
}

int g_paint_lineedit(int vid)
{
  int			i,j,jb,k,l,m,m2,n,o,p,q,r;
  int			x,y;
  unsigned char		c1,c2,c3;
  int  			i0,i1,i2,i3,i4;
  int                      j1,j2,j3,j4;
  int                      k1,k2,k3,k4;
  int			len;
  char 			str[FN_SIZE];
  char 			str2[FN_SIZE];

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);
  
  for (i=0;i<g_lineedit_ptr;i++)
  {
    if (g_lineedit_delete[i]==1) continue;
    
    j=g_lineedit_posi[i][0];
    k=g_lineedit_posi[i][1];
    l=g_lineedit_posi[i][2];
    m=g_lineedit_posi[i][3];
    
    i1=g_lineedit_color[i][0];
    i2=g_lineedit_color[i][1];
    i3=g_lineedit_color[i][2];
    i4=255;
    
    k1=255-i1;
    k2=255-i2;
    k3=255-i3;
    k4=i4;
    
    for (n=0;n<l;n++)
    {
      for (o=0;o<m;o++)
      {
        if (g_lineedit_enable[i]==1)
        {
          if (vid)
          {
                  SDL_SetRenderDrawColor(renderer, i1, i2, i3, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
          }
          else
          {
                  deb_set_dot(j+n,k+o,i1,i2,i3);
          }
        }
        else
        {
          if (g_lineedit_black[i]==0)
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, i1*7/10, i2*7/10, i3*7/10, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,i1*7/10,i2*7/10,i3*7/10);
            }
          }
          else
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, 255-(255-i1)*7/10, 255-(255-i2)*7/10, 255-(255-i3)*7/10, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10);
            }
          }
        }
      }
    }

    for (n=0;n<l;n++)
    {
      if (vid)
      {
                  SDL_SetRenderDrawColor(renderer, k1, k2, k3, k4);
                  fill_rectangle(j+n, k, 1, 1);
                  fill_rectangle(j+n, k+m,1, 1);
      }
      else
      {
                  deb_set_dot(j+n,k,k1,k2,k3);
                  deb_set_dot(j+n,k+m,k1,k2,k3);
      }
    }
    
    for (o=0;o<m;o++)
    {
      if (vid)
      {
                  SDL_SetRenderDrawColor(renderer, k1, k2, k3, k4);
                  fill_rectangle(j, k+o, 1, 1);
                  fill_rectangle(j+l,k+o, 1, 1);
      }
      else
      {
                  deb_set_dot(j,k+o,k1,k2,k3);
                  deb_set_dot(j+l,k+o,k1,k2,k3);
      }
    }

    get_smg_string(i,g_lineedit_text[i],FN_SIZE);

    //printf("lineedit=%s,\n",g_lineedit_text[i]);

    if ((g_focus_ptr1==4)&&(g_focus_ptr2==i))
    {
      str[0]=0;
      
      for (p=smg_p1;p<FN_SIZE;p++)
      {
        str[p-smg_p1+0]=g_lineedit_text[i][p];
        str[p-smg_p1+1]=0;
      }
    }
    else
    {
      strcpy(str,g_lineedit_text[i]);
    }
    
    //printf("str=%s,\n",str);

    //p=u_strlen(str,FN_SIZE);
    q=(l-4)/7;
    if (q<0) continue;
    
    u_strcut(str,FN_SIZE,str2,FN_SIZE,q,1);
    
    //printf("str2=%s,\n",str2);
    
    

    x=j+2;
    y=k+1;
    
    len=(int)strlen(str2);
    jb=0;

    while(jb<len)
    {
      if (g_lineedit_type[i]=='p')
      {
        if (g_lineedit_enable[i]==1)
        {
            i0=255;
        
            j1=i0*g_lineedit_text_color[i][0]/255;
            j2=i0*g_lineedit_text_color[i][1]/255;
            j3=i0*g_lineedit_text_color[i][2]/255;
            j4=255;
        }
        else
        {
          if (g_lineedit_black[i]==0)
          {
            i0=255;
        
            j1=(i0*g_lineedit_text_color[i][0]/255)*7/10;
            j2=(i0*g_lineedit_text_color[i][1]/255)*7/10;
            j3=(i0*g_lineedit_text_color[i][2]/255)*7/10;
            j4=255;
          }
          else
          {
            i0=255;
        
            j1=255-((255-i0)*(255-g_lineedit_text_color[i][0])/255)*7/10;
            j2=255-((255-i0)*(255-g_lineedit_text_color[i][1])/255)*7/10;
            j3=255-((255-i0)*(255-g_lineedit_text_color[i][2])/255)*7/10;
            j4=255;
          }
        }
    
        g_draw_circle(x+7/2,y+7/2,7/2,vid,j1,j2,j3,1);
      
        x=x+7;
        jb=jb+1;
        continue;
      }
    
      if (strlen(str)>jb+0) c1=(unsigned char)str[jb+0];
      else c1=0;
    
      if (strlen(str)>jb+1) c2=(unsigned char)str[jb+1];
      else c2=0;
    
      if (strlen(str)>jb+2) c3=(unsigned char)str[jb+2];
      else c3=0;
    
      u_get_char_bmp(c1,c2,c3);

      if ((u_err==0)&&(u_err2==0))
      {
          for (m2=0;m2<u_char_size_x;m2++)
          {
            for (n=0;n<u_char_size_y;n++)
            {
              if ((m2<0)||(m2>=14)) continue;
              if ((n<0)||(n>=14)) continue;
        
              i0=u_char_bmp[m2][n];
        
              if (g_lineedit_enable[i]==1)
              {

                if (g_lineedit_text_bl[i]==0)  //white letter
                {
                  i0=255-i0;
        
                  if (i0>=96)
                  {
                    j1=i0*g_lineedit_text_color[i][0]/255;
                    j2=i0*g_lineedit_text_color[i][1]/255;
                    j3=i0*g_lineedit_text_color[i][2]/255;
                    j4=255;
                  }
                  else
                  {
                    j1=i1;
                    j2=i2;
                    j3=i3;
                    j4=255;
                  }
                }
                else //black letter
                {
                  if (i0<160)
                  {
                    j1=255-(255-i0)*(255-g_lineedit_text_color[i][0])/255;
                    j2=255-(255-i0)*(255-g_lineedit_text_color[i][1])/255;
                    j3=255-(255-i0)*(255-g_lineedit_text_color[i][2])/255;
                    j4=255;
                  }
                  else
                  {
                    j1=i1;
                    j2=i2;
                    j3=i3;
                    j4=255;
                  }
                }
        
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, j4);
                  fill_rectangle(x+m2, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m2,y+n,j1,j2,j3);
                }
              
              }
              else
              {

                if (g_lineedit_text_bl[i]==0)  //white letter
                {
                  i0=255-i0;
        
                  if (i0>=96)
                  {
                    j1=(i0*g_lineedit_text_color[i][0]/255)*7/10;
                    j2=(i0*g_lineedit_text_color[i][1]/255)*7/10;
                    j3=(i0*g_lineedit_text_color[i][2]/255)*7/10;
                    j4=255;
                  }
                  else
                  {
                    if (g_lineedit_black[i]==0)
                    {
                      j1=i1*7/10;
                      j2=i2*7/10;
                      j3=i3*7/10;
                      j4=255;
                    }
                    else
                    {
                      j1=255-(255-i1)*7/10;
                      j2=255-(255-i2)*7/10;
                      j3=255-(255-i3)*7/10;
                      j4=255;
                    }
                  }
                }
                else //black letter
                {
                  if (i0<160)
                  {
                    j1=255-((255-i0)*(255-g_lineedit_text_color[i][0])/255)*7/10;
                    j2=255-((255-i0)*(255-g_lineedit_text_color[i][1])/255)*7/10;
                    j3=255-((255-i0)*(255-g_lineedit_text_color[i][2])/255)*7/10;
                    j4=255;
                  }
                  else
                  {
                    j1=i1*7/10;
                    j2=i2*7/10;
                    j3=i3*7/10;
                    j4=255;
                  }
                }
        
                if (vid)
                {
                  SDL_SetRenderDrawColor(renderer, j1, j2, j3, j4);
                  fill_rectangle(x+m2, y+n, 1, 1);
                }
                else
                {
                  deb_set_dot(x+m2,y+n,j1,j2,j3);
                }

              }
            }
          }
    
          x=x+u_char_size_x;
          jb=jb+u_nb;
          continue;
      }
      else
      {
          x=x+u_char_size_x;
          jb=jb+u_nb;
          continue;
      }
    }
    
    if ((g_focus_ptr1==4)&&(g_focus_ptr2==i)&&(g_caret_show==1))
    {
      //j=g_lineedit_posi[i][0];
      //k=g_lineedit_posi[i][1];
      //l=g_lineedit_posi[i][2];
      //m=g_lineedit_posi[i][3];
    
      i1=g_lineedit_text_color[i][0];
      i2=g_lineedit_text_color[i][1];
      i3=g_lineedit_text_color[i][2];
      i4=255;

      r=get_smg_posi(i);
      //q=j+2+r*7;
    
      for (o=0;o<14;o++)
      {
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, i1, i2, i3, i4);
                  fill_rectangle(j+2+r*7, k+1+o, 1, 1);

        }
        else
        {
                  deb_set_dot(j+2+r*7,k+1+o,i1,i2,i3);
        }
      }

    }

  }

  return(0);
}

int g_lineedit_set_posi(int le_ptr,int x)
{
  int  i,j;
  char str[FN_SIZE];
  
  if ((le_ptr<0)||(le_ptr>=G_LINEEDIT_NUM)||(le_ptr>=g_lineedit_ptr)) return(1);

  i=(x-2)/7;
  if (i<0) i=0;
  
  j=(g_lineedit_posi[le_ptr][2]-4)/7;
  if (j<0) j=0;
  
  if (i>j) i=j;
  
  get_smg_string(le_ptr,str,FN_SIZE);
  if (smg_p1+i>strlen(str)) i=strlen(str)-smg_p1;
  if (i<0) i=0;
  
  set_smg_posi(le_ptr,i);
  
  return(0);
}



int  smg_get_read_ini()
{
  int i;

  for (i=0;i<SMG_GET_NUM;i++) set_smg_read_id(i,0);
  smg_insert=1;

  return(0);
}

int  tst_smg_get_read()
{
  int  i;
  char str1[SMG_SIZE];
  char str2[SMG_SIZE*2];

  deb_record(" tst_smg_get_read()");

  for (i=0;i<=12;i++)
  {
    get_smg_string(i,str1,SMG_SIZE);

    sprintf(str2,"ptr=%d,smg_line=%d,colu=%d,data=%s,type=%c,len=%d,len2=%d,dec=%d,link=%d,color=%d,id=%d,"
	    ,i,get_smg_line(i),get_smg_colu(i),str1
	    ,get_smg_type(i),get_smg_dlen(i),get_smg_dlen_real(i)
	    ,get_smg_ddec(i),get_smg_link(i),get_smg_color(i)
	    ,get_smg_read_id(i));

    deb_record(str2);
  }

  return(0);
}

int  smg_get_read(int p_wptr,int scrn_l,int scrn_c,const char *atten,
                     char *string,int datalen,char datatype,
		     char command,int datalen_real,int datadec,
		     char link,int color,int posi,
		     int  array_ptr)
{  
  int   key;
  int   i,j,k,w,u;
  char  str1[SMG_SIZE];
  char  str2[SMG_SIZE];
  int   pos[300];

  if (command=='g')
  {
    if (array_ptr<0)        return(1);
    if (array_ptr>=SMG_GET_NUM) return(1);

    
    //sprintf(str4, "smg_hnd_id=%d,wp=%d,", smg_hnd_id, wp);
    //deb_record("get read ,g");
    

    smg_ptr=array_ptr;

    set_smg_line(smg_ptr,scrn_l);
    set_smg_colu(smg_ptr,scrn_c);
    set_smg_atte(smg_ptr,atten,strlen(atten)+1);
    set_smg_type(smg_ptr,datatype);
    set_smg_dlen(smg_ptr,datalen);
    set_smg_dlen_real(smg_ptr,datalen_real);
    set_smg_ddec(smg_ptr,datadec);
    set_smg_link(smg_ptr,link);
    set_smg_color(smg_ptr,color);
    set_smg_posi(smg_ptr,posi);
    set_smg_string(smg_ptr,string,strlen(string)+1);
    //set_smg_modi(smg_ptr,0);

    smg_p_y=(int)(smg_edit_ychar*(scrn_l));
    smg_p_x=(int)(smg_edit_xchar*(scrn_c));

    //printf("get read g str=%s,\n",string);

    i=strlen(atten);
/*
    hdc=GetDC(bw_main1.get_smg_hwnd(p_wptr));
    hfont=(HFONT)GetStockObject(OEM_FIXED_FONT);
    holdfont=(HFONT)SelectObject(hdc,hfont);
    TextOut(hdc,smg_p_x,smg_p_y,atten,i);
*/
    smg_p_x=smg_p_x+(int)(i*smg_edit_xchar);

    get_smg_string(smg_ptr,str1,SMG_SIZE);
    i=strlen(str1);
    if (i>datalen) i=datalen;

    u=smg_chs_string_cut(str1,i);
    str1[u]=0;

    strcat(str1," ");
    str1[i]=0;

    //printf("get read g str 2=%s,\n",string);

/*
    HideCaret(bw_main1.get_smg_hwnd(p_wptr));
    TextOut(hdc,smg_p_x,smg_p_y,str1,strlen(str1));
    ShowCaret(bw_main1.get_smg_hwnd(p_wptr));

    SelectObject(hdc,holdfont);
    ReleaseDC(bw_main1.get_smg_hwnd(p_wptr),hdc);
*/
    if (posi>=datalen) posi=datalen-1;
    if (posi<0)        posi=0;



    // get char , avoid half chiness
    get_smg_string(smg_ptr,str1,SMG_SIZE);

    for (k=0;k<300;k++) pos[k]=(-1);

    k=0;
    j=0;
    pos[0]=0;

    while(k<strlen(str1))
    {
      if (str1[k]<0) k=k+2;
      else k=k+1;

      j++;

      pos[j]=k;
    }

    w=0;

    for (k=0;k<300;k++)
    {
      if (pos[k]<0) break;

      if (pos[k]<= posi)
      {
        w=pos[k];
      }

    }

    posi=w;
  }
  else if (command=='r')
  {
    if (array_ptr<0)        return(1);
    if (array_ptr>=SMG_GET_NUM) return(1);


    //sprintf(str4, "into getread()... wptr=%d,",p_wptr);
    //deb_record(str4);

    smg_ptr2=array_ptr;

    if (get_smg_type(smg_ptr2)!='p')
    {
      get_smg_string(smg_ptr2,str1,SMG_SIZE);

      set_smg_data(0,0);

      //printf("in getread(r)... ,string=%s,%c,\n",str1,get_smg_type(smg_ptr2));

      for (i=0;i<get_smg_dlen_real(smg_ptr2);i++)
      {
        if (i<(int)strlen(str1))
        {
          set_smg_data(i+0,str1[i]);
          set_smg_data(i+1,0);
        }
        else
        {
          set_smg_data(i+0,' ');
          set_smg_data(i+1,0);
        }
      }
    }
    else
    {
      get_smg_string(smg_ptr2,str1,SMG_SIZE);

      set_smg_data(0,0);

      //printf("in getread(r)... ,string=%s,%c,\n",str1,get_smg_type(smg_ptr2));

      for (i=0;i<get_smg_dlen_real(smg_ptr2);i++)
      {
        if (i<(int)strlen(str1))
        {
          set_smg_data(i+0,str1[i]);
          set_smg_data(i+1,0);
        }
        else break;
      }
    }

    //get_s_smg_data(str5,SMG_SIZE);
    //sprintf(str4, "in getread()... ,data=%s,dlen_real=%d,",str5,get_smg_dlen_real(smg_ptr2));
    //deb_record(str4);

    if (get_smg_type(smg_ptr2)=='c')
    {
      get_smg_atte(smg_ptr2,str2,SMG_SIZE);

      key=smg_get_text(
p_wptr,
get_smg_line(smg_ptr2),get_smg_colu(smg_ptr2),str2,
get_smg_dlen(smg_ptr2),get_smg_dlen_real(smg_ptr2),get_smg_link(smg_ptr2),
smg_ptr2,get_smg_color(smg_ptr2),get_smg_posi(smg_ptr2));
    }
    else if (get_smg_type(smg_ptr2)=='p')
    {
      get_smg_atte(smg_ptr2,str2,SMG_SIZE);

      key=smg_get_password(
p_wptr,
get_smg_line(smg_ptr2),get_smg_colu(smg_ptr2),str2,
get_smg_dlen(smg_ptr2),get_smg_dlen_real(smg_ptr2),get_smg_link(smg_ptr2),
smg_ptr2,get_smg_color(smg_ptr2),get_smg_posi(smg_ptr2));
    }
    else
    {
      get_smg_atte(smg_ptr2,str2,SMG_SIZE);

      key=smg_get_number(
p_wptr,
get_smg_line(smg_ptr2),get_smg_colu(smg_ptr2),str2,
get_smg_dlen(smg_ptr2),get_smg_dlen_real(smg_ptr2),get_smg_link(smg_ptr2),
smg_ptr2,get_smg_color(smg_ptr2),get_smg_ddec(smg_ptr2),get_smg_posi(smg_ptr2));
    }
      
    set_c_smg_string(smg_ptr2,0,0);
      
    for (i=0;i<get_smg_dlen_real(smg_ptr2);i++)
    {
      set_c_smg_string(smg_ptr2,i+0,get_smg_data(i));
      set_c_smg_string(smg_ptr2,i+1,0);
    }

    if (key!=0)
    {
      if (    (key==127)
            ||(key==SMG_KEY_ESC)
            ||(key==CTRL_W)
	    ||(key>200)
	    ||(key==SMG_KEY_RET)
	    ||((key>0)&&(key<32))  )

      {
	smg_key=key;
	return(1);
      }
      else  if (  (((key>=32)&&(key<127))||(key<0))  &&
                  (smg_confirm==0)  )  
      {
	smg_key=10;
        return(1);
      }
    }
  }

  return(0);
}

int  smg_get_text(int p_wptr,int scrn_l,int scrn_c,char *atten,
		    int datalen,int datalen_real,char link,
		    int ptr,int color,int posi)
{
  char  c1;
  char  str1[SMG_SIZE];
  //char  str2[SMG_SIZE];
  int   n1,n2;
  int   i;
  int   key;
  int   p2,p3;
  int   w,w2;

  int   j,k;
  int   pos[300];

/* --- display attention and data --- */
  n1=strlen(atten);

/* --- input command --- */
  if ( posi >= datalen ) posi= datalen-1;
  if ( posi < 0)         posi=0;
  n2= posi;

  smg_cur1=scrn_c+n1;

  scrn_c =scrn_c+n1+n2;

/* --- pre insert when smg_chns_next==1 --- */
  if (smg_key!=0)
  {
    if (smg_key==410) return(0);
    if (smg_key==427) return(0);
    if (smg_key==639) return(0);
/*
    hdc=GetDC(bw_main1.get_smg_hwnd(p_wptr));
    hfont=(HFONT)GetStockObject(OEM_FIXED_FONT);
    holdfont=(HFONT)SelectObject(hdc,hfont);
*/
    key=smg_key;

    if ( (key<0) || ((key>=32)&&(key<127)) )
    {
        set_smg_modi(ptr,1);

        if ((key>0)&&(smg_chns_char==0))  //save key to str2
        {
          c1=key;
          //str2[0]=c1;
          //str2[1]=0;
/*
          // --- debug ---
          deb_record(str2);
*/
        }
        else
        {
          if (smg_chns_char==0)
          {
            smg_chns_char=1;
            smg_chns_str[0]=key;
            smg_chns_str[1]=0;

            return(0);
          }
          else
          {
            smg_chns_str[1]=key;
            smg_chns_str[2]=0;
/*
            // --- debug ---
            deb_record(smg_chns_str);
*/
          }
        }
      


        // get char , if chiness w=2 if ascii w=1 ,w2=1(half chiness)
        get_s_smg_data(str1,SMG_SIZE);

        for (k=0;k<300;k++) pos[k]=(-1);

        k=0;
        j=0;
        pos[0]=0;

        while(/*(k<l1)&&*/(k<strlen(str1)))
        {
          if (str1[k]<0) k=k+2;
          else k=k+1;

          j++;

          pos[j]=k;
        }

        j=0;
        w=0;
        w2=0;

        for (k=0;k<300;k++)
        {
          if (pos[k]==(smg_p1+n2))
          {
            w2=0;
            break;
          }

          if (pos[k]> (smg_p1+n2))
          {
            w2=1;
            break;
          }

          if (pos[k]<0) break;
        }



        n2=n2+w2;
	scrn_c=scrn_c+w2;

        // insert
        if (smg_chns_char==0)   // save str2 to smg_data
        {
          if (smg_calc_len()+1>datalen_real) smg_erase_last_char(1);

	  for (i=datalen_real-1;i>n2+smg_p1;i--)
	  {
	    set_smg_data(i,get_smg_data(i-1));
	  }
	  set_smg_data(smg_p1+n2,c1);
	  n2++;
	  scrn_c++;
        }
        else
        {
          if (smg_p1+n2+1<=datalen_real)
          {
            if (smg_calc_len()+2>datalen_real) smg_erase_last_char(2);

            smg_chns_char=0;
            
	    for (i=datalen_real-1;i>=n2+smg_p1;i--)
	    {
	      set_smg_data(i+2,get_smg_data(i));
	    }
	    
	    set_smg_data(smg_p1+n2+0,smg_chns_str[0]);
	    set_smg_data(smg_p1+n2+1,smg_chns_str[1]);
	    n2=n2+2;
	    scrn_c=scrn_c+2;
          }
          else
          {
            smg_chns_char=0;
	    n2=n2+2;
	    scrn_c=scrn_c+2;
          }
        }
        
	if (smg_p1+n2+1>datalen_real)  //display string
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          //smg_chns_char=0;
          set_smg_modi(ptr,0);
          
          return(SMG_KEY_RET);
	}
	else if (n2+1>datalen)
	{
          for (k=0;k<3;k++)   //left scroll
          {
	    get_s_smg_data(str1,SMG_SIZE);
	    p2=get_smg_txt_after(str1,smg_p1);
	    p3=p2-smg_p1;
	    smg_p1=p2;
	    smg_p_x=(int)(smg_edit_xchar*(smg_cur1));

	    n2=n2-p3;
	    scrn_c=scrn_c-p3;

	    if (n2+1>datalen) continue;
            else break;
          }

	}
	else
	{
	}

        //posi=n2;
        set_smg_posi(ptr,n2);
        return(0);
    }
    
    if (key==SMG_KEY_LEFT)
    {



      // get prev char , if chiness w=2 if ascii w=1 ,w2=1(half chiness)
      get_s_smg_data(str1,SMG_SIZE);

      for (k=0;k<300;k++) pos[k]=(-1);

      k=0;
      j=0;
      pos[0]=0;

      while(/*(k<l1)&&*/(k<strlen(str1)))
      {
        if (str1[k]<0) k=k+2;
        else k=k+1;

        j++;

        pos[j]=k;
      }

      j=0;
      w=0;
      w2=0;

      for (k=0;k<300;k++)
      {
        if (pos[k]==(smg_p1+n2))
        {
          if (k>0)
          {
            j=pos[k-1];

            if (str1[j]<0) w=2;
            else w=1;
            w2=0;
          }
          else
          {
            j=pos[0];

            w=1;
            w2=0;
          }
          
          break;
        }

        if (pos[k]>(smg_p1+n2))
        {
          if (k>=2)
          {
            j=pos[k-2];

            if (str1[j]<0) w=2;
            else w=1;
            w2=1;
          }
          else 
          {
            if (k==1)
            {
              j=pos[0];

              w=0;
              w2=1;
            }
            else
            {
              j=(smg_p1+n2);

              w=0;
              w2=0;
            }
          }

          break;
        }

        if (pos[k]<0) break;
      }



      if (n2+1>w+w2)
      {
        n2=n2-w-w2;
	scrn_c=scrn_c-w-w2;
      }
      else
      {
	if (smg_p1>0)  //right scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_before(str1,smg_p1);
	  p3=smg_p1-p2;
	  smg_p1=p2;

	  n2=n2+p3;
	  scrn_c=scrn_c+p3;
	}
	else  //at begin of string,normal display string and return with key 
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      
      return(0);
    }
    
    if (key==SMG_KEY_RIGHT)
    {



      // get char , if chiness w=2 if ascii w=1 ,w2=1(half chiness)
      get_s_smg_data(str1,SMG_SIZE);

      for (k=0;k<300;k++) pos[k]=(-1);

      k=0;
      j=0;
      pos[0]=0;

      while(/*(k<l1)&&*/(k<strlen(str1)))
      {
        if (str1[k]<0) k=k+2;
        else k=k+1;

        j++;

        pos[j]=k;
      }

      j=0;
      w=0;
      w2=0;

      for (k=0;k<300;k++)
      {
        if (pos[k]==(smg_p1+n2))
        {
          j=pos[k];

          if (str1[j]<0) w=2;
          else w=1;
          w2=0;

          break;
        }

        if (pos[k]>(smg_p1+n2))
        {
          w=0;
          w2=1;

          break;
        }

        if (pos[k]<0) break;
      }



      if (n2+w+w2+1<datalen)
      {
	if (n2+smg_p1+1<datalen_real)
	{
          n2=n2+w+w2;
	  scrn_c=scrn_c+w+w2;
        }
        else  // at end of string , display string and return with key
        {
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
        }
      }
      else
      {
	if (n2+smg_p1+1<datalen_real)  //left scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_after(str1,smg_p1);
	  p3=p2-smg_p1;
	  smg_p1=p2;

	  n2=n2-p3;
	  scrn_c=scrn_c-p3;
	}
	else  // at end of string , display string and return with key
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      
      return(0);
    }
    
    if (key==SMG_KEY_UP)
    {
      //posi=n2;
      set_smg_posi(ptr,n2);
      
      smg_p1=0;
      set_smg_modi(ptr,0);

      return(key);
    }
    
    if (key==SMG_KEY_DOWN)
    {
      //posi=n2;
      set_smg_posi(ptr,n2);

      smg_p1=0;
      set_smg_modi(ptr,0);

      return(key);
    }
    
    if (key==SMG_KEY_BACKSP)
    {
      set_smg_modi(ptr,1);



      // get prev char , if chiness w=2 if ascii w=1 ,w2=1(half chiness)
      get_s_smg_data(str1,SMG_SIZE);

      for (k=0;k<300;k++) pos[k]=(-1);

      k=0;
      j=0;
      pos[0]=0;

      while(/*(k<l1)&&*/(k<strlen(str1)))
      {
        if (str1[k]<0) k=k+2;
        else k=k+1;

        j++;

        pos[j]=k;
      }

      j=0;
      w=0;
      w2=0;

      for (k=0;k<300;k++)
      {
        if (pos[k]==(smg_p1+n2))
        {
          if (k>0)
          {
            j=pos[k-1];

            if (str1[j]<0) w=2;
            else w=1;
            w2=0;
          }
          else
          {
            j=pos[0];

            w=1;
            w2=0;
          }
          
          break;
        }

        if (pos[k]>(smg_p1+n2))
        {
          if (k>=2)
          {
            j=pos[k-2];

            if (str1[j]<0) w=2;
            else w=1;
            w2=1;
          }
          else 
          {
            if (k==1)
            {
              j=pos[0];

              w=0;
              w2=1;
            }
            else
            {
              j=(smg_p1+n2);

              w=0;
              w2=0;
            }
          }

          break;
        }

        if (pos[k]<0) break;
      }



      if (n2+1>w+w2)
      {
        n2=n2-w2;
        scrn_c=scrn_c-w2;

        if (w==1)
        {
  	  for (i=n2+smg_p1+0;i<datalen_real;i++)
	  {
	    set_smg_data(i-1,get_smg_data(i));
	  }
	  set_smg_data(datalen_real-1,' ');
	  n2--;
	  scrn_c--;
        }
        else 
        {
          if (w==2)
          {
    	    for (i=n2+smg_p1+0;i<datalen_real;i++)
	    {
	      set_smg_data(i-2,get_smg_data(i));
	    }
	    set_smg_data(datalen_real-2,' ');
	    set_smg_data(datalen_real-1,' ');
	    n2=n2-w;
	    scrn_c=scrn_c-w;
          }
        }
      }
      else
      {
	if (smg_p1>0)  // right scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_before(str1,smg_p1);
	  p3=smg_p1-p2;
	  smg_p1=p2;
	  
	  n2=n2+p3;
	  scrn_c=scrn_c+p3;
	}
	else  //at begin of string ,display string and return with key
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (key==SMG_KEY_RET)
    {
      //posi=n2;
      set_smg_posi(ptr,n2);
      
      smg_p1=0;
      set_smg_modi(ptr,0);

      return(key);
    }

    if (key==SMG_KEY_DEL)
    {
      set_smg_modi(ptr,1);



      // get char , if chiness w=2 if ascii w=1 ,w2=1(half chiness)
      get_s_smg_data(str1,SMG_SIZE);

      for (k=0;k<300;k++) pos[k]=(-1);

      k=0;
      j=0;
      pos[0]=0;

      while(/*(k<l1)&&*/(k<strlen(str1)))
      {
        if (str1[k]<0) k=k+2;
        else k=k+1;

        j++;

        pos[j]=k;
      }

      j=0;
      w=0;
      w2=0;

      for (k=0;k<300;k++)
      {
        if (pos[k]==(smg_p1+n2))
        {
          j=pos[k];

          if (str1[j]<0) w=2;
          else w=1;
          w2=0;

          break;
        }

        if (pos[k]>(smg_p1+n2))
        {
          w=0;
          w2=1;

          break;
        }

        if (pos[k]<0) break;
      }



      if (smg_p1+n2+1<datalen_real)
      {
        n2=n2-w2;
        scrn_c=scrn_c-w2;

        if (w==1)
        {
	  for (i=smg_p1+n2+1;i<datalen_real;i++)
	  {
	    set_smg_data(i-1,get_smg_data(i));
	  }
	  set_smg_data(datalen_real-1,' ');
        }
        else
        {
          if (w==2)
          {
	    for (i=smg_p1+n2+2;i<datalen_real;i++)
	    {
	      set_smg_data(i-2,get_smg_data(i));
	    }
	    set_smg_data(datalen_real-2,' ');
	    set_smg_data(datalen_real-1,' ');
          }
        }
      }
      else
      {
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (  ((key>=0)&&(key<32))  ||(key==SMG_KEY_PGUP)||(key==SMG_KEY_PGDOWN)||  ((key>200)&&(key<=410))  )
    {
      //posi=n2;
      set_smg_posi(ptr,n2);
      
      smg_p1=0;
      set_smg_modi(ptr,0);
      
      return(key);
    }
    
    //posi=n2;
    set_smg_posi(ptr,n2);
    return(0);
  }
  
  return(0);
}

int  smg_get_number(int p_wptr,int scrn_l,int scrn_c,char *atten,
		    int datalen,int datalen_real,char link,
		    int ptr,int color,int dec,int posi)
{
  char  c1;
  char  str1[SMG_SIZE];
  //char  str2[SMG_SIZE];
  int   n1,n2;
  int   i;
  int   key;
  int   p2,p3;

/* --- display attention and data --- */
  n1=strlen(atten);

/* --- input command --- */
  if ( posi >= datalen ) posi= datalen-1;
  if ( posi <  0)        posi= 0;

  n2= posi;

  smg_cur1=scrn_c+n1;

  scrn_c=scrn_c+n1+n2;

/* --- pre insert when smg_chns_next==1 --- */
  if (smg_key!=0)
  {
    key=smg_key;

    if (  (key=='+')||(key=='-')||(key=='.')||(key==' ')||  ((key>='0')&&(key<='9'))  )    //(((key>=32)&&(key<=126))/*||(key<0)*/)
    {
      set_smg_modi(ptr,1);

        c1=key;                   //save to str2
        //str2[0]=c1;
        //str2[1]=0;
        
        // insert
        if (smg_chns_char==0)           //save to smg_data
        {
	  for (i=datalen_real-1;i>=n2+smg_p1;i--)
	  {
	    set_smg_data(i+1,get_smg_data(i));
	  }
	  set_smg_data(smg_p1+n2,c1);
	  n2++;
	  scrn_c++;
        }

	if (smg_p1+n2+1>datalen_real)
	{
	  get_smg_num_conv(datalen_real,dec);

          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          //smg_chns_char=0;
          set_smg_modi(ptr,0);
          
          return(SMG_KEY_RET);
	}
	
	if (n2+1>datalen)  //left scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_after(str1,smg_p1);
	  p3=p2-smg_p1;
	  smg_p1=p2;

	  n2=n2-p3;
	  scrn_c=scrn_c-p3;
	}
	else  //normal display string
	{
	}

      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    //get_s_smg_data(str5,SMG_SIZE);
    //sprintf(str4, "in get_num...before left_arrow,data=%s,",str5);
    //deb_record(str4);

    if (key==SMG_KEY_LEFT)
    {
      if (n2+1>1)
      {
        n2--;
	scrn_c--;
      }
      else
      {
	if (smg_p1>0)  //right scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_before(str1,smg_p1);
	  p3=smg_p1-p2;
	  smg_p1=p2;
	  
	  n2=n2+p3;
	  scrn_c=scrn_c+p3;
	}
	else  // at begin of string ,normal display and return with key
	{
	  get_smg_num_conv(datalen_real,dec);

          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (key==SMG_KEY_RIGHT)
    {
      if (n2+1<datalen)
      {
        if (n2+smg_p1+1<datalen_real)
        {
          n2++;
	  scrn_c++;
        }
        else  //at end of string, normal display and return with key
        {
	  get_smg_num_conv(datalen_real,dec);

          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          
          return(key);
        }
      }
      else
      {
	if (n2+smg_p1+1<datalen_real)  //left scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_after(str1,smg_p1);
	  p3=p2-smg_p1;
	  smg_p1=p2;
	  
	  n2=n2-p3;
	  scrn_c=scrn_c-p3;
	}
	else  //at end of string ,normal display string and return with key
	{
	  get_smg_num_conv(datalen_real,dec);

          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    //get_s_smg_data(str5,SMG_SIZE);
    //sprintf(str4, "in get_num...before up_arrow,data=%s,",str5);
    //deb_record(str4);

    if (key==SMG_KEY_UP)
    {
      get_smg_num_conv(datalen_real,dec);

      //posi=n2;
      set_smg_posi(ptr,n2);
      
      smg_p1=0;
      set_smg_modi(ptr,0);
      
      return(key);
    }
    
    if (key==SMG_KEY_DOWN)
    {
      get_smg_num_conv(datalen_real,dec);

      //posi=n2;
      set_smg_posi(ptr,n2);
      
      smg_p1=0;
      set_smg_modi(ptr,0);

      return(key);
    }
    
    if (key==SMG_KEY_BACKSP)
    {
      set_smg_modi(ptr,1);

      if (n2+1>1)
      {
	for (i=n2+smg_p1+0;i<datalen_real;i++)
	{
	  set_smg_data(i-1,get_smg_data(i));
	}
	set_smg_data(datalen_real-1,' ');
	
	n2--;
	scrn_c--;
      }
      else
      {
	if (smg_p1>0)  //right scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_before(str1,smg_p1);
	  p3=smg_p1-p2;
	  smg_p1=p2;

	  n2=n2+p3;
	  scrn_c=scrn_c+p3;
	}
	else  //at begin of string ,normal display and return with key
	{
	  get_smg_num_conv(datalen_real,dec);

          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (key==SMG_KEY_RET)
    {
      get_smg_num_conv(datalen_real,dec);

      //posi=n2;
      set_smg_posi(ptr,n2);

      smg_p1=0;
      set_smg_modi(ptr,0);

      return(key);
    }

    if (key==SMG_KEY_DEL)
    {
      set_smg_modi(ptr,1);

      if (smg_p1+n2+1<=datalen_real)
      {
	for (i=smg_p1+n2+1;i<datalen_real;i++)
	{
	  set_smg_data(i-1,get_smg_data(i));
	}
	set_smg_data(datalen_real-1,' ');
      }
      else
      {
	set_smg_data(smg_p1+n2-1,' ');
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (  ((key>=0)&&(key<32))  ||(key==SMG_KEY_PGUP)||(key==SMG_KEY_PGDOWN)||  ((key>200)&&(key<=410))  )
    {
      get_smg_num_conv(datalen_real,dec);

      //posi=n2;
      set_smg_posi(ptr,n2);

      smg_p1=0;
      set_smg_modi(ptr,0);

      return(key);
    }
    
    //SelectObject(hdc,holdfont);
    //ReleaseDC(bw_main1.get_smg_hwnd(p_wptr),hdc);
    
    //posi=n2;
    set_smg_posi(ptr,n2);
    return(0);
  }
  
  return(0);
}

int  smg_get_password(int p_wptr,int scrn_l,int scrn_c,char *atten,
		    int datalen,int datalen_real,char link,
		    int ptr,int color,int posi)
{
  char  c1;
  char  str1[SMG_SIZE];
  //char  str2[SMG_SIZE];
  int   n1,n2;
  int   i;
  int   key;
  int   p2,p3;

/* --- display attention and data --- */
  n1=strlen(atten);

/* --- input command --- */
  get_s_smg_data(str1,SMG_SIZE);

  if ( posi > datalen ) posi= datalen;
  if ( posi > strlen(str1) ) posi= strlen(str1);
  if ( posi <  0)        posi= 0;
  
  n2= posi;

  smg_cur1=scrn_c+n1;

  scrn_c=scrn_c+n1+n2;

/* --- pre insert when smg_chns_next==1 --- */
  if (smg_key!=0)
  {
    key=smg_key;

    if (  ( key>=' ') && (key<127)  ) 
    {
      set_smg_modi(ptr,1);

      c1=key;                   //save to str2
      //str2[0]=c1;
      //str2[1]=0;
        
      //HideCaret(bw_main1.get_smg_hwnd(p_wptr));
      //TextOut(hdc,smg_p_x,smg_p_y,str2,1);
      //ShowCaret(bw_main1.get_smg_hwnd(p_wptr));

      // insert
      get_s_smg_data(str1,SMG_SIZE);

      //printf("get pass 1 str1=%s,\n",str1);
        
      for (i=strlen(str1);i>=n2+smg_p1;i--)
      {
        set_smg_data(i+1,get_smg_data(i));
      }
	
      set_smg_data(smg_p1+n2,c1);



      //get_s_smg_data(str1,SMG_SIZE);
      //
      //printf("get pass 2 str1=%s,\n",str1);
        

	
      n2++;
      scrn_c++;

      if (smg_p1+n2+1>datalen_real)
      {
        //posi=n2;
        set_smg_posi(ptr,n2);
        
        smg_p1=0;
        //smg_chns_char=0;
        set_smg_modi(ptr,0);
        
        return(SMG_KEY_RET);
      }
	
      if (n2+1>datalen)  //left scroll
      {
	get_s_smg_data(str1,SMG_SIZE);
	p2=get_smg_txt_after(str1,smg_p1);
	p3=p2-smg_p1;
	smg_p1=p2;
	smg_p_x=(int)(smg_edit_xchar*(smg_cur1));
	  
	n2=n2-p3;
	scrn_c=scrn_c-p3;
      }
      else  //normal display string
      {
      }

      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (key==SMG_KEY_LEFT)
    {
      if (n2+1>1)
      {
        n2--;
	scrn_c--;
      }
      else
      {
	if (smg_p1>0)  //right scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_before(str1,smg_p1);
	  p3=smg_p1-p2;
	  smg_p1=p2;
	  
	  n2=n2+p3;
	  scrn_c=scrn_c+p3;
	}
	else  // at begin of string ,normal display and return with key
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (key==SMG_KEY_RIGHT)
    {
      if (n2+1<datalen)
      {
        get_s_smg_data(str1,SMG_SIZE);

        if (  (n2+smg_p1+1<=datalen_real)  &&  (n2+smg_p1+1<=strlen(str1))  )
        {
          n2++;
	  scrn_c++;
        }
        else  //at end of string, normal display and return with key
        {
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          
          return(key);
        }
      }
      else
      {
	if (  (n2+smg_p1+1<=datalen_real)  &&  (n2+smg_p1+1<=strlen(str1))  ) //left scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  p2=get_smg_txt_after(str1,smg_p1);
	  p3=p2-smg_p1;
	  smg_p1=p2;
	  
	  n2=n2-p3;
	  scrn_c=scrn_c-p3;
	}
	else  //at end of string ,normal display string and return with key
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    if (key==SMG_KEY_UP)
    {
      set_smg_modi(ptr,0);

      //posi=n2;
      set_smg_posi(ptr,n2);
      smg_p1=0;
      return(key);
    }
    
    if (key==SMG_KEY_DOWN)
    {
      set_smg_modi(ptr,0);

      //posi=n2;
      set_smg_posi(ptr,n2);
      smg_p1=0;
      return(key);
    }
    
    if (key==SMG_KEY_BACKSP)
    {
      set_smg_modi(ptr,1);

      if (n2+1>1)
      {
        get_s_smg_data(str1,SMG_SIZE);
        
	for (i=n2+smg_p1+0;i<=strlen(str1);i++)
	{
	  set_smg_data(i-1,get_smg_data(i));
	}
	
	set_smg_data(strlen(str1),0);
	
	n2--;
	scrn_c--;
      }
      else
      {
	if (smg_p1>0)  //right scroll
	{
	  get_s_smg_data(str1,SMG_SIZE);
	  
	  p2=get_smg_txt_before(str1,smg_p1);
	  p3=smg_p1-p2;
	  smg_p1=p2;
	  smg_p_x=(int)(smg_edit_xchar*(smg_cur1));

	  n2=n2+p3;
	  scrn_c=scrn_c+p3;
	}
	else  //at begin of string ,normal display and return with key
	{
          //posi=n2;
          set_smg_posi(ptr,n2);
          
          smg_p1=0;
          set_smg_modi(ptr,0);
          
          return(key);
	}
      }
      
      //posi=n2;
      set_smg_posi(ptr,n2);
      
      return(0);
    }
    
    if (key==SMG_KEY_RET)
    {
      set_smg_modi(ptr,0);

      //posi=n2;
      set_smg_posi(ptr,n2);
      smg_p1=0;
      return(key);
    }

    if (key==SMG_KEY_DEL)
    {
      set_smg_modi(ptr,1);

      if (smg_p1+n2+1<=strlen(str1))
      {
	get_s_smg_data(str1,SMG_SIZE);
	  
	for (i=smg_p1+n2+1;i<=strlen(str1);i++)
	{
	  set_smg_data(i-1,get_smg_data(i));
	}
	
	set_smg_data(strlen(str1),0);
      }
      else
      {
      }

      //posi=n2;
      set_smg_posi(ptr,n2);
      return(0);
    }
    
    //posi=n2;
    set_smg_posi(ptr,n2);
    return(0);
  }
  
  return(0);
}

int   get_smg_num_conv(int datalen_real,int dec)
{
  char str1[SMG_SIZE];
  int  s_state1;
  int  i,j,k;

/* convert to number */
  for (i=0;i<SMG_SIZE;i++)  str1[i]=' ';
  str1[61]=0;

  s_state1=0;
  
  if (datalen_real<=0) return(0);
  
  // erase number which after number+' '(after number and space)
  k=0;
  
  for (i=0;i<datalen_real;i++)
  {
      if (get_smg_data(i)>='0') s_state1=1;
      if ((get_smg_data(i)==' ')&&(s_state1==1))
      {
        k=1;
        break;
      }
  }

  if (k==1)
      for (i=i+1;i<datalen_real;i++) set_smg_data(i,' ');
  
  // search '.' position
  k=0;
  
  for (i=0;i<datalen_real;i++) 
  {
    if (get_smg_data(i)=='.')
    {
      k=1;
      break;
    }
  }

  if (k==0)  // not found '.'
  {
    for (i=datalen_real-1;i>=0;i--) if (get_smg_data(i)>' ') break;
    
    i++;
  }
  
  for (j=0;j<i;j++)
  {
    if  ((30-i+j<0)||(30-i+j>=61)) continue;
    
    str1[30-i+j]=get_smg_data(j);  // copy number where before '.'
  }
  
  str1[30]='.';
  
  for (j=i+1;j<datalen_real;j++)
  {
    if ((30+j-i<0)||(30+j-i>=61)) continue;
    
    str1[30+j-i]=get_smg_data(j);  // copy number where after '.'
  }
  
  for (j=0;j<datalen_real;j++)  // copy number
    if (dec==0)
    {
      if ((30-datalen_real+j<0)||(30-datalen_real+j>=61)) continue;
      
      set_smg_data(j,str1[30-datalen_real+j]);
    }
    else
    {
      if ((30-(datalen_real-dec-1)+j<0)||(30-(datalen_real-dec-1)+j>=61)) continue;
      
      set_smg_data(j,str1[30-(datalen_real-dec-1)+j]);
    }

  return(0);
}

int  get_smg_txt_after(char *data,int p1)
{
/*
  if (data[p1]>=0)// &&(data[p1]<=127)) 
  {
    if (p1+1<=l2) return(p1+1);
    else return(p1);
  }
  else
  {
    if ((p1+2<=l2)&&(2<=l1)) return(p1+2);
    else
    {
      if (p1+1>l2) return(p1);
      else return(p1+1);
    }
  }
*/
  int pos[300];
  int i,j;

  for (i=0;i<300;i++) pos[i]=(-1);

  i=0;
  j=0;
  pos[0]=0;

  while(i<strlen(data))
  {
    if (data[i]<0) i=i+2;
    else i=i+1;

    j++;

    pos[j]=i;
  }

  j=p1;

  for (i=0;i<300;i++)
  {
    if (pos[i]>p1)
    {
      j=pos[i];

      break;
    }

    if (pos[i]<0) break;
  }

  return(j);
}

int  get_smg_txt_before(char *data,int p1)
{
/*
  if (p1<2) 
  {
    if (p1>=1) return(p1-1);
    else return(p1);
  }
  else
  {
    if (data[p1-2]>=0)//&&(data[p1-1]<=127))
    {
      return(p1-1);
    }
    else
    {
      return(p1-2);
    }
  }
*/
  int pos[300];
  int i,j;

  for (i=0;i<300;i++) pos[i]=(-1);

  i=0;
  j=0;
  pos[0]=0;

  while(i<strlen(data))
  {
    if (data[i]<0) i=i+2;
    else i=i+1;

    j++;

    pos[j]=i;
  }

  j=0;

  for (i=0;i<300;i++)
  {
    if (pos[i]==p1)
    {
      if (i>0) j=pos[i-1];
      else j=pos[0];

      break;
    }

    if (pos[i]>p1)
    {
      if (i>=2) j=pos[i-2];
      else 
      {
        if (i==1) j=pos[0];
        else j=p1;
      }

      break;
    }

    if (pos[i]<0) break;
  }

  return(j);
}

int  smg_calc_len(void)
{
  int  pos[SMG_SIZE];
  int  pos2[SMG_SIZE];
  char str1[SMG_SIZE];
  int  i,j,k;

  get_s_smg_data(str1,SMG_SIZE);

  for (k=0;k<300;k++)
  {
    pos[k]=(-1);
    pos2[k]=(-1);
  }

  i=0;
  j=0;
  k=0;
  pos[0]=0;
  pos2[0]=0;

  while(/*(k<l1)&&*/(k<strlen(str1)))
  {
    pos[j] =k;

    if (str1[k]<0)
    {
      k=k+2;
      i=2;
    }
    else
    {
      k=k+1;
      i=1;
    }

    pos2[j]=i;
    j++;
  }

  if (j>0)
  {
    j--;

    while(j>0)
    {
      if ((pos2[j]==1)&&(get_smg_data(pos[j])>=0)&&(get_smg_data(pos[j])<=' ')) j--;
      else break;
    }

    return(pos[j]+pos2[j]);
  }

  return(0);
}

int  smg_erase_last_char(int p1)
{
  int  pos[SMG_SIZE];
  int  pos2[SMG_SIZE];
  char str1[SMG_SIZE];
  int  i,j,k;

  get_s_smg_data(str1,SMG_SIZE);

  for (k=0;k<300;k++)
  {
    pos[k]=(-1);
    pos2[k]=(-1);
  }

  i=0;
  j=0;
  k=0;
  pos[0]=0;
  pos2[0]=0;

  while(/*(k<l1)&&*/(k<strlen(str1)))
  {
    pos[j] =k;

    if (str1[k]<0)
    {
      k=k+2;
      i=2;
    }
    else
    {
      k=k+1;
      i=1;
    }

    pos2[j]=i;
    j++;
  }

  if (j>0)
  {
    j--;

    if (p1==1)
    {
      if (pos2[j]==1) 
      {
        set_smg_data(pos[j],' ');
      }
      else
      {
        set_smg_data(pos[j]+0,' ');
        set_smg_data(pos[j]+1,' ');
      }
    }
    else // p1==2
    {
      if (pos2[j]==2)
      {
        set_smg_data(pos[j]+0,' ');
        set_smg_data(pos[j]+1,' ');
      }
      else // pos2[j]==1
      {
        if (pos2[j-1]==1)
        {
          set_smg_data(pos[j-0],' ');
          set_smg_data(pos[j-1],' ');
        }
        else // pos2[j-1]==2
        {
          set_smg_data(pos[j-1]+0,' ');
          set_smg_data(pos[j-1]+1,' ');

          set_smg_data(pos[j],' ');
        }
      }
    }
  }

  return(0);
}

int smg_chs_string_cut(char *p_str,int pn) // pn :field length
{
  int p1,p2;

  if (pn<=0) return(0);

  for (p1=0;p1<strlen(p_str);p1++) 
    if ((p_str[p1]>=0)&&(p_str[p1]<' ')) 
      p_str[p1]=' ';

  for (p1=strlen(p_str);p1<pn;p1++)
  {
    p_str[p1+0]=' ';
    p_str[p1+1]=0;
  }

  p1=0;
  p2=0;

  while(p2<=pn)
  {
    p1=p2;
    if (p_str[p2]==0) return(p1);
    else if (p_str[p2]<0)
    {
      if (p_str[p2+1]==0) return(p1);
      else p2=p2+2;
    }
    else p2=p2+1;
  }

  return(p1);
}

int  SetCaretPos(int x,int y)
{
  return(0);
}



int  get_smg_line(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_line[gptr]);
}
int  get_smg_colu(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_colu[gptr]);
}
int  get_smg_data(int ptr1)
{
  if ((ptr1<0)||(ptr1>=SMG_SIZE)) return(0);
  return(smg_data[ptr1]);
}
int  get_s_smg_data(char *p_s1,int p_s1_size)
{
  int i,j;

  if (p_s1_size>SMG_SIZE) i=SMG_SIZE;
  else i=p_s1_size;

  p_s1[0]=0;

  for (j=0;j<i-1;j++)
  {
    p_s1[j+0]=smg_data[j];
    p_s1[j+1]=0;
  }

  return(0);
}
int  get_smg_string(int gptr,char *p_s1,int p_s1_size)
{
  int i,j;

  p_s1[0]=0;

  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);

  if (p_s1_size>SMG_SIZE) i=SMG_SIZE;
  else i=p_s1_size;

  for (j=0;j<i-1;j++)
  {
    p_s1[j+0]=smg_string[gptr][j];
    p_s1[j+1]=0;
  }

  return(0);
}
int  get_c_smg_string(int gptr,int ptr1)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  if ((ptr1<0)||(ptr1>=SMG_SIZE)) return(0);
  return(smg_string[gptr][ptr1]);
}
int  get_smg_type(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_type[gptr]);
}
int  get_smg_dlen(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_dlen[gptr]);
}
int  get_smg_dlen_real(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_dlen_real[gptr]);
}
int  get_smg_ddec(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_ddec[gptr]);
}
int  get_smg_link(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_link[gptr]);
}
int  get_smg_color(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_color[gptr]);
}
int  get_smg_posi(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_posi[gptr]);
}
int  get_smg_atte(int gptr,char *p_s1,int p_s1_size)
{
  int i,j;

  p_s1[0]=0;

  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);

  if (p_s1_size>SMG_SIZE) i=SMG_SIZE;
  else i=p_s1_size;

  for (j=0;j<i-1;j++)
  {
    p_s1[j+0]=smg_atte[gptr][j];
    p_s1[j+1]=0;
  }

  return(0);
}
int  get_smg_read_id(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_read_id[gptr]);
}

int  set_smg_line(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_line[gptr]=val;
  return(0);
}
int  set_smg_colu(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_colu[gptr]=val;
  return(0);
}
int  set_smg_data(int ptr1,int val)
{
  if ((ptr1<0)||(ptr1>=SMG_SIZE)) return(0);
  smg_data[ptr1]=val;
  return(0);
}
int  set_smg_type(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_type[gptr]=val;
  return(0);
}
int  set_smg_dlen(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_dlen[gptr]=val;
  return(0);
}
int  set_smg_dlen_real(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_dlen_real[gptr]=val;
  return(0);
}
int  set_smg_ddec(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_ddec[gptr]=val;
  return(0);
}
int  set_smg_link(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_link[gptr]=val;
  return(0);
}
int  set_smg_color(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_color[gptr]=val;
  return(0);
}
int  set_smg_posi(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_posi[gptr]=val;
  return(0);
}
int  set_smg_read_id(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_read_id[gptr]=val;
  return(0);
}
int  set_smg_string(int gptr,char *p_s1,int p_s1_size)
{
  int i,j;

  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);

  if (p_s1_size>SMG_SIZE) i=SMG_SIZE;
  else i=p_s1_size;

  smg_string[gptr][0]=0;

  for (j=0;j<i-1;j++)
  {
    smg_string[gptr][j+0]=p_s1[j];
    smg_string[gptr][j+1]=0;
  }

  return(0);
}
int  set_c_smg_string(int gptr,int ptr1,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  if ((ptr1<0)||(ptr1>=SMG_SIZE)) return(0);
  smg_string[gptr][ptr1]=val;
  return(0);
}
int  set_smg_atte(int gptr,const char *p_s1,int p_s1_size)
{
  int i,j;

  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);

  if (p_s1_size>SMG_SIZE) i=SMG_SIZE;
  else i=p_s1_size;

  smg_atte[gptr][0]=0;

  for (j=0;j<i-1;j++)
  {
    smg_atte[gptr][j+0]=p_s1[j];
    smg_atte[gptr][j+1]=0;
  }

  return(0);
}

int  set_smg_modi(int gptr,int val)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  smg_modi[gptr]=val;
  return(0);
}

int  get_smg_modi(int gptr)
{
  if ((gptr<0)||(gptr>=SMG_GET_NUM)) return(0);
  return(smg_modi[gptr]);
}



int g_create_checkbox(int x,int y,int w,int h,int black_box,int color1,int color2,int color3)
{
  int i,k;
  int find,find_ptr;
  
  find=0;
  find_ptr=(-1);
  
  for (k=0;k<g_checkbox_ptr;k++)
  {
    if (g_checkbox_delete[k]==1)
    {
      find=1;
      find_ptr=k;
      break;
    }
  }
  
  if (find==0)
  {
    if ((g_checkbox_ptr<0)||(g_checkbox_ptr>=G_CHECKBOX_NUM)) return(-1);
  
    find_ptr=g_checkbox_ptr;
  }
  
  g_checkbox_posi[find_ptr][0]=x;
  g_checkbox_posi[find_ptr][1]=y;
  g_checkbox_posi[find_ptr][2]=w;
  g_checkbox_posi[find_ptr][3]=h;
  
  g_checkbox_color[find_ptr][0]=color1;
  g_checkbox_color[find_ptr][1]=color2;
  g_checkbox_color[find_ptr][2]=color3;

  g_checkbox_black[find_ptr]=black_box;
  g_checkbox_enable[find_ptr]=1;

  i=find_ptr;
  if (find==0) g_checkbox_ptr++;
  g_checkbox_delete[i]=0;
  
  return(i);
}

int g_move_checkbox(int box_ptr,int x,int y,int w,int h)
{
  if ((box_ptr<0)||(box_ptr>=G_CHECKBOX_NUM)||(box_ptr>=g_checkbox_ptr)) return(1);
  
  g_checkbox_posi[box_ptr][0]=x;
  g_checkbox_posi[box_ptr][1]=y;
  g_checkbox_posi[box_ptr][2]=w;
  g_checkbox_posi[box_ptr][3]=h;
  
  return(0);
}

int g_enable_checkbox(int ptr,int enable)
{
  if ((ptr<0)||(ptr>=G_CHECKBOX_NUM)||(ptr>=g_checkbox_ptr)) return(1);
  
  g_checkbox_enable[ptr]=enable;
  
  return(0);
}

int g_delete_checkbox(int ptr)
{
  if ((ptr<0)||(ptr>=G_CHECKBOX_NUM)||(ptr>=g_checkbox_ptr)) return(1);
  
  g_checkbox_delete[ptr]=1;
  
  return(0);
}

int g_set_checkbox_value(int ptr,int v)
{
  if ((ptr<0)||(ptr>=G_CHECKBOX_NUM)||(ptr>=g_checkbox_ptr)) return(0);
  
  g_checkbox_value[ptr]=v;
  
  return(0);
}

int g_get_checkbox_value(int ptr)
{
  if ((ptr<0)||(ptr>=G_CHECKBOX_NUM)||(ptr>=g_checkbox_ptr)) return(0);
  
  return(g_checkbox_value[ptr]);
}

int g_paint_checkbox(int vid,int mousedown)
{
  int			i,j,k,l,m,n,o;
  int  			i1,i2,i3,i4;
  int                   k1,k2,k3,k4;
  int                   x1,y1,x2,y2;

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);

  for (i=0;i<g_checkbox_ptr;i++)
  {
    if (g_checkbox_delete[i]==1) continue;
    
    j=g_checkbox_posi[i][0];
    k=g_checkbox_posi[i][1];
    l=g_checkbox_posi[i][2];
    m=g_checkbox_posi[i][3];
    
    i1=g_checkbox_color[i][0];
    i2=g_checkbox_color[i][1];
    i3=g_checkbox_color[i][2];
    i4=255;
    
    k1=255-i1;
    k2=255-i2;
    k3=255-i3;
    k4=i4;
    
    for (n=0;n<l;n++)
    {
      for (o=0;o<m;o++)
      {
        if (g_checkbox_enable[i]==1)
        {
          if (g_checkbox_black[i]==0)
          {
            if ((mousedown==1)&&(g_detect_ptr1==5)&&(g_detect_ptr2==i))
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, i1*7/10, i2*7/10, i3*7/10, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,k+o,i1*7/10,i2*7/10,i3*7/10);
              }
            }
            else
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, i1, i2, i3, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,k+o,i1,i2,i3);
              }
            }
          }
          else
          {
            if ((mousedown==1)&&(g_detect_ptr1==5)&&(g_detect_ptr2==i))
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, 255-(255-i1)*7/10, 255-(255-i2)*7/10, 255-(255-i3)*7/10, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,k+o,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10);
              }
            }
            else
            {
              if (vid)
              {
                  SDL_SetRenderDrawColor(renderer, i1, i2, i3, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
              }
              else
              {
                  deb_set_dot(j+n,k+o,i1,i2,i3);
              }
            }
          }
        }
        else
        {
          if (g_checkbox_black[i]==0)
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, i1*7/10, i2*7/10, i3*7/10, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,i1*7/10,i2*7/10,i3*7/10);
            }
          }
          else
          {
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, 255-(255-i1)*7/10, 255-(255-i2)*7/10, 255-(255-i3)*7/10, i4);
                  fill_rectangle(j+n, k+o, 1, 1);
            }
            else
            {
                  deb_set_dot(j+n,k+o,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10);
            }
          }
        }
      }
    }

    for (n=1;n<l-1;n++)
    {
      if ((mousedown==1)&&(g_detect_ptr1==5)&&(g_detect_ptr2==i))
      {
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, k1*7/10, k2*7/10, k3*7/10, k4);
                  fill_rectangle(j+n,    k+1, 1, 1);
                  fill_rectangle(j+n,k+m-1-1, 1, 1);
        }
        else
        {
                  deb_set_dot(j+n,k+1    ,k1*7/10,k2*7/10,k3*7/10);
                  deb_set_dot(j+n,k+m-1-1,k1*7/10,k2*7/10,k3*7/10);
        }
      }
      else
      {
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, k1, k2, k3, k4);
                  fill_rectangle(j+n,    k+1, 1, 1);
                  fill_rectangle(j+n,k+m-1-1, 1, 1);
        }
        else
        {
                  deb_set_dot(j+n,    k+1,k1,k2,k3);
                  deb_set_dot(j+n,k+m-1-1,k1,k2,k3);
        }
      }
    }
    
    for (o=1;o<m-1;o++)
    {
      if ((mousedown==1)&&(g_detect_ptr1==5)&&(g_detect_ptr2==i))
      {
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, k1*7/10, k2*7/10, k3*7/10, k4);
                  fill_rectangle(j+1,     k+o, 1, 1);
                  fill_rectangle(j+l-1-1, k+o, 1, 1);
        }
        else
        {
                  deb_set_dot(j+1,    k+o,k1*7/10,k2*7/10,k3*7/10);
                  deb_set_dot(j+l-1-1,k+o,k1*7/10,k2*7/10,k3*7/10);
        }
      }
      else
      {
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, k1, k2, k3, k4);
                  fill_rectangle(j+1,     k+o, 1, 1);
                  fill_rectangle(j+l-1-1, k+o, 1, 1);
        }
        else
        {
                  deb_set_dot(j+1,    k+o,k1,k2,k3);
                  deb_set_dot(j+l-1-1,k+o,k1,k2,k3);
        }
      }
    }
    
    if (g_checkbox_value[i]==1)
    {
      x1=j+1+1;
      y1=k+m*0.5;
    
      x2=j+l*0.5;
      y2=k+m-1-2;
    
      if (g_checkbox_enable[i]==1)
      {
        if ((mousedown==1)&&(g_detect_ptr1==5)&&(g_detect_ptr2==i))
        {
          g_draw_line(x1,y1,x2,y2,vid,k1*7/10,k2*7/10,k3*7/10);
        }
        else
        {
          g_draw_line(x1,y1,x2,y2,vid,k1,k2,k3);
        }
      }
      else
      {
        if (g_checkbox_black[i]==0)
        {
          g_draw_line(x1,y1,x2,y2,vid,k1*7/10,k2*7/10,k3*7/10);
        }
        else
        {
          g_draw_line(x1,y1,x2,y2,vid,255-(255-k1)*7/10,255-(255-k2)*7/10,255-(255-k3)*7/10);
        }
      }
    
      x1=x2;
      y1=y2;
    
      x2=j+l-1-1;
      y2=k+m*0.2;

      if (g_checkbox_enable[i]==1)
      {
        if ((mousedown==1)&&(g_detect_ptr1==5)&&(g_detect_ptr2==i))
        {
          g_draw_line(x1,y1,x2,y2,vid,k1*7/10,k2*7/10,k3*7/10);
        }
        else
        {
          g_draw_line(x1,y1,x2,y2,vid,k1,k2,k3);
        }
      }
      else
      {
        if (g_checkbox_black[i]==0)
        {
          g_draw_line(x1,y1,x2,y2,vid,k1*7/10,k2*7/10,k3*7/10);
        }
        else
        {
          g_draw_line(x1,y1,x2,y2,vid,255-(255-k1)*7/10,255-(255-k2)*7/10,255-(255-k3)*7/10);
        }
      }
    }
  }

  return(0);
}

int g_draw_line(int x1,int y1,int x2,int y2,int vid,int color1,int color2,int color3)
{
  // screen top and left is x=0;y=0;
  int x3,y3,x4,y4,x5,y5;
  int i,j;

  if ((x1<0)||(x1>=7680)) return(1);
  if ((x2<0)||(x2>=7680)) return(1);

  if ((y1<0)||(y1>=4320)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);

  if (x2<x1)
  {
    x3=x2;
    y3=y2;

    x2=x1;
    y2=y1;

    x1=x3;
    y1=y3;
  }

  if ((x1<0)||(x1>=7680)) return(0);
  if ((y1<0)||(y1>=4320)) return(0);
  //deb_sr_d_buff[x1][y1]=1;
  if (vid)
  {
          SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
          fill_rectangle(x1, y1, 1, 1);
  }
  else
  {
          deb_set_dot(x1,y1,color1,color2,color3);
  }

  if ((x2<0)||(x2>=7680)) return(0);
  if ((y2<0)||(y2>=4320)) return(0);
  //deb_sr_d_buff[x2][y2]=1;
  if (vid)
  {
          SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
          fill_rectangle(x2, y2, 1, 1);
  }
  else
  {
          deb_set_dot(x2,y2,color1,color2,color3);
  }

  if (x1==x2)
  {
    if (y1<y2)
    {
      for (i=y1+1;i<=y2-1;i++)// deb_sr_d_buff[x1][i]=1;
      {
	if ((x1<0)||(x1>=7680)) return(0);
	if ((i<0)||(i>=4320)) return(0);
	//deb_sr_d_buff[x1][i]=1;
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(x1, i, 1, 1);
        }
        else
        {
                  deb_set_dot(x1,i,color1,color2,color3);
        }
      }
    }
    else if (y2<y1)
    {
      for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--) //deb_sr_d_buff[x1][i]=1;
      {
	if ((x1<0)||(x1>=7680)) return(0);
	if ((i<0)||(i>=4320)) return(0);
	//deb_sr_d_buff[x1][i]=1;
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(x1, i, 1, 1);
        }
        else
        {
                  deb_set_dot(x1,i,color1,color2,color3);
        }
      }
    }
  }
  else
  {
    if (y1==y2)
    {
      for (i=x1+1;i<=x2-1;i++)// deb_sr_d_buff[i][y1]=1;
      {
	if ((i<0)||(i>=7680)) return(0);
	if ((y1<0)||(y1>=4320)) return(0);
	//deb_sr_d_buff[i][y1]=1;
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(i, y1, 1, 1);
        }
        else
        {
                  deb_set_dot(i,y1,color1,color2,color3);
        }
      }
    }
    else if (y2>y1)
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1+(y2-y1)*i/(x2-x1);

	if ((x4<0)||(x4>=7680)) return(0);
	if ((y4<0)||(y4>=4320)) return(0);
        //deb_sr_d_buff[x4][y4]=1;
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(x4, y4, 1, 1);
        }
        else
        {
                  deb_set_dot(x4,y4,color1,color2,color3);
        }

        if (y4>y5+1)
        {
          for (j=y5+1;j<y4;j++)// deb_sr_d_buff[x5][j]=1;
	  {
	    if ((x5<0)||(x5>=7680)) return(0);
	    if ((j<0)||(j>=4320)) return(0);
	    //deb_sr_d_buff[x5][j]=1;
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(x5, j, 1, 1);
            }
            else
            {
                  deb_set_dot(x5,j,color1,color2,color3);
            }
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
    else
    {
      x5=x1;
      y5=y1;

      for (i=1;i<=x2-x1;i++)
      {
        x4=x1+i;
        y4=y1-(y1-y2)*i/(x2-x1);

	if ((x4<0)||(x4>=7680)) return(0);
	if ((y4<0)||(y4>=4320)) return(0);
        //deb_sr_d_buff[x4][y4]=1;
        if (vid)
        {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(x4, y4, 1, 1);
        }
        else
        {
                  deb_set_dot(x4,y4,color1,color2,color3);
        }

        if (y4<y5-1)
        {
          for (j=y5-1;j>y4;j--) //deb_sr_d_buff[x5][j]=1;
	  {
	    if ((x5<0)||(x5>=7680)) return(0);
	    if ((j<0)||(j>=4320)) return(0);
	    //deb_sr_d_buff[x5][j]=1;
            if (vid)
            {
                  SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
                  fill_rectangle(x5, j, 1, 1);
            }
            else
            {
                  deb_set_dot(x5,j,color1,color2,color3);
            }
	  }
        }

        x5=x4;
	y5=y4;
      }
    }
  }

  return(0);
}

int g_create_radiobutton(int x,int y,int w,int h,int black_button,int color1,int color2,int color3,int grp)
{
  int i,k;
  int find,find_ptr;
  
  find=0;
  find_ptr=(-1);
  
  for (k=0;k<g_radiobutton_ptr;k++)
  {
    if (g_radiobutton_delete[k]==1)
    {
      find=1;
      find_ptr=k;
      break;
    }
  }
  
  if (find==0)
  {
    if ((g_radiobutton_ptr<0)||(g_radiobutton_ptr>=G_RADIOBUTTON_NUM)) return(-1);
  
    find_ptr=g_radiobutton_ptr;
  }
  
  g_radiobutton_posi[find_ptr][0]=x;
  g_radiobutton_posi[find_ptr][1]=y;
  g_radiobutton_posi[find_ptr][2]=w;
  g_radiobutton_posi[find_ptr][3]=h;
  
  g_radiobutton_color[find_ptr][0]=color1;
  g_radiobutton_color[find_ptr][1]=color2;
  g_radiobutton_color[find_ptr][2]=color3;

  g_radiobutton_black[find_ptr]=black_button;
  g_radiobutton_enable[find_ptr]=1;

  g_radiobutton_value_group[find_ptr]=grp;

  i=find_ptr;
  if (find==0) g_radiobutton_ptr++;
  g_radiobutton_delete[i]=0;
  
  return(i);
}

int g_move_radiobutton(int radio_ptr,int x,int y,int w,int h)
{
  if ((radio_ptr<0)||(radio_ptr>=G_RADIOBUTTON_NUM)||(radio_ptr>=g_radiobutton_ptr)) return(1);
  
  g_radiobutton_posi[radio_ptr][0]=x;
  g_radiobutton_posi[radio_ptr][1]=y;
  g_radiobutton_posi[radio_ptr][2]=w;
  g_radiobutton_posi[radio_ptr][3]=h;
  
  return(0);
}

int g_enable_radiobutton(int ptr,int enable)
{
  if ((ptr<0)||(ptr>=G_RADIOBUTTON_NUM)||(ptr>=g_radiobutton_ptr)) return(1);
  
  g_radiobutton_enable[ptr]=enable;
  
  return(0);
}

int g_delete_radiobutton(int ptr)
{
  if ((ptr<0)||(ptr>=G_RADIOBUTTON_NUM)||(ptr>=g_radiobutton_ptr)) return(1);
  
  g_radiobutton_delete[ptr]=1;
  
  return(0);
}

int g_set_radiobutton_value(int ptr,int v)
{
  if ((ptr<0)||(ptr>=G_RADIOBUTTON_NUM)||(ptr>=g_radiobutton_ptr)) return(0);
  
  g_radiobutton_value[ptr]=v;
  
  return(0);
}

int g_get_radiobutton_value(int ptr)
{
  if ((ptr<0)||(ptr>=G_RADIOBUTTON_NUM)||(ptr>=g_radiobutton_ptr)) return(0);
  
  return(g_radiobutton_value[ptr]);
}

int g_set_radiobutton_value_group(int ptr,int v)
{
  if ((ptr<0)||(ptr>=G_RADIOBUTTON_NUM)||(ptr>=g_radiobutton_ptr)) return(0);
  
  g_radiobutton_value_group[ptr]=v;
  
  return(0);
}

int g_get_radiobutton_value_group(int ptr)
{
  if ((ptr<0)||(ptr>=G_RADIOBUTTON_NUM)||(ptr>=g_radiobutton_ptr)) return(0);
  
  return(g_radiobutton_value_group[ptr]);
}

int g_paint_radiobutton(int vid,int mousedown)
{
  int			i,j,k,l,m;
  int  			i1,i2,i3;
  int                   k1,k2,k3;

  if ((deb_tx_locked!=1)&&(vid!=1)) return(0);

  for (i=0;i<g_radiobutton_ptr;i++)
  {
    if (g_radiobutton_delete[i]==1) continue;
    
    j=g_radiobutton_posi[i][0];
    k=g_radiobutton_posi[i][1];
    l=g_radiobutton_posi[i][2];
    m=g_radiobutton_posi[i][3];
    
    i1=g_radiobutton_color[i][0];
    i2=g_radiobutton_color[i][1];
    i3=g_radiobutton_color[i][2];
    
    k1=255-i1;
    k2=255-i2;
    k3=255-i3;

    if (g_radiobutton_enable[i]==1)
    {
      if (g_radiobutton_black[i]==0)
      {
        if ((mousedown==1)&&(g_detect_ptr1==6)&&(g_detect_ptr2==i))
        {
          g_draw_circle(j+l/2,k+m/2,l/2,vid,i1*7/10,i2*7/10,i3*7/10,1);    
    
          g_draw_circle(j+l/2,k+m/2,l/2-1,vid,k1*7/10,k2*7/10,k3*7/10,0);    
    
          if (g_radiobutton_value[i]==1)
          {
            g_draw_circle(j+l/2,k+m/2,(l/2)*0.7-2,vid,k1*7/10,k2*7/10,k3*7/10,1);    
          }
        }
        else
        {
          g_draw_circle(j+l/2,k+m/2,l/2,vid,i1,i2,i3,1);    
    
          g_draw_circle(j+l/2,k+m/2,l/2-1,vid,k1,k2,k3,0);    
    
          if (g_radiobutton_value[i]==1)
          {
            g_draw_circle(j+l/2,k+m/2,(l/2)*0.7-2,vid,k1,k2,k3,1);    
          }
        }
      }
      else
      {
        if ((mousedown==1)&&(g_detect_ptr1==6)&&(g_detect_ptr2==i))
        {
          g_draw_circle(j+l/2,k+m/2,l/2,vid,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10,1);    
    
          g_draw_circle(j+l/2,k+m/2,l/2-1,vid,255-(255-k1)*7/10,255-(255-k2)*7/10,255-(255-k3)*7/10,0);    
    
          if (g_radiobutton_value[i]==1)
          {
            g_draw_circle(j+l/2,k+m/2,(l/2)*0.7-2,vid,255-(255-k1)*7/10,255-(255-k2)*7/10,255-(255-k3)*7/10,1);    
          }
        }
        else
        {
          g_draw_circle(j+l/2,k+m/2,l/2,vid,i1,i2,i3,1);    
    
          g_draw_circle(j+l/2,k+m/2,l/2-1,vid,k1,k2,k3,0);    
    
          if (g_radiobutton_value[i]==1)
          {
            g_draw_circle(j+l/2,k+m/2,(l/2)*0.7-2,vid,k1,k2,k3,1);    
          }
        }
      }
    }
    else
    {
      if (g_radiobutton_black[i]==0)
      {
        g_draw_circle(j+l/2,k+m/2,l/2,vid,i1*7/10,i2*7/10,i3*7/10,1);    
    
        g_draw_circle(j+l/2,k+m/2,l/2-1,vid,k1*7/10,k2*7/10,k3*7/10,0);    
    
        if (g_radiobutton_value[i]==1)
        {
          g_draw_circle(j+l/2,k+m/2,(l/2)*0.7-2,vid,k1*7/10,k2*7/10,k3*7/10,1);    
        }
      }
      else
      {
        g_draw_circle(j+l/2,k+m/2,l/2,vid,255-(255-i1)*7/10,255-(255-i2)*7/10,255-(255-i3)*7/10,1);    
    
        g_draw_circle(j+l/2,k+m/2,l/2-1,vid,255-(255-k1)*7/10,255-(255-k2)*7/10,255-(255-k3)*7/10,0);    
    
        if (g_radiobutton_value[i]==1)
        {
          g_draw_circle(j+l/2,k+m/2,(l/2)*0.7-2,vid,255-(255-k1)*7/10,255-(255-k2)*7/10,255-(255-k3)*7/10,1);    
        }
      }
    }
  }

  return(0);
}

int g_data[4][10000][2];
int g_data_ptr[4];

int g_draw_circle(int x,int y,int r,int vid,int color1,int color2,int color3,int st)
{
  int   x0,y0,x1,y1,x2,y2,x3,y3;
  float f1,f2,f3,f4;
  int   i,j,k;
  
  g_data_ptr[0]=0;
  g_data_ptr[1]=0;
  g_data_ptr[2]=0;
  g_data_ptr[3]=0;

  x0=0;         // init position
  y0=r*(-1);
  
  g_data[0][g_data_ptr[0]][0]=x0;
  g_data[0][g_data_ptr[0]][1]=y0;

  g_data_ptr[0]++;
  
  while (1)  //part 1
  {
    x1=x0-1;           // 2 posible next dot-1
    y1=y0;
    
    i=y1*y1+x1*x1;
    f1=sqrt(i);
    
    x2=x0-1;           // 2 posible next dot-2
    y2=y0+1;
    
    i=y2*y2+x2*x2;
    f2=sqrt(i);
    
    f3=f1-r;
    if (f3<0) f3=f3*(-1);
    
    f4=f2-r;
    if (f4<0) f4=f4*(-1);
    
    if (f3<f4)     // who is smaller
    {
      x0=x1;
      y0=y1;
    }
    else
    {
      x0=x2;
      y0=y2;
    }

    g_data[0][g_data_ptr[0]][0]=x0;
    g_data[0][g_data_ptr[0]][1]=y0;

    g_data_ptr[0]++;
    if (g_data_ptr[0]>=10000) return(0);
  
    if (x0<0) x3=x0*(-1);
    else x3=x0;
    
    if (y0<0) y3=y0*(-1);
    else y3=y0;
    
    if (x3>=y3) break;
  }  

  while (1)    //part 2
  {
    x1=x0-1;           // 2 posible next dot-1
    y1=y0+1;
    
    i=y1*y1+x1*x1;
    f1=sqrt(i);
    
    x2=x0;             // 2 posible next dot-2
    y2=y0+1;
    
    i=y2*y2+x2*x2;
    f2=sqrt(i);
    
    f3=f1-r;
    if (f3<0) f3=f3*(-1);
    
    f4=f2-r;
    if (f4<0) f4=f4*(-1);
    
    if (f3<f4)     // who is smaller
    {
      x0=x1;
      y0=y1;
    }
    else
    {
      x0=x2;
      y0=y2;
    }

    g_data[0][g_data_ptr[0]][0]=x0;
    g_data[0][g_data_ptr[0]][1]=y0;

    g_data_ptr[0]++;
    if (g_data_ptr[0]>=10000) return(0);
  
    if (y0>=0) break;
  }  
  
  for (i=0;i<g_data_ptr[0];i++)   //copy to other seg
  {
    g_data[1][i][0]=g_data[0][i][0];
    g_data[1][i][1]=g_data[0][i][1]*(-1);
    
    g_data[2][i][0]=g_data[0][i][0]*(-1);
    g_data[2][i][1]=g_data[0][i][1]*(-1);
    
    g_data[3][i][0]=g_data[0][i][0]*(-1);
    g_data[3][i][1]=g_data[0][i][1];
  }

  g_data_ptr[1]=g_data_ptr[0];
  g_data_ptr[2]=g_data_ptr[0];
  g_data_ptr[3]=g_data_ptr[0];

  for (j=0;j<4;j++)  //move circle
  {
    for (i=0;i<g_data_ptr[j];i++)
    {
      g_data[j][i][0]=g_data[j][i][0]+x;
      g_data[j][i][1]=g_data[j][i][1]+y;
    }
  }
  
  if (st==0)   // hollow circle
  {
    for (j=0;j<4;j++)
    {
      for (i=0;i<g_data_ptr[j];i++)
      {
        x0=g_data[j][i][0];
        y0=g_data[j][i][1];
        
	if ((x0<0)||(x0>=7680)) return(0);
	if ((y0<0)||(y0>=4320)) return(0);
        
        if (vid)
        {
              SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
              fill_rectangle( x0, y0, 1, 1);
        }
        else
        {
              deb_set_dot(x0,y0,color1,color2,color3);
        }
      }
    }
  }
  else  // solid circle
  {
    for (i=0;i<g_data_ptr[0];i++)
    {
      x0=g_data[0][i][0];
      y0=g_data[0][i][1];
        
      if ((x0<0)||(x0>=7680)) return(0);
      if ((y0<0)||(y0>=4320)) return(0);
        
      x1=g_data[1][i][0];
      y1=g_data[1][i][1];
        
      if ((x1<0)||(x1>=7680)) return(0);
      if ((y1<0)||(y1>=4320)) return(0);

      for (k=y0;k<=y1;k++)
      {
        if (vid)
        {
            SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
            fill_rectangle( x0, k, 1, 1);
        }
        else
        {
            deb_set_dot(x0,k,color1,color2,color3);
        }
      }
    }
  
    for (i=0;i<g_data_ptr[2];i++)
    {
      x0=g_data[2][i][0];
      y0=g_data[2][i][1];
        
      if ((x0<0)||(x0>=7680)) return(0);
      if ((y0<0)||(y0>=4320)) return(0);

      x1=g_data[3][i][0];
      y1=g_data[3][i][1];
        
      if ((x1<0)||(x1>=7680)) return(0);
      if ((y1<0)||(y1>=4320)) return(0);

      for (k=y1;k<=y0;k++)
      {
        if (vid)
        {
            SDL_SetRenderDrawColor(renderer, color1, color2, color3, 255);
            fill_rectangle( x0, k, 1, 1);
        }
        else
        {
            deb_set_dot(x0,k,color1,color2,color3);
        }
      }
    }
  }
  
  return(0);
}


// ---------------- lib automatic return -------------------------------
//     in today, file name usually very long, and screen is small,
//     lib automatic return separate one line file name to mutiple lines
//     in the best way.
// ---------------------------------------------------------------------
// ar : automatic return

int ar_conv(char *p_in_str,int p_in_str_size,int p_len)
{
  int  i,j,k,l,m,n;
  char c1,c2;
  int  val1,val2,val3;
  int  len3;
  
  if (p_len<2) return(1);
  
  ar_buff2_ptr=0;
  ar_buff4_ptr=0;
  
  cpy_string(ar_buff3,1000,p_in_str,p_in_str_size);    
  
  while (1)
  {
    ar_buff1_ptr=0;
  
    i=u_strlen(ar_buff3,1000);  // if string's len < p_len, don't need calculate
  
    if (i<=p_len)
    {
      j=strlen(ar_buff3);
      ar_buff2_len[ar_buff2_ptr]=j;
      ar_buff2_ptr++;
      if (ar_buff2_ptr>=1000) ar_buff2_ptr=1000-1;
      break;
    }
  
    ar_separ2char(ar_buff3,1000);
  
    j=0;   //calculate last char
    k=0;
  
    while (j<ar_char_ptr)
    {
      if (ar_len2[j]<=p_len) k=j;
    
      j++;
    }
  
    if (k>0) k--;
  
    for (l=k;l>=0;l--)   // scan string
    {
      c1=ar_char[l][0];
      c2=ar_char[l+1][0];
  
      val1=ar_len2[l+1]*10;  // calculate value
      val2=0;
    
      if (c1==' ')
      {
        val2=0;
      }
      else if (c1=='!')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='"')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='#')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='$')
      {
        if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='%')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='&')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1==39/*'''*/)
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='(')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-80);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-80);
      }
      else if (c1==')')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='*')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='+')
      {
        if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1==',')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='-')
      {
        if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='.')
      {
        if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='/')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if ((c1>='0')&&(c1<='9'))
      {
        if (c2=='.') val2=(-80);
        else if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if ((c2>='A')&&(c2<='Z')) val2=(-80);
        else if ((c2>='a')&&(c2<='z')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1==':')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1==';')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='<')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-80);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-80);
      }
      else if (c1=='=')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='>')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='?')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='@')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if ((c1>='A')&&(c1<='Z'))
      {
        if ((c2>='A')&&(c2<='Z')) val2=(-80);
        else if ((c2>='a')&&(c2<='z')) val2=(-80);
        else if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='[')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-80);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-80);
      }
      else if (c1==92/*'\'*/)
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1==']')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='^')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='_')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-10);
      }
      else if (c1=='`')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if ((c1>='a')&&(c1<='z'))
      {
        if ((c2>='A')&&(c2<='Z')) val2=(-80);
        else if ((c2>='a')&&(c2<='z')) val2=(-80);
        else if ((c2>='0')&&(c2<='9')) val2=(-80);
        else if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='{')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-80);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-80);
      }
      else if (c1=='|')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='}')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
      else if (c1=='~')
      {
        if (c2==' ') val2=0;
        else if (c2=='_') val2=(-10);
        else if ( (c2==')')||(c2=='>')||(c2==']')||(c2=='}') ) val2=(-80);
        else if (c2<0) val2=0;
        else val2=(-30);
      }
//DEL (delete)
      else
      {
        val2=0;
      }
    
      ar_buff1_val[ar_buff1_ptr]=val1+val2;     // store every value to buffer 1
      ar_buff1_len[ar_buff1_ptr]=ar_size2[l+1];
      ar_buff1_ptr++;
      if (ar_buff1_ptr>=1000) ar_buff1_ptr=1000-1;
    }
  
    val3=(-1000000);
    len3=1;
  
    for (l=0;l<ar_buff1_ptr;l++)  //choose max value one
    {
      if (ar_buff1_val[l]>val3)
      {
        val3=ar_buff1_val[l];
        len3=ar_buff1_len[l];
      }
    }
  
    ar_buff2_len[ar_buff2_ptr]=len3;  // store to buffer 2 (result)
    ar_buff2_ptr++;
    if (ar_buff2_ptr>=1000) ar_buff2_ptr=1000-1;
  
    m=0;            // copy left string
    
    for (l=len3;l<strlen(ar_buff3);l++)
    {
      ar_buff3[m]=ar_buff3[l];
      m++;
    }
  
    ar_buff3[m]=0;
  
  }
  
  n=0;                //generate result
  ar_buff4[0][0]=0;
  
  for (l=0;l<ar_buff2_ptr;l++)
  {
    ar_buff4[l][0]=0;
    
    for (m=0;m<ar_buff2_len[l];m++)
    {
      if ((m<0)||(m>=FN_SIZE)) continue;
      if ((m+1<0)||(m+1>=FN_SIZE)) continue;
      if ((n+m<0)||(n+m>=p_in_str_size)) continue;
      
      ar_buff4[l][m+0]=p_in_str[n+m];
      ar_buff4[l][m+1]=0;
    }
    
    n=n+ar_buff2_len[l];
  }
  
  ar_buff4_ptr=ar_buff2_ptr;
  
  return(0);
}

int ar_separ2char(char *instr,int instr_size)
{
  int			i,j;
  unsigned char		c1,c2,c3,c4;
  int                   len,len2,size2;

  if (deb_str_has_null(instr,instr_size)!=1) return(0);

  len=strlen(instr);
  i=len;
  j=0;
  len2=0;
  size2=0;
  ar_char_ptr=0;

  while(j<i)
  {
    if (len>j+0) c1=(unsigned char)instr[j+0];
    else c1=0;
    
    if (len>j+1) c2=(unsigned char)instr[j+1];
    else c2=0;
    
    if (len>j+2) c3=(unsigned char)instr[j+2];
    else c3=0;
    
    u_get_char_bmp(c1,c2,c3);

    if (u_err==0)
    {
      if (u_err2==0)
      {
        if (u_nb==1)
        {
          ar_char[ar_char_ptr][0]=c1;
          ar_char[ar_char_ptr][1]=0;
          ar_len[ar_char_ptr]   =1;
          ar_len2[ar_char_ptr]  =len2;
          ar_size[ar_char_ptr]  =1;
          ar_size2[ar_char_ptr]  =size2;
          len2=len2+1;
          size2=size2+1;
        }
        else if (u_nb==2)
        {
          ar_char[ar_char_ptr][0]=c1;
          ar_char[ar_char_ptr][1]=c2;
          ar_char[ar_char_ptr][2]=0;
          ar_len[ar_char_ptr]   =u_char_size_x/7;
          ar_len2[ar_char_ptr]  =len2;
          ar_size[ar_char_ptr]  =2;
          ar_size2[ar_char_ptr]  =size2;
          len2=len2+u_char_size_x/7;
          size2=size2+2;
        }
        else if (u_nb==3)
        {
          ar_char[ar_char_ptr][0]=c1;
          ar_char[ar_char_ptr][1]=c2;
          ar_char[ar_char_ptr][2]=c3;
          ar_char[ar_char_ptr][3]=0;
          ar_len[ar_char_ptr]   =u_char_size_x/7;
          ar_len2[ar_char_ptr]  =len2;
          ar_size[ar_char_ptr]  =3;
          ar_size2[ar_char_ptr] =size2;
          len2=len2+u_char_size_x/7;
          size2=size2+3;
        }
    
        ar_char_ptr++;
        if (ar_char_ptr>=1000) ar_char_ptr=1000-1;
        j=j+u_nb;
        continue;
      }
      else
      {
        ar_char[ar_char_ptr][0]=c1;
        ar_char[ar_char_ptr][1]=c2;
        ar_char[ar_char_ptr][2]=c3;
        ar_char[ar_char_ptr][3]=c4;
        ar_char[ar_char_ptr][4]=0;
        ar_len[ar_char_ptr]   =14/7;
        ar_len2[ar_char_ptr]  =len2;
        ar_size[ar_char_ptr]=4;
        ar_size2[ar_char_ptr]=size2;
        len2=len2+14/7;
        size2=size2+4;

        ar_char_ptr++;
        if (ar_char_ptr>=1000) ar_char_ptr=1000-1;
        j=j+u_nb;
        continue;
      }
    }
    else
    {
      ar_char[ar_char_ptr][0]=c1;
      ar_char[ar_char_ptr][1]=0;
      ar_len[ar_char_ptr]   =7/7;
      ar_len2[ar_char_ptr]  =len2;
      ar_size[ar_char_ptr]  =1;
      ar_size2[ar_char_ptr] =size2;
      len2=len2+7/7;
      size2=size2+1;

      ar_char_ptr++;
      if (ar_char_ptr>=1000) ar_char_ptr=1000-1;
      j=j+u_nb;
      continue;
    }
  }

  return(0);
}

