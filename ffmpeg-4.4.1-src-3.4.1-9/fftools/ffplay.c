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




#include "config.h"
#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/fifo.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/avassert.h"
#include "libavutil/time.h"
#include "libavutil/bprint.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavcodec/avfft.h"
#include "libswresample/swresample.h"

#if CONFIG_AVFILTER
# include "libavfilter/avfilter.h"
# include "libavfilter/buffersink.h"
# include "libavfilter/buffersrc.h"
#endif

#include <SDL.h>
#include <SDL_thread.h>

#include "cmdutils.h"

#include <assert.h>

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




// daipozhi for suound river
#define SAMPLE_ARRAY_SIZE (16 * 8 * 65536)




#define CURSOR_HIDE_DELAY 1000000

#define USE_ONEPASS_SUBTITLE_RENDER 1

static unsigned sws_flags = SWS_BICUBIC;

typedef struct MyAVPacketList {
    AVPacket *pkt;
    int serial;
} MyAVPacketList;

typedef struct PacketQueue {
    AVFifoBuffer *pkt_list;
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
    int channels;
    int64_t channel_layout;
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

/* Common struct for handling all types of decoded data and allocated render buffers. */
typedef struct Frame {
    AVFrame *frame;
    AVSubtitle sub;




    //daipozhi for sdl1
    AVSubtitleRect **subrects;  /* rescaled subtitle rectangles in yuva */




    int serial;
    double pts;           /* presentation timestamp for the frame */
    double duration;      /* estimated duration of the frame */
    int64_t pos;          /* byte position of the frame in the input file */




    // daipozhi for sdl1
    SDL_Overlay *bmp;
    int allocated;




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
    AVInputFormat *iformat;
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




    // daipozhi for sdl1
    int viddec_width;
    int viddec_height;




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
#if CONFIG_AVFILTER
    struct AudioParams audio_filter_src;
#endif
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
    RDFTContext *rdft;
    int rdft_bits;
    FFTSample *rdft_data;
    int xpos;
    double last_vis_time;




    //daipozhi for sdl1
    //SDL_Texture *vis_texture;
    //SDL_Texture *sub_texture;
    //SDL_Texture *vid_texture;
    SDL_Rect last_display_rect;




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
    struct SwsContext *img_convert_ctx;
    struct SwsContext *sub_convert_ctx;
    int eof;

    char *filename;
    int width, height, xleft, ytop;
    int step;

#if CONFIG_AVFILTER
    int vfilter_idx;
    AVFilterContext *in_video_filter;   // the first filter in the video chain
    AVFilterContext *out_video_filter;  // the last filter in the video chain
    AVFilterContext *in_audio_filter;   // the first filter in the audio chain
    AVFilterContext *out_audio_filter;  // the last filter in the audio chain
    AVFilterGraph *agraph;              // audio filter graph
#endif

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


// daipozhi modified 
static  char      deb_ascii_bmp[128][13][6][3];
static  char      deb_chns_bmp2[6][128][13][12][3];
static  char      deb_chns_bmp[128][128][13][12][3];

static  int       deb_fh;
static 	char     *deb_str;
static 	char      deb_scrn_str[2001];
static 	char      deb_scrn_str2[8001];

// daipozhi modified
static 	int    deb_seek_bar_cntr=0;

static 	int    deb_echo_char4seekbar(int x,int y,int ec);
static 	int    deb_echo_str4seekbar(int y1,char *str);

static 	int    deb_echo_char4en(int x,int y,int ec);
static 	int    deb_echo_char4en_black(int x,int y,int ec);
static  int    deb_echo_char4en_green(int x,int y,int ec);

static 	int    deb_echo_char4chns(int x,int y,int ,int);
static 	int    deb_echo_char4chns_black(int x,int y,int ,int);
static  int    deb_echo_char4chns_green(int x,int y,int k,int l);

static 	int    deb_echo_str4screenstring(int xx,int yy,char *str,int len);
static 	int    deb_echo_str4screenstring_black(int xx,int yy,char *str,int len);
static  int    deb_echo_str4screenstring_green(int xx,int yy,char *str,int len);

static 	int    deb_ch_h=13;
static 	int    deb_ch_w=6;
static 	int    deb_ch_m=30;
static 	int    deb_ch_d=0;

#define FN_SIZE       300
#define MAX_FILE_NUM  30000

static  char   deb_filenamebuff[MAX_FILE_NUM][FN_SIZE];
static  int    deb_filenamebuffptr[MAX_FILE_NUM];
static  char   deb_filenamebuff_ext[MAX_FILE_NUM][6];
static  char   deb_filenamebuff_size[MAX_FILE_NUM][7];
static  char   deb_filenamebuff_date[MAX_FILE_NUM][20];
static  int    deb_filenamebuff_len[MAX_FILE_NUM];
static  char   deb_filenamebuff_type[MAX_FILE_NUM];
static 	int    deb_filenamebuff_n;
static 	int    deb_filenamecnt;
static 	int    deb_filenameplay;
static 	char   deb_currentpath[FN_SIZE];

// daipozhi modified 
static 	int    deb_get_dir(void);
static 	char   deb_lower(char c1);
static 	int    deb_lower_string(char *p_instr);
static 	char   deb_upper(char c1);
static 	int    deb_upper_string(char *p_instr);
static 	int    deb_string2int(char *string,int p1,int p2);
static 	int    deb_disp_dir(VideoState *is);
static 	int    deb_disp_bar(VideoState *is);
static  int    deb_utf8_to_gb18030(char *inbuffer,char *outbuffer,int outbufferlen);
static 	int    deb_disp_scrn(VideoState *is);
//static 	int    deb_filenameext(char *path,char *name,char *fext);
static 	int    deb_filename_dir(char *path,char *name);

static 	int deb_video_open_again(VideoState *is, int force_set_video_mode);
//static 	int deb_ini_var(void);
static 	int deb_ini_is(VideoState *is);

static 	int deb_st_play=0;

static 	int deb_eo_stream;
static  int deb_seek_stt=0;

static 	int deb_thr_v=0;
static 	int deb_thr_s=0;
static 	int deb_thr_r=0;
static 	int deb_thr_a =0;
static 	int deb_thr_a2=0;

static 	int deb_thr_timer_id;
static 	int deb_thr_timer_id_old;
static 	int deb_thr_timer_d;
static 	int deb_thr_frame_id;
static 	int deb_thr_frame_id_old;
static 	int deb_thr_frame_d;

static 	int deb_load_font(void);

static 	int  deb_record_init(void);
static 	int  deb_record_close(void);
static FILE *deb_record_fp;
static 	int  deb_record(char *p_str1);


static 	char deb_tableline[3000];

static  int  deb_cover=0;
static  int  deb_cover_close=0;
static  int  deb_border=0;
static 	int  deb_frame_num=0;
static  int  deb_stream_err=0;
static  int  deb_opts_stt=0;

static  char deb_dir_buffer[FN_SIZE];
static  char deb_dir_buffer_wchar[FN_SIZE];

// daipozhi modified 
static 	int  deb_get_dir_ini(void);
static 	int  deb_dir_opened(int ptr );
static 	int  deb_get_space(char *buffer);
static 	char deb_getfirstchar(char *buffer);
static 	int  deb_dir_add_after(int ptr);
static 	int  deb_dir_remove_after(int ptr);
static 	int  deb_get_path(int ptr);
static 	int  deb_get_path1(char *buffer1,char *buffer2);
static 	int  deb_get_path2(char *buffer1,char *buffer2);
static 	int  deb_cmp_dir(char *buffer1,char *buffer2);

//static 	int deb_supported_formats(char *p_str);
static 	int deb_filenameext2(char *path,char *fext);

static  int deb_str_has_null(char *s1,int s1_size);

#if !defined(_WIN32) && !defined(__APPLE__)
  static  struct stat64 deb_m_info;
#else
  static  struct stat   deb_m_info;
#endif

static  struct tm*    deb_m_info_tm;
static  int           deb_m_info_len;
static  char          deb_m_info_type;

static int deb_size_format(int pn,char *buffer);
static int deb_get_dir_len(int ptr);


// ---- binary tree ----------------------------------------------------------

#define BTREE1_SIZE   30000
#define BTREE1_LSIZE  30000

static    char  t1_node_mark[BTREE1_SIZE];
static    char  t1_node_val[BTREE1_SIZE][FN_SIZE];
static    char  t1_node_val2[BTREE1_SIZE];
static    char  t1_node_val3[BTREE1_SIZE][6];
static    char  t1_node_val4[BTREE1_SIZE][7];
static    char  t1_node_val5[BTREE1_SIZE][20];

static    int   t1_stack[BTREE1_SIZE];
static    int   t1_stack_ptr;

static    int   t1_parent;
static    int   t1_parent_side;
static    int   t1_current;
static    int   t1_child_left;
static    int   t1_child_right;

static    int   t1_node_ptr[BTREE1_SIZE][3];
static    int   t1_root_ptr;

static    int   t1_find_ptr;
static    int   t1_find_ptr2;
static    int   t1_find_side;
    
static    int   t1_list_stack[BTREE1_LSIZE];
static    char  t1_list_stack_type[BTREE1_LSIZE];
static    int   t1_list_ptr;

    //int   t1_out_buff[BTREE1_SIZE];
    //int   t1_out_ptr;

static     char  t1_out_buff[BTREE1_SIZE][FN_SIZE];
static     char  t1_out_buff2[BTREE1_SIZE];
static     char  t1_out_buff3[BTREE1_SIZE][6];
static     char  t1_out_buff4[BTREE1_SIZE][7];
static     char  t1_out_buff5[BTREE1_SIZE][20];
static     int   t1_out_ptr;
static     int   t1_out_ptr2;

static    int   t1_err;

static    int   t1_init_tree(void);
static    int   t1_new_node(void);
static    int   t1_old_node(int ptr);
static    int   t1_clear_node(int ptr);
static    int   t1_search_node(char *pstr,char ptype);
static    int   t1_insert_node(char *pstr,char ptype);
static    int   t1_delete_node(char *pstr,char ptype);
static    int   t1_smallest(void);
//static    int   t1_istrcmp(int i,long long int pn1,long long int pn2,long long int pn3,long long int pn4);
static int string_comp(char *ps1,char *ps2);
static    int   t1_dsp_tree(void);
static    int   t1_after_list(void);
static    int   t1_out_list(char *pstr,char ptype,int ptr);
//static    int   t1_dsp_list(void);
//static    int   t1_save_list(char *fn);

//static     int   bt_opendir(void);
//static     int   bt_readdir(void);

static     char  str_lower(char);
static     int   str_lower_string(char *,char *);

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
//static long long int 	  deb_sr_time4;
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
static int deb_sr_win_top;

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

//static        int     char2int(char *string,int p1,int p2);
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
//static int   deb_sr_river_f[2][2][2000];
//static float deb_sr_river_f2[2][2][2000];

static int  deb_sr_river_f_init;
static int  deb_sr_river_f_init_fail;
static int  deb_sr_river_f_cons(void);
static int  deb_sr_river_f_cons_test(VideoState *cur_stream,int ptr);
static int  deb_sr_river_show(VideoState *cur_stream);

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

// ------- resample ------------------------------------------------------

static int deb_resam_rate;
static int deb_resam_dir;

static int deb_resam_f[450];

static short int deb_resam_buff1[256000];
static short int deb_resam_buff2[256000];
static short int deb_resam_buff3[20];

static int deb_resam_buff1_ptr;
static int deb_resam_buff2_ptr;
static int deb_resam_buff3_ptr;
static int deb_resam_buff2_ind;
static int deb_resam_last_ptr;

// ------- end of resample ------------------------------------------------




/* options specified by the user */
static AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;




//daipozhi for sdl1
static int fs_screen_width;
static int fs_screen_height;




//daipozhi for sound river
static int default_width  = 1224;
static int default_height = 725;




static int screen_width  = 0;
static int screen_height = 0;




// daipozhi modified
static int screen_left = 0/*SDL_WINDOWPOS_CENTERED*/;
static int screen_top = 0/*SDL_WINDOWPOS_CENTERED*/;




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
static int autoexit=1;




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
#if CONFIG_AVFILTER
static const char **vfilters_list = NULL;
static int nb_vfilters = 0;
static char *afilters = NULL;
#endif
static int autorotate = 1;
static int find_stream_info = 1;
static int filter_nbthreads = 0;

/* current context */
static int is_full_screen;
static int64_t audio_callback_time;




// daipozhi modified
#define FF_ALLOC_EVENT   (SDL_USEREVENT)




#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)




//daipozhi for sdl1
//static SDL_Window *window;
//static SDL_Renderer *renderer;
//static SDL_RendererInfo renderer_info = {0};
//static SDL_AudioDeviceID audio_dev;
static SDL_Surface *screen;




//daipozhi for sdl1
#define SDL_PIXELFORMAT_RGB332   0
#define SDL_PIXELFORMAT_RGB444   0
#define SDL_PIXELFORMAT_RGB555   0
#define SDL_PIXELFORMAT_BGR555   0
#define SDL_PIXELFORMAT_RGB565   0
#define SDL_PIXELFORMAT_BGR565   0
#define SDL_PIXELFORMAT_RGB24    0
#define SDL_PIXELFORMAT_BGR24    0
#define SDL_PIXELFORMAT_RGB888   0
#define SDL_PIXELFORMAT_BGR888   0
#define SDL_PIXELFORMAT_RGBX8888 0 
#define SDL_PIXELFORMAT_BGRX8888 0
#define SDL_PIXELFORMAT_ARGB8888 0
#define SDL_PIXELFORMAT_RGBA8888 0
#define SDL_PIXELFORMAT_ABGR8888 0
#define SDL_PIXELFORMAT_BGRA8888 0
#define SDL_PIXELFORMAT_IYUV     0
#define SDL_PIXELFORMAT_YUY2     0
#define SDL_PIXELFORMAT_UYVY     0
#define SDL_PIXELFORMAT_UNKNOWN  0




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

#if CONFIG_AVFILTER
static int opt_add_vfilter(void *optctx, const char *opt, const char *arg)
{
    GROW_ARRAY(vfilters_list, nb_vfilters);
    vfilters_list[nb_vfilters - 1] = arg;
    return 0;
}
#endif

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

static inline
int64_t get_valid_channel_layout(int64_t channel_layout, int channels)
{
    if (channel_layout && av_get_channel_layout_nb_channels(channel_layout) == channels)
        return channel_layout;
    else
        return 0;
}

static int packet_queue_put_private(PacketQueue *q, AVPacket *pkt)
{
    MyAVPacketList pkt1;

    if (q->abort_request)
       return -1;

    if (av_fifo_space(q->pkt_list) < sizeof(pkt1)) {
        if (av_fifo_grow(q->pkt_list, sizeof(pkt1)) < 0)
            return -1;
    }

    pkt1.pkt = pkt;
    pkt1.serial = q->serial;

    av_fifo_generic_write(q->pkt_list, &pkt1, sizeof(pkt1), NULL);
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
    q->pkt_list = av_fifo_alloc(sizeof(MyAVPacketList));
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
    while (av_fifo_size(q->pkt_list) >= sizeof(pkt1)) {
        av_fifo_generic_read(q->pkt_list, &pkt1, sizeof(pkt1), NULL);
        av_packet_free(&pkt1.pkt);
    }
    q->nb_packets = 0;
    q->size = 0;
    q->duration = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_destroy(PacketQueue *q)
{
    packet_queue_flush(q);
    av_fifo_freep(&q->pkt_list);
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

        if (av_fifo_size(q->pkt_list) >= sizeof(pkt1)) {
            av_fifo_generic_read(q->pkt_list, &pkt1, sizeof(pkt1), NULL);
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




    //daipozhi for sdl1
    int i;
    for (i = 0; i < vp->sub.num_rects; i++) {
        av_freep(&vp->subrects[i]->data[0]);
        av_freep(&vp->subrects[i]);
    }
    av_freep(&vp->subrects);




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

static void frame_queue_destory(FrameQueue *f)
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




//daipozhi modified
static inline void fill_rectangle(/*int x, int y, int w, int h*/SDL_Surface *screen,
                                  int x, int y, int w, int h, int color, int update )
{
/*
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    if (w && h)
        SDL_RenderFillRect(renderer, &rect);
*/

    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    SDL_FillRect(screen, &rect, color);
    if (update && w > 0 && h > 0)
        SDL_UpdateRect(screen, x, y, w, h);
}




//daipozhi for sdl1
/* draw only the border of a rectangle */
static void fill_border(int xleft, int ytop, int width, int height, int x, int y, int w, int h, int color, int update)
{
    int w1, w2, h1, h2;

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

    h2 = height - (y + h) -deb_ch_h*2 -deb_ch_d ;  //daipozhi modified

    if (h2 < 0)
        h2 = 0;

    fill_rectangle(screen,
                   xleft, ytop,
                   w1, height -deb_ch_h*2 -deb_ch_d,  //daipozhi modified
                   color, update);
    fill_rectangle(screen,
                   xleft + width - w2, ytop,
                   w2, height -deb_ch_h*2 -deb_ch_d ,  //daipozhi modified
                   color, update);
    fill_rectangle(screen,
                   xleft + w1, ytop,
                   width - w1 - w2, h1,
                   color, update);
    fill_rectangle(screen,
                   xleft + w1, ytop + height - h2 -deb_ch_h*2 -deb_ch_d, //daipozhi modified
                   width - w1 - w2, h2,
                   color, update);
}

// daipozhi for sdl1
#define ALPHA_BLEND(a, oldp, newp, s)\
((((oldp << s) * (255 - (a))) + (newp * (a))) / (255 << s))

#define BPP 1

static void blend_subrect(uint8_t **data, int *linesize, const AVSubtitleRect *rect, int imgw, int imgh)
{
    int x, y, Y, U, V, A;
    uint8_t *lum, *cb, *cr;
    int dstx, dsty, dstw, dsth;
    const AVSubtitleRect *src = rect;

    dstw = av_clip(rect->w, 0, imgw);
    dsth = av_clip(rect->h, 0, imgh);
    dstx = av_clip(rect->x, 0, imgw - dstw);
    dsty = av_clip(rect->y, 0, imgh - dsth);
    lum = data[0] + dstx + dsty * linesize[0];
    cb  = data[1] + dstx/2 + (dsty >> 1) * linesize[1];
    cr  = data[2] + dstx/2 + (dsty >> 1) * linesize[2];

    for (y = 0; y<dsth; y++) {
        for (x = 0; x<dstw; x++) {
            Y = src->data[0][x + y*src->linesize[0]];
            A = src->data[3][x + y*src->linesize[3]];
            lum[0] = ALPHA_BLEND(A, lum[0], Y, 0);
            lum++;
        }
        lum += linesize[0] - dstw;
    }

    for (y = 0; y<dsth/2; y++) {
        for (x = 0; x<dstw/2; x++) {
            U = src->data[1][x + y*src->linesize[1]];
            V = src->data[2][x + y*src->linesize[2]];
            A = src->data[3][2*x     +  2*y   *src->linesize[3]]
              + src->data[3][2*x + 1 +  2*y   *src->linesize[3]]
              + src->data[3][2*x + 1 + (2*y+1)*src->linesize[3]]
              + src->data[3][2*x     + (2*y+1)*src->linesize[3]];
            cb[0] = ALPHA_BLEND(A>>2, cb[0], U, 0);
            cr[0] = ALPHA_BLEND(A>>2, cr[0], V, 0);
            cb++;
            cr++;
        }
        cb += linesize[1] - dstw/2;
        cr += linesize[2] - dstw/2;
    }
}

static void free_picture(Frame *vp)
{




     // daipozhi for sdl1
     /*
     if (vp->bmp) {
         SDL_DestroyTexture(vp->bmp);
         vp->bmp = NULL;
     }*/

     if (vp->bmp) {
         SDL_FreeYUVOverlay(vp->bmp);
         vp->bmp = NULL;
     }




}




//daipozhi modified for sdl1
/*static int realloc_texture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture)
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
            memset(pixels, 0, pitch * new_height);
            SDL_UnlockTexture(*texture);
        }
        av_log(NULL, AV_LOG_VERBOSE, "Created %dx%d texture with %s.\n", new_width, new_height, SDL_GetPixelFormatName(new_format));
    }
    return 0;
}
*/




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
    height = scr_height -deb_ch_h*2 -deb_ch_d ;




    width = av_rescale(height, aspect_ratio.num, aspect_ratio.den) & ~1;
    if (width > scr_width) {
        width = scr_width;
        height = av_rescale(width, aspect_ratio.den, aspect_ratio.num) & ~1;
    }
    x = (scr_width - width) / 2;




    // daipozhi modified
    y = (scr_height - height -deb_ch_h*2 -deb_ch_d ) / 2;




    rect->x = scr_xleft + x;
    rect->y = scr_ytop  + y;
    rect->w = FFMAX((int)width,  1);
    rect->h = FFMAX((int)height, 1);
}




// daipozhi modified
/*static void get_sdl_pix_fmt_and_blendmode(int format, Uint32 *sdl_pix_fmt, SDL_BlendMode *sdl_blendmode)
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
}*/




//daipozhi modified
static int upload_texture(/*SDL_Texture **tex*/SDL_Overlay *tex, AVFrame *frame, struct SwsContext **img_convert_ctx) {
    int ret = 0;




    //daipozhi modified
    Uint32 sdl_pix_fmt = SDL_PIXELFORMAT_UNKNOWN;




    //SDL_BlendMode sdl_blendmode;
    /*get_sdl_pix_fmt_and_blendmode(frame->format, &sdl_pix_fmt, &sdl_blendmode);
    if (realloc_texture(tex, sdl_pix_fmt == SDL_PIXELFORMAT_UNKNOWN ? SDL_PIXELFORMAT_ARGB8888 : sdl_pix_fmt, frame->width, frame->height, sdl_blendmode, 0) < 0)
        return -1;*/
    switch (sdl_pix_fmt) {
/*
        case SDL_PIXELFORMAT_UNKNOWN:
            // This should only happen if we are not using avfilter... 
            *img_convert_ctx = sws_getCachedContext(*img_convert_ctx,
                frame->width, frame->height, frame->format, frame->width, frame->height,
                AV_PIX_FMT_BGRA, sws_flags, NULL, NULL, NULL);
            if (*img_convert_ctx != NULL) {
                uint8_t *pixels[4];
                int pitch[4];
                if (!SDL_LockTexture(*tex, NULL, (void **)pixels, pitch)) {
                    sws_scale(*img_convert_ctx, (const uint8_t * const *)frame->data, frame->linesize,
                              0, frame->height, pixels, pitch);
                    SDL_UnlockTexture(*tex);
                }
            } else {
                av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
                ret = -1;
            }
            break;
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
            break;*/
        default:
/*
            if (frame->linesize[0] < 0) {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0]);
            } else {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0], frame->linesize[0]);
            }*/
            break;
    }
    return ret;
}




static void set_sdl_yuv_conversion_mode(AVFrame *frame)
{
#if SDL_VERSION_ATLEAST(2,0,8)
    SDL_YUV_CONVERSION_MODE mode = SDL_YUV_CONVERSION_AUTOMATIC;
    if (frame && (frame->format == AV_PIX_FMT_YUV420P || frame->format == AV_PIX_FMT_YUYV422 || frame->format == AV_PIX_FMT_UYVY422)) {
        if (frame->color_range == AVCOL_RANGE_JPEG)
            mode = SDL_YUV_CONVERSION_JPEG;
        else if (frame->colorspace == AVCOL_SPC_BT709)
            mode = SDL_YUV_CONVERSION_BT709;
        else if (frame->colorspace == AVCOL_SPC_BT470BG || frame->colorspace == AVCOL_SPC_SMPTE170M || frame->colorspace == AVCOL_SPC_SMPTE240M)
            mode = SDL_YUV_CONVERSION_BT601;
    }
    SDL_SetYUVConversionMode(mode);
#endif
}

static void video_image_display(VideoState *is)
{
    Frame *vp;
    Frame *sp = NULL;
    SDL_Rect rect;




    //daipozhi modified
    int bgcolor; 
    int i;
    if (deb_cover_close==1) return ;




    vp = frame_queue_peek_last(&is->pictq);




    //daipozhi modified
    if (deb_border==0)  
    {
      deb_border=1;
      bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);  //daipozhi modified
      fill_rectangle(screen,0, 0, is->width, is->height -deb_ch_h*2-deb_ch_d , bgcolor,1);
      SDL_UpdateRect(screen,0, 0, is->width, is->height -deb_ch_h*2-deb_ch_d );
    }




    if (is->subtitle_st) {
        if (frame_queue_nb_remaining(&is->subpq) > 0) {
            sp = frame_queue_peek(&is->subpq);

            if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {




		    // daipozhi for sdl1
                    uint8_t *data[4];
                    int linesize[4];

                    SDL_LockYUVOverlay (vp->bmp);

                    data[0] = vp->bmp->pixels[0];
                    data[1] = vp->bmp->pixels[2];
                    data[2] = vp->bmp->pixels[1];

                    linesize[0] = vp->bmp->pitches[0];
                    linesize[1] = vp->bmp->pitches[2];
                    linesize[2] = vp->bmp->pitches[1];

                    for (i = 0; i < sp->sub.num_rects; i++)
                        blend_subrect(data, linesize, sp->subrects[i],
                                      vp->bmp->w, vp->bmp->h);

                    SDL_UnlockYUVOverlay (vp->bmp);




                //daipozhi modified
                /*if (!sp->uploaded) {
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
                }*/




            } else
                sp = NULL;
        }
    }

    calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);




    //daipozhi for sdl1
    /*if (!vp->uploaded) {
        if (upload_texture(&is->vid_texture, vp->frame, &is->img_convert_ctx) < 0)
            return;
        vp->uploaded = 1;
        vp->flip_v = vp->frame->linesize[0] < 0;
    }

    set_sdl_yuv_conversion_mode(vp->frame);
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
    }*/




	//daipozhi for sdl1
        SDL_DisplayYUVOverlay(vp->bmp, &rect);

        if (rect.x != is->last_display_rect.x || rect.y != is->last_display_rect.y || rect.w != is->last_display_rect.w || rect.h != is->last_display_rect.h || is->force_refresh) {
            int bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);
            fill_border(is->xleft, is->ytop, is->width, is->height, rect.x, rect.y, rect.w, rect.h, bgcolor, 1);
            is->last_display_rect = rect;
        }

        deb_cover=1;




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

    // compute display index : center on currently output samples 
    channels = s->audio_tgt.channels;
    nb_display_channels = channels;
    if (!s->paused) {
        int data_used= s->show_mode == SHOW_MODE_WAVES ? s->width : (2*nb_freq);
        n = 2 * channels;
        delay = s->audio_write_buf_size;
        delay /= n;

        // to be more precise, we take into account the time spent since
        //   the last buffer computation 
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

        // total height for one channel 
        h = s->height / nb_display_channels;
        // graph height / 2 
        h2 = (h * 9) / 20;
        for (ch = 0; ch < nb_display_channels; ch++) {
            i = i_start + ch;
            y1 = s->ytop + ch * h + (h / 2); // position of center line 
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
        if (realloc_texture(&s->vis_texture, SDL_PIXELFORMAT_ARGB8888, s->width, s->height, SDL_BLENDMODE_NONE, 1) < 0)
            return;

        if (s->xpos >= s->width)
            s->xpos = 0;
        nb_display_channels= FFMIN(nb_display_channels, 2);
        if (rdft_bits != s->rdft_bits) {
            av_rdft_end(s->rdft);
            av_free(s->rdft_data);
            s->rdft = av_rdft_init(rdft_bits, DFT_R2C);
            s->rdft_bits = rdft_bits;
            s->rdft_data = av_malloc_array(nb_freq, 4 *sizeof(*s->rdft_data));
        }
        if (!s->rdft || !s->rdft_data){
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffers for RDFT, switching to waves display\n");
            s->show_mode = SHOW_MODE_WAVES;
        } else {
            FFTSample *data[2];
            SDL_Rect rect = {.x = s->xpos, .y = 0, .w = 1, .h = s->height};
            uint32_t *pixels;
            int pitch;
            for (ch = 0; ch < nb_display_channels; ch++) {
                data[ch] = s->rdft_data + 2 * nb_freq * ch;
                i = i_start + ch;
                for (x = 0; x < 2 * nb_freq; x++) {
                    double w = (x-nb_freq) * (1.0 / nb_freq);
                    data[ch][x] = s->sample_array[i] * (1.0 - w * w);
                    i += channels;
                    if (i >= SAMPLE_ARRAY_SIZE)
                        i -= SAMPLE_ARRAY_SIZE;
                }
                av_rdft_calc(s->rdft, data[ch]);
            }
            // Least efficient way to do this, we should of course
            // directly access it but it is more than fast enough. 
            if (!SDL_LockTexture(s->vis_texture, &rect, (void **)&pixels, &pitch)) {
                pitch >>= 2;
                pixels += pitch * s->height;
                for (y = 0; y < s->height; y++) {
                    double w = 1 / sqrt(nb_freq);
                    int a = sqrt(w * sqrt(data[0][2 * y + 0] * data[0][2 * y + 0] + data[0][2 * y + 1] * data[0][2 * y + 1]));
                    int b = (nb_display_channels == 2 ) ? sqrt(w * hypot(data[1][2 * y + 0], data[1][2 * y + 1]))
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
    }*/




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




	// daipozhi modified
        //SDL_CloseAudioDevice(audio_dev);
        SDL_CloseAudio();




        decoder_destroy(&is->auddec);
        swr_free(&is->swr_ctx);
        av_freep(&is->audio_buf1);
        is->audio_buf1_size = 0;
        is->audio_buf = NULL;

        if (is->rdft) {
            av_rdft_end(is->rdft);
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
    frame_queue_destory(&is->pictq);
    frame_queue_destory(&is->sampq);
    frame_queue_destory(&is->subpq);
    SDL_DestroyCond(is->continue_read_thread);
    sws_freeContext(is->img_convert_ctx);
    sws_freeContext(is->sub_convert_ctx);
    av_free(is->filename);




    //daipozhi for sdl1
    //if (is->vis_texture)
    //    SDL_DestroyTexture(is->vis_texture);
    //if (is->vid_texture)
    //    SDL_DestroyTexture(is->vid_texture);
    //if (is->sub_texture)
    //    SDL_DestroyTexture(is->sub_texture);
    //av_free(is);




    //daipozhi modified
    deb_st_play=0;




}

static void do_exit(VideoState *is)
{
    if (is) {
        stream_close(is);
    }




    //daipozhi for sdl1
    //if (renderer)
    //    SDL_DestroyRenderer(renderer);
    //if (window)
    //    SDL_DestroyWindow(window);




    uninit_opts();




    //daipozhi modified
    deb_opts_stt=0;




#if CONFIG_AVFILTER
    av_freep(&vfilters_list);
#endif
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




//daipozhi modified
static int video_open(VideoState *is, int force_set_video_mode, Frame *vp)
{
/*
    int w,h;

    w = screen_width ? screen_width : default_width;
    h = screen_height ? screen_height : default_height;

    if (!window_title)
        window_title = input_filename;
    SDL_SetWindowTitle(window, window_title);

    SDL_SetWindowSize(window, w, h);
    SDL_SetWindowPosition(window, screen_left, screen_top);
    if (is_full_screen)
        SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    SDL_ShowWindow(window);

    is->width  = w;
    is->height = h;

    return 0;
*/

    int flags = SDL_HWSURFACE | SDL_ASYNCBLIT | SDL_HWACCEL;
    int w,h;
    //daipozhi modified 
    int tmp_n1,tmp_n2;

    if (is_full_screen) flags |= SDL_FULLSCREEN;
    else                flags |= SDL_RESIZABLE;

    if (vp && vp->width)
        set_default_window_size(vp->width, vp->height, vp->sar);

    //daipozhi modified
    if (is_full_screen && fs_screen_width) {
        w = fs_screen_width;
        h = fs_screen_height;
    } else if (!is_full_screen && screen_width) {
        w = screen_width;
        h = screen_height;
    } else {
        w = 1224;	//daipozhi modified 
        h = 725;	//daipozhi modified 
    }

    //daipozhi modified
    tmp_n1 = deb_ch_w*2*(w /(deb_ch_w*2)) ;
    tmp_n2 = deb_ch_h*1*(h /(deb_ch_h*1))+7 ;

    w=tmp_n1;
    h=tmp_n2;

    w = FFMIN(16383, w);
    if (screen && is->width == screen->w && screen->w == w
       && is->height== screen->h && screen->h == h && !force_set_video_mode)
        return 0;
    screen = SDL_SetVideoMode(w, h, 0, flags);
    if (!screen) {
        av_log(NULL, AV_LOG_FATAL, "SDL: could not set video mode - exiting\n");
        do_exit(is);
    }

    //daipozhi modified
    //if (!window_title)
    //    window_title = input_filename;
    //SDL_WM_SetCaption(window_title, window_title);

    is->width  = screen->w;
    is->height = screen->h;

    //daipozhi modified
    deb_ch_d= screen->h - deb_ch_h *( screen->h / deb_ch_h ) ;  

    return 0;
}




//daipozhi modified
static int deb_video_open_again(VideoState *is, int force_set_video_mode)//daipozhi modified
{
    int i;

    is->width  = screen->w;
    is->height = screen->h;

    deb_ch_d= screen->h - deb_ch_h *( screen->h / deb_ch_h ) ;

    //daipozhi modified 
    //if (!window_title)
    //    window_title = deb_dir_buffer;

#if !defined(_WIN32) && !defined(__APPLE__)
    SDL_WM_SetCaption(deb_dir_buffer, deb_dir_buffer);
#else
    for (i=0;i<FN_SIZE;i++) deb_dir_buffer_wchar[i]=0;
    deb_utf8_to_gb18030(deb_dir_buffer,deb_dir_buffer_wchar,FN_SIZE);
    SDL_WM_SetCaption(deb_dir_buffer_wchar, deb_dir_buffer_wchar);
#endif
    return 0;
}




/* display the current picture, if any */
static void video_display(VideoState *is)
{




    // daipozhi modified for sdl1
    /*if (!is->width)
        video_open(is);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
        video_audio_display(is);
    else if (is->video_st)
        video_image_display(is);
    SDL_RenderPresent(renderer);*/

    if (!screen)
        video_open(is, 0, NULL);
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
        video_audio_display(is);
    else if (is->video_st)
        video_image_display(is);




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
static void stream_seek(VideoState *is, int64_t pos, int64_t rel, int seek_by_bytes)
{
    if (!is->seek_req) {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (seek_by_bytes)
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
    /*double volume_level = is->audio_volume ? (20 * log(is->audio_volume / (double)SDL_MIX_MAXVOLUME) / log(10)) : -1000.0;
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

static void update_video_pts(VideoState *is, double pts, int64_t pos, int serial) {
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
                update_video_pts(is, vp->pts, vp->pos, vp->serial);
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




                        //daipozhi modified
                        /*if (sp->uploaded) {
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
                        }*/




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
                      "%7.2f %s:%7.3f fd=%4d aq=%5dKB vq=%5dKB sq=%5dB f=%"PRId64"/%"PRId64"   \r",
                      get_master_clock(is),
                      (is->audio_st && is->video_st) ? "A-V" : (is->video_st ? "M-V" : (is->audio_st ? "M-A" : "   ")),
                      av_diff,
                      is->frame_drops_early + is->frame_drops_late,
                      aqsize / 1024,
                      vqsize / 1024,
                      sqsize,
                      is->video_st ? is->viddec.avctx->pts_correction_num_faulty_dts : 0,
                      is->video_st ? is->viddec.avctx->pts_correction_num_faulty_pts : 0);

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




//daipozhi for sdl1
static void duplicate_right_border_pixels(SDL_Overlay *bmp) {
    int i, width, height;
    Uint8 *p, *maxp;
    for (i = 0; i < 3; i++) {
        width  = bmp->w;
        height = bmp->h;
        if (i > 0) {
            width  >>= 1;
            height >>= 1;
        }
        if (bmp->pitches[i] > width) {
            maxp = bmp->pixels[i] + bmp->pitches[i] * height - 1;
            for (p = bmp->pixels[i] + width - 1; p < maxp; p += bmp->pitches[i])
                *(p+1) = *p;
        }
    }
}




// daipozhi modified
/* allocate a picture (needs to do that in main thread to avoid
   potential locking problems */
static void alloc_picture(VideoState *is)
{
    // daipozhi for sdl1
    /*
    Frame *vp;
    int sdl_format;

    vp = &is->pictq.queue[is->pictq.windex];

    video_open(is, vp);

    if (vp->format == AV_PIX_FMT_YUV420P)
        sdl_format = SDL_PIXELFORMAT_YV12;
    else
        sdl_format = SDL_PIXELFORMAT_ARGB8888;

    if (realloc_texture(&vp->bmp, sdl_format, vp->width, vp->height, SDL_BLENDMODE_NONE, 0) < 0) {
        // SDL allocates a buffer smaller than requested if the video
        // overlay hardware is unable to support the requested size. 
        av_log(NULL, AV_LOG_FATAL,
               "Error: the video system does not support an image\n"
                        "size of %dx%d pixels. Try using -lowres or -vf \"scale=w:h\"\n"
                        "to reduce the image size.\n", vp->width, vp->height );
        do_exit(is);
    }

    SDL_LockMutex(is->pictq.mutex);
    vp->allocated = 1;
    SDL_CondSignal(is->pictq.cond);
    SDL_UnlockMutex(is->pictq.mutex);*/

    Frame *vp;
    int64_t bufferdiff;

    vp = &is->pictq.queue[is->pictq.windex];

    free_picture(vp);

    video_open(is, 0, vp);

    vp->bmp = SDL_CreateYUVOverlay(vp->width, vp->height,
                                   SDL_YV12_OVERLAY,
                                   screen);
    bufferdiff = vp->bmp ? FFMAX(vp->bmp->pixels[0], vp->bmp->pixels[1]) - FFMIN(vp->bmp->pixels[0], vp->bmp->pixels[1]) : 0;
    if (!vp->bmp || vp->bmp->pitches[0] < vp->width || bufferdiff < (int64_t)vp->height * vp->bmp->pitches[0]) {
        /* SDL allocates a buffer smaller than requested if the video
         * overlay hardware is unable to support the requested size. */
        av_log(NULL, AV_LOG_FATAL,
               "Error: the video system does not support an image\n"
                        "size of %dx%d pixels. Try using -lowres or -vf \"scale=w:h\"\n"
                        "to reduce the image size.\n", vp->width, vp->height );
        do_exit(is);
    }

    SDL_LockMutex(is->pictq.mutex);
    vp->allocated = 1;
    SDL_CondSignal(is->pictq.cond);
    SDL_UnlockMutex(is->pictq.mutex);




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




    // daipozhi modified
    /*vp->width = src_frame->width;
    vp->height = src_frame->height;
    vp->format = src_frame->format;

    vp->pts = pts;
    vp->duration = duration;
    vp->pos = pos;
    vp->serial = serial;

    set_default_window_size(vp->width, vp->height, vp->sar);

    av_frame_move_ref(vp->frame, src_frame);
    frame_queue_push(&is->pictq);*/

    /* alloc or resize hardware picture buffer */
    if (!vp->bmp || !vp->allocated ||
        vp->width  != src_frame->width ||
        vp->height != src_frame->height ||
        vp->format != src_frame->format) {
        SDL_Event event;

        vp->allocated = 0;
        vp->width = src_frame->width;
        vp->height = src_frame->height;
        vp->format = src_frame->format;

        /* the allocation must be done in the main thread to avoid
           locking problems. */
        event.type = FF_ALLOC_EVENT;
        event.user.data1 = is;
        SDL_PushEvent(&event);

        /* wait until the picture is allocated */
        SDL_LockMutex(is->pictq.mutex);
        while (!vp->allocated && !is->videoq.abort_request) {
            SDL_CondWait(is->pictq.cond, is->pictq.mutex);
        }
        /* if the queue is aborted, we have to pop the pending ALLOC event or wait for the allocation to complete */

	// daipozhi for sdl1
        if (is->videoq.abort_request && SDL_PeepEvents(&event, 1, SDL_GETEVENT, /*FF_ALLOC_EVENT, FF_ALLOC_EVENT*/
SDL_EVENTMASK(FF_ALLOC_EVENT) ) != 1) {
            while (!vp->allocated && !is->abort_request) {
                SDL_CondWait(is->pictq.cond, is->pictq.mutex);
            }
        }

        SDL_UnlockMutex(is->pictq.mutex);

        if (is->videoq.abort_request)
            return -1;
    }

    /* if the frame is not skipped, then display it */
    if (vp->bmp) {
	// daipozhi for sdl1
        uint8_t *data[4];
        int linesize[4];

        /* get a pointer on the bitmap */
        SDL_LockYUVOverlay (vp->bmp);

        data[0] = vp->bmp->pixels[0];
        data[1] = vp->bmp->pixels[2];
        data[2] = vp->bmp->pixels[1];

        linesize[0] = vp->bmp->pitches[0];
        linesize[1] = vp->bmp->pitches[2];
        linesize[2] = vp->bmp->pitches[1];

/*#if CONFIG_AVFILTER
        // FIXME use direct rendering
        av_image_copy(data, linesize, (const uint8_t **)src_frame->data, src_frame->linesize,
                        src_frame->format, vp->width, vp->height);
#else*/
        {
            AVDictionaryEntry *e = av_dict_get(sws_dict, "sws_flags", NULL, 0);
            if (e) {
                const AVClass *class = sws_get_class();
                const AVOption    *o = av_opt_find(&class, "sws_flags", NULL, 0,
                                                   AV_OPT_SEARCH_FAKE_OBJ);
                int ret = av_opt_eval_flags(&class, o, e->value, &sws_flags);
                if (ret < 0)
                    exit(1);
            }
        }

        is->img_convert_ctx = sws_getCachedContext(is->img_convert_ctx,
            vp->width, vp->height, src_frame->format, vp->width, vp->height,
            AV_PIX_FMT_YUV420P, sws_flags, NULL, NULL, NULL);
        if (!is->img_convert_ctx) {
            av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
            exit(1);
        }
        sws_scale(is->img_convert_ctx, src_frame->data, src_frame->linesize,
                  0, vp->height, data, linesize);
//#endif
        /* workaround SDL PITCH_WORKAROUND */
        duplicate_right_border_pixels(vp->bmp);
        /* update the bitmap content */
        SDL_UnlockYUVOverlay(vp->bmp);

        vp->pts = pts;
        vp->duration = duration;
        vp->pos = pos;
        vp->serial = serial;

        av_frame_move_ref(vp->frame, src_frame);
        frame_queue_push(&is->pictq);
    }




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




        // daipozhi for sdl1
        is->viddec_width  = frame->width;
        is->viddec_height = frame->height;




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

#if CONFIG_AVFILTER
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




    // daipozhi modified
    //enum AVPixelFormat pix_fmts[FF_ARRAY_ELEMS(sdl_texture_format_map)];
    enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE };




    char sws_flags_str[512] = "";
    char buffersrc_args[256];
    int ret;
    AVFilterContext *filt_src = NULL, *filt_out = NULL, *last_filter = NULL;
    AVCodecParameters *codecpar = is->video_st->codecpar;
    AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL);
    AVDictionaryEntry *e = NULL;
    int nb_pix_fmts = 0;
    int i, j;




    //daipozhi modified
    /*for (i = 0; i < renderer_info.num_texture_formats; i++) {
        for (j = 0; j < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; j++) {
            if (renderer_info.texture_formats[i] == sdl_texture_format_map[j].texture_fmt) {
                pix_fmts[nb_pix_fmts++] = sdl_texture_format_map[j].format;
                break;
            }
        }
    }
    pix_fmts[nb_pix_fmts] = AV_PIX_FMT_NONE;*/




    while ((e = av_dict_get(sws_dict, "", e, AV_DICT_IGNORE_SUFFIX))) {
        if (!strcmp(e->key, "sws_flags")) {
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", e->value);
        } else
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", e->key, e->value);
    }
    if (strlen(sws_flags_str))
        sws_flags_str[strlen(sws_flags_str)-1] = '\0';

    graph->scale_sws_opts = av_strdup(sws_flags_str);

    snprintf(buffersrc_args, sizeof(buffersrc_args),
             "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
             frame->width, frame->height, frame->format,
             is->video_st->time_base.num, is->video_st->time_base.den,
             codecpar->sample_aspect_ratio.num, FFMAX(codecpar->sample_aspect_ratio.den, 1));
    if (fr.num && fr.den)
        av_strlcatf(buffersrc_args, sizeof(buffersrc_args), ":frame_rate=%d/%d", fr.num, fr.den);

    if ((ret = avfilter_graph_create_filter(&filt_src,
                                            avfilter_get_by_name("buffer"),
                                            "ffplay_buffer", buffersrc_args, NULL,
                                            graph)) < 0)
        goto fail;

    ret = avfilter_graph_create_filter(&filt_out,
                                       avfilter_get_by_name("buffersink"),
                                       "ffplay_buffersink", NULL, NULL, graph);
    if (ret < 0)
        goto fail;

    if ((ret = av_opt_set_int_list(filt_out, "pix_fmts", pix_fmts,  AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
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
        double theta  = get_rotation(is->video_st);

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
    return ret;
}

static int configure_audio_filters(VideoState *is, const char *afilters, int force_output_format)
{
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
    int sample_rates[2] = { 0, -1 };
    int64_t channel_layouts[2] = { 0, -1 };
    int channels[2] = { 0, -1 };
    AVFilterContext *filt_asrc = NULL, *filt_asink = NULL;
    char aresample_swr_opts[512] = "";
    AVDictionaryEntry *e = NULL;
    char asrc_args[256];
    int ret;

    avfilter_graph_free(&is->agraph);
    if (!(is->agraph = avfilter_graph_alloc()))
        return AVERROR(ENOMEM);
    is->agraph->nb_threads = filter_nbthreads;

    while ((e = av_dict_get(swr_opts, "", e, AV_DICT_IGNORE_SUFFIX)))
        av_strlcatf(aresample_swr_opts, sizeof(aresample_swr_opts), "%s=%s:", e->key, e->value);
    if (strlen(aresample_swr_opts))
        aresample_swr_opts[strlen(aresample_swr_opts)-1] = '\0';
    av_opt_set(is->agraph, "aresample_swr_opts", aresample_swr_opts, 0);

    ret = snprintf(asrc_args, sizeof(asrc_args),
                   "sample_rate=%d:sample_fmt=%s:channels=%d:time_base=%d/%d",
                   is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
                   is->audio_filter_src.channels,
                   1, is->audio_filter_src.freq);
    if (is->audio_filter_src.channel_layout)
        snprintf(asrc_args + ret, sizeof(asrc_args) - ret,
                 ":channel_layout=0x%"PRIx64,  is->audio_filter_src.channel_layout);

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
        channel_layouts[0] = is->audio_tgt.channel_layout;
        channels       [0] = is->audio_tgt.channel_layout ? -1 : is->audio_tgt.channels;
        sample_rates   [0] = is->audio_tgt.freq;
        if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 0, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set_int_list(filt_asink, "channel_layouts", channel_layouts,  -1, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set_int_list(filt_asink, "channel_counts" , channels       ,  -1, AV_OPT_SEARCH_CHILDREN)) < 0)
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
    return ret;
}
#endif  /* CONFIG_AVFILTER */

static int audio_thread(void *arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    Frame *af;
#if CONFIG_AVFILTER
    int last_serial = -1;
    int64_t dec_channel_layout;
    int reconfigure;
#endif
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

#if CONFIG_AVFILTER
                dec_channel_layout = get_valid_channel_layout(frame->channel_layout, frame->channels);

                reconfigure =
                    cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.channels,
                                   frame->format, frame->channels)    ||
                    is->audio_filter_src.channel_layout != dec_channel_layout ||
                    is->audio_filter_src.freq           != frame->sample_rate ||
                    is->auddec.pkt_serial               != last_serial;

                if (reconfigure) {
                    char buf1[1024], buf2[1024];
                    av_get_channel_layout_string(buf1, sizeof(buf1), -1, is->audio_filter_src.channel_layout);
                    av_get_channel_layout_string(buf2, sizeof(buf2), -1, dec_channel_layout);
                    av_log(NULL, AV_LOG_DEBUG,
                           "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
                           is->audio_filter_src.freq, is->audio_filter_src.channels, av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
                           frame->sample_rate, frame->channels, av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);

                    is->audio_filter_src.fmt            = frame->format;
                    is->audio_filter_src.channels       = frame->channels;
                    is->audio_filter_src.channel_layout = dec_channel_layout;
                    is->audio_filter_src.freq           = frame->sample_rate;
                    last_serial                         = is->auddec.pkt_serial;

                    if ((ret = configure_audio_filters(is, afilters, 1)) < 0)
                        goto the_end;
                }

            if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0)
                goto the_end;

            while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame, 0)) >= 0) {
                tb = av_buffersink_get_time_base(is->out_audio_filter);
#endif
                if (!(af = frame_queue_peek_writable(&is->sampq)))
                    goto the_end;

                af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
                af->pos = frame->pkt_pos;
                af->serial = is->auddec.pkt_serial;
                af->duration = av_q2d((AVRational){frame->nb_samples, frame->sample_rate});

                av_frame_move_ref(af->frame, frame);
                frame_queue_push(&is->sampq);

#if CONFIG_AVFILTER
                if (is->audioq.serial != is->auddec.pkt_serial)
                    break;
            }
            if (ret == AVERROR_EOF)
                is->auddec.finished = is->auddec.pkt_serial;
#endif




	    deb_thr_a=1;   //daipozhi modified




        }
    } while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);
 the_end:
#if CONFIG_AVFILTER
    avfilter_graph_free(&is->agraph);
#endif
    av_frame_free(&frame);
    return ret;
}

static int decoder_start(Decoder *d, int (*fn)(void *), const char *thread_name, void* arg)
{
    packet_queue_start(d->queue);




    // daipozhi modified
    d->decoder_tid = SDL_CreateThread(fn,/* thread_name,*/ arg);




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

#if CONFIG_AVFILTER
    AVFilterGraph *graph = NULL;
    AVFilterContext *filt_out = NULL, *filt_in = NULL;
    int last_w = 0;
    int last_h = 0;
    enum AVPixelFormat last_format = -2;
    int last_serial = -1;
    int last_vfilter_idx = 0;
#endif

    if (!frame)
        return AVERROR(ENOMEM);

    for (;;) {
        ret = get_video_frame(is, frame);
        if (ret < 0)
            goto the_end;
        if (!ret)
            continue;

#if CONFIG_AVFILTER
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
            is->frame_last_returned_time = av_gettime_relative() / 1000000.0;

            ret = av_buffersink_get_frame_flags(filt_out, frame, 0);
            if (ret < 0) {
                if (ret == AVERROR_EOF)
                    is->viddec.finished = is->viddec.pkt_serial;
                ret = 0;
                break;
            }

            is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
            if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0)
                is->frame_last_filter_delay = 0;
            tb = av_buffersink_get_time_base(filt_out);
#endif
            duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational){frame_rate.den, frame_rate.num}) : 0);
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
            ret = queue_picture(is, frame, pts, duration, frame->pkt_pos, is->viddec.pkt_serial);
            av_frame_unref(frame);
#if CONFIG_AVFILTER
            if (is->videoq.serial != is->viddec.pkt_serial)
                break;
        }
#endif

        if (ret < 0)
            goto the_end;




	deb_thr_v=1;  //daipozhi modified 




    }
 the_end:
#if CONFIG_AVFILTER
    avfilter_graph_free(&graph);
#endif
    av_frame_free(&frame);
    return 0;
}

static int subtitle_thread(void *arg)
{
    VideoState *is = arg;
    Frame *sp;
    int got_subtitle;
    double pts;




    // daipozhi for sdl1
    int i;




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




            // daipozhi for sdl1
            if (!(sp->subrects = av_mallocz_array(sp->sub.num_rects, sizeof(AVSubtitleRect*)))) {
                av_log(NULL, AV_LOG_FATAL, "Cannot allocate subrects\n");
                exit(1);
            }

            for (i = 0; i < sp->sub.num_rects; i++)
            {
                int in_w = sp->sub.rects[i]->w;
                int in_h = sp->sub.rects[i]->h;
                int subw = is->subdec.avctx->width  ? is->subdec.avctx->width  : is->viddec_width;
                int subh = is->subdec.avctx->height ? is->subdec.avctx->height : is->viddec_height;
                int out_w = is->viddec_width  ? in_w * is->viddec_width  / subw : in_w;
                int out_h = is->viddec_height ? in_h * is->viddec_height / subh : in_h;

                if (!(sp->subrects[i] = av_mallocz(sizeof(AVSubtitleRect))) ||
                    av_image_alloc(sp->subrects[i]->data, sp->subrects[i]->linesize, out_w, out_h, AV_PIX_FMT_YUVA420P, 16) < 0) {
                    av_log(NULL, AV_LOG_FATAL, "Cannot allocate subtitle data\n");
                    exit(1);
                }

                is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx,
                    in_w, in_h, AV_PIX_FMT_PAL8, out_w, out_h,
                    AV_PIX_FMT_YUVA420P, sws_flags, NULL, NULL, NULL);
                if (!is->sub_convert_ctx) {
                    av_log(NULL, AV_LOG_FATAL, "Cannot initialize the sub conversion context\n");
                    exit(1);
                }
                sws_scale(is->sub_convert_ctx,
                          (void*)sp->sub.rects[i]->data, sp->sub.rects[i]->linesize,
                          0, in_h, sp->subrects[i]->data, sp->subrects[i]->linesize);

                sp->subrects[i]->w = out_w;
                sp->subrects[i]->h = out_h;
                sp->subrects[i]->x = sp->sub.rects[i]->x * out_w / in_w;
                sp->subrects[i]->y = sp->sub.rects[i]->y * out_h / in_h;
            }




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




// daipozhi for sound river
#if DPZ_DEBUG1
static char m711_str1[300];
#endif




/* copy samples for viewing in editor window */
static void update_sample_display(VideoState *is, short *samples, int samples_size)
{
    int size, len;




    // daipozhi modified for sound river
    int  l,m;
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
        len = /*SAMPLE_ARRAY_SIZE*/deb_sr_sample_size - is->sample_array_index;




        if (len > size)
            len = size;
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        samples += len;
        is->sample_array_index += len;




	// daipozhi for sound river
        if (is->sample_array_index >= /*SAMPLE_ARRAY_SIZE*/deb_sr_sample_size)
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
    int64_t dec_channel_layout;
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

    data_size = av_samples_get_buffer_size(NULL, af->frame->channels,
                                           af->frame->nb_samples,
                                           af->frame->format, 1);

    dec_channel_layout =
        (af->frame->channel_layout && af->frame->channels == av_get_channel_layout_nb_channels(af->frame->channel_layout)) ?
        af->frame->channel_layout : av_get_default_channel_layout(af->frame->channels);
    wanted_nb_samples = synchronize_audio(is, af->frame->nb_samples);

    if (af->frame->format        != is->audio_src.fmt            ||
        dec_channel_layout       != is->audio_src.channel_layout ||
        af->frame->sample_rate   != is->audio_src.freq           ||
        (wanted_nb_samples       != af->frame->nb_samples && !is->swr_ctx)) {
        swr_free(&is->swr_ctx);
        is->swr_ctx = swr_alloc_set_opts(NULL,
                                         is->audio_tgt.channel_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
                                         dec_channel_layout,           af->frame->format, af->frame->sample_rate,
                                         0, NULL);
        if (!is->swr_ctx || swr_init(is->swr_ctx) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                    af->frame->sample_rate, av_get_sample_fmt_name(af->frame->format), af->frame->channels,
                    is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.channels);
            swr_free(&is->swr_ctx);
            return -1;
        }
        is->audio_src.channel_layout = dec_channel_layout;
        is->audio_src.channels       = af->frame->channels;
        is->audio_src.freq = af->frame->sample_rate;
        is->audio_src.fmt = af->frame->format;
    }

    if (is->swr_ctx) {
        const uint8_t **in = (const uint8_t **)af->frame->extended_data;
        uint8_t **out = &is->audio_buf1;
        int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate + 256;
        int out_size  = av_samples_get_buffer_size(NULL, is->audio_tgt.channels, out_count, is->audio_tgt.fmt, 0);
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
        resampled_data_size = len2 * is->audio_tgt.channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
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
    int i,j,k,l,m,n;
    int repe,over,start;
    int size,size1,ind;
    short int *nstr;

    len4=len;
    size=deb_resam_buff2_ptr;
    ind =deb_resam_buff2_ind;




    audio_callback_time = av_gettime_relative();

    while (len > 0) {




        // daipozhi modified
        //if (is->audio_buf_index >= is->audio_buf_size) {
        if (ind >= size) {




           audio_size = audio_decode_frame(is);
           if (audio_size < 0) {
                /* if error, just output silence */
               is->audio_buf = NULL;
               is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE / is->audio_tgt.frame_size * is->audio_tgt.frame_size;




               //daipozhi for sound river
               size=is->audio_buf_size;
               for (i=0;i<size/2;i++) deb_resam_buff2[i]=0;




           } else {




               //daipozhi for sound river  
               // need resample,so sound river is fix speed,frequency's position is fixed
               j = audio_size / sizeof(short);
               for (i=0;i<deb_resam_buff3_ptr;i++) deb_resam_buff1[i]=deb_resam_buff3[i];  // left samples in before

               nstr=is->audio_buf;
               for (i=0;i<j;i++) deb_resam_buff1[i+deb_resam_buff3_ptr]=nstr[i];  // copy sample
               j=j+deb_resam_buff3_ptr;  // total sample

               if (deb_resam_dir==1) // both are 44.1khz , don't need convert
               {
                 for (i=0;i<j;i++) deb_resam_buff2[i]=deb_resam_buff1[i];
                 size=j*2;
                 deb_resam_buff3_ptr=0;
               }
               else if (deb_sr_rate==48000)
               {
                 size=0;

                 for (n=0;n<deb_sr_ch;n++)
                 {
                   i=0;
                   repe=0;
                   l=deb_resam_last_ptr;
                   m=deb_resam_f[l];
                   start=m;

                   while (1)
                   {
                     if (deb_resam_last_ptr+i-repe*147>=147)
                     {
                       repe++;
                     }
                     
                     l=deb_resam_last_ptr+i-repe*147;
                     m=deb_resam_f[l];

                     if (160*repe-start+m<j/deb_sr_ch)
                     {
                       deb_resam_buff2[i*deb_sr_ch+n]=deb_resam_buff1[(160*repe-start+m)*deb_sr_ch+n];
                       if (size<i) size=i;
                     }
                     else
                     {
                       if (n==deb_sr_ch-1)
                       {
                         deb_resam_last_ptr=deb_resam_last_ptr+size-((deb_resam_last_ptr+size)/147)*147;
                         break;
                       }
                       else break;
                     }

                     i++;
                   }
                 }

                 for (i=(size+1)*deb_sr_ch;i<j;i++) deb_resam_buff3[i-(size+1)*deb_sr_ch]=deb_resam_buff1[i];
                 deb_resam_buff3_ptr=j-(size+1)*deb_sr_ch;
                 size=(size+1)*deb_sr_ch*2;
               }
               else if (deb_sr_rate==32000)
               {
                 size=0;

                 for (n=0;n<deb_sr_ch;n++)
                 {
                   i=0;
                   repe=0;
                   l=deb_resam_last_ptr;
                   m=deb_resam_f[l];
                   start=m;

                   while (1)
                   {
                     if (deb_resam_last_ptr+i-repe*441>=441)
                     {
                       repe++;
                     }
                     
                     l=deb_resam_last_ptr+i-repe*441;
                     m=deb_resam_f[l];

                     if (320*repe-start+m<j/deb_sr_ch)
                     {
                       deb_resam_buff2[i*deb_sr_ch+n]=deb_resam_buff1[(320*repe-start+m)*deb_sr_ch+n];
                       if (size<i) size=i;
                     }
                     else
                     {
                       if (n==deb_sr_ch-1)
                       {
                         deb_resam_last_ptr=deb_resam_last_ptr+size-((deb_resam_last_ptr+size)/441)*441;
                         break;
                       }
                       else break;
                     }

                     i++;
                   }
                 }

                 for (i=(size+1)*deb_sr_ch;i<j;i++) deb_resam_buff3[i-(size+1)*deb_sr_ch]=deb_resam_buff1[i];
                 deb_resam_buff3_ptr=j-(size+1)*deb_sr_ch;
                 size=(size+1)*deb_sr_ch*2;
               }
               else size=j*2;




	       //daipozhi for sound river
               //if (is->show_mode != SHOW_MODE_VIDEO)
               //    update_sample_display(is, (int16_t *)is->audio_buf, audio_size);
               update_sample_display(is, deb_resam_buff2, size);




               is->audio_buf_size = audio_size;
           }
           is->audio_buf_index = 0;




           // daipozhi for sound river
           ind=0;




        }
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len)
            len1 = len;




        //daipozhi for sound river
        size1 = size - ind;
        if (size1>len)
            size1=len;




// daipozhi modified for sound river
#if DPZ_DEBUG2
	len2=0;    // playback high,low,middle freq sound ,to sure fft is ok
	len3=0;
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
                	SDL_MixAudio(stream2, (uint8_t *)(deb_sr_fft_deb[deb_sr_fft_deb_ptr3]) + deb_sr_fft_deb_ptr1+len3, 1, is->audio_volume);
       		}

		len2=len2+1;
		len3=len3+1;
		stream2=stream2+1;
	     }
	     else
	     {
		len2=len2+1;
		//len3=len3+1;
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
            memcpy(stream, (uint8_t *)deb_resam_buff2 + ind, size1);
        else {
            //memset(stream, 0, len1);
            //if (!is->muted && is->audio_buf)
            //    SDL_MixAudioFormat(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, AUDIO_S16SYS, len1, is->audio_volume);
            memset(stream, 0, size1);
            if (!is->muted && is->audio_buf)
                SDL_MixAudio(stream, (uint8_t *)deb_resam_buff2 + ind, size1, is->audio_volume);
        }
#endif




        // daipozhi for sound river
        //len -= len1;
        //stream += len1;
        is->audio_buf_index += len1;




        // daipozhi for sound river
        len=len-size1;
        stream=stream+size1;
        ind=ind+size1;

        deb_resam_buff2_ptr=size;
        deb_resam_buff2_ind=ind;




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




static int audio_open(void *opaque, int64_t wanted_channel_layout, int wanted_nb_channels, int wanted_sample_rate, struct AudioParams *audio_hw_params)
{
    SDL_AudioSpec wanted_spec, spec;
    const char *env;
    static const int next_nb_channels[] = {0, 0, 1, 6, 2, 6, 4, 6};
    static const int next_sample_rates[] = {0, 44100, 48000, 96000, 192000};
    int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1;




    //daipozhi modified for sound river
    int i,j;
    deb_sr_show=0;
    deb_sr_rate=0;
    deb_sr_ch  =0;
    deb_resam_rate=44100;
    deb_resam_dir =0;




    env = SDL_getenv("SDL_AUDIO_CHANNELS");
    if (env) {
        wanted_nb_channels = atoi(env);
        wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);
    }
    if (!wanted_channel_layout || wanted_nb_channels != av_get_channel_layout_nb_channels(wanted_channel_layout)) {
        wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);
        wanted_channel_layout &= ~AV_CH_LAYOUT_STEREO_DOWNMIX;
    }
    wanted_nb_channels = av_get_channel_layout_nb_channels(wanted_channel_layout);
    wanted_spec.channels = wanted_nb_channels;




    // daipozhi for sound river
    wanted_spec.freq = deb_resam_rate/*wanted_sample_rate*/;




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




    // daipozhi modified
    /*while (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE | SDL_AUDIO_ALLOW_CHANNELS_CHANGE))) {
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
        wanted_channel_layout = av_get_default_channel_layout(wanted_spec.channels);
    }*/

    while (SDL_OpenAudio(&wanted_spec, &spec) < 0) {
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
        wanted_channel_layout = av_get_default_channel_layout(wanted_spec.channels);
    }




    if (spec.format != AUDIO_S16SYS) {
        av_log(NULL, AV_LOG_ERROR,
               "SDL advised audio format %d is not supported!\n", spec.format);
        return -1;
    }
    if (spec.channels != wanted_spec.channels) {
        wanted_channel_layout = av_get_default_channel_layout(spec.channels);
        if (!wanted_channel_layout) {
            av_log(NULL, AV_LOG_ERROR,
                   "SDL advised channel count %d is not supported!\n", spec.channels);
            return -1;
        }
    }

    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = spec.freq;
    audio_hw_params->channel_layout = wanted_channel_layout;
    audio_hw_params->channels =  spec.channels;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
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
    if (spec.freq==44100)
    {
	deb_sr_show=1;
    }
    if (spec.freq==48000)
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
    if (spec.freq==48000)
    {
	deb_resam_dir=0; // need resample,so sound river is fix speed,frequency's position is fixed

        //44.1k:48k ==  147:160
        for (i=0;i<147;i++)
        {
          deb_resam_f[i]=0;
        }

        for (i=0;i<147;i++)
        {
          deb_resam_f[i]= (float)i*(float)160/(float)147 + 0.5;
        }

        deb_resam_buff1_ptr=0;
        deb_resam_buff2_ptr=0;
        deb_resam_buff3_ptr=0;
        deb_resam_buff2_ind=0;
        deb_resam_last_ptr=0;
    }
    if (spec.freq==32000)
    {
	deb_resam_dir=0; // need resample,so sound river is fix speed,frequency's position is fixed

        //44.1k:32k ==  441:320
        for (i=0;i<441;i++)
        {
          deb_resam_f[i]=0;
        }

        for (i=0;i<441;i++)
        {
          deb_resam_f[i]= (float)i*(float)320/(float)441 + 0.5;
        }

        deb_resam_buff1_ptr=0;
        deb_resam_buff2_ptr=0;
        deb_resam_buff3_ptr=0;
        deb_resam_buff2_ind=0;
        deb_resam_last_ptr=0;
    }




    return spec.size;
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    const AVCodec *codec;
    const char *forced_codec_name = NULL;
    AVDictionary *opts = NULL;
    AVDictionaryEntry *t = NULL;
    int sample_rate, nb_channels;
    int64_t channel_layout;
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

    opts = filter_codec_opts(codec_opts, avctx->codec_id, ic, ic->streams[stream_index], codec);
    if (!av_dict_get(opts, "threads", NULL, 0))
        av_dict_set(&opts, "threads", "auto", 0);
    if (stream_lowres)
        av_dict_set_int(&opts, "lowres", stream_lowres, 0);
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
#if CONFIG_AVFILTER
        {
            AVFilterContext *sink;

            is->audio_filter_src.freq           = avctx->sample_rate;
            is->audio_filter_src.channels       = avctx->channels;
            is->audio_filter_src.channel_layout = get_valid_channel_layout(avctx->channel_layout, avctx->channels);
            is->audio_filter_src.fmt            = avctx->sample_fmt;
            if ((ret = configure_audio_filters(is, afilters, 0)) < 0)
                goto fail;
            sink = is->out_audio_filter;
            sample_rate    = av_buffersink_get_sample_rate(sink);
            nb_channels    = av_buffersink_get_channels(sink);
            channel_layout = av_buffersink_get_channel_layout(sink);
        }
#else
        sample_rate    = avctx->sample_rate;
        nb_channels    = avctx->channels;
        channel_layout = avctx->channel_layout;
#endif

        /* prepare audio output */
        if ((ret = audio_open(is, channel_layout, nb_channels, sample_rate, &is->audio_tgt)) < 0)
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
        if ((is->ic->iformat->flags & (AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK)) && !is->ic->iformat->read_seek) {
            is->auddec.start_pts = is->audio_st->start_time;
            is->auddec.start_pts_tb = is->audio_st->time_base;
        }
        if ((ret = decoder_start(&is->auddec, audio_thread, "audio_decoder", is)) < 0)
            goto out;




        // daipozhi modified
        //SDL_PauseAudioDevice(audio_dev, 0);
        SDL_PauseAudio(0);




        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_stream = stream_index;
        is->video_st = ic->streams[stream_index];




	// daipozhi for sdl1
        is->viddec_width  = avctx->width;
        is->viddec_height = avctx->height;




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
    AVDictionaryEntry *t;
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

    av_format_inject_global_side_data(ic);

    if (find_stream_info) {
        AVDictionary **opts = setup_find_stream_info_opts(ic, codec_opts);
        int orig_nb_streams = ic->nb_streams;

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
        seek_by_bytes = !!(ic->iformat->flags & AVFMT_TS_DISCONT) && strcmp("ogg", ic->iformat->name);

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

    while(1)
    {
	if (deb_seek_stt!=1) break;
        else av_usleep((int64_t)(0.03 * 1000000.0));
    }




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
static VideoState *stream_open(const char *filename, AVInputFormat *iformat, int step)
{
    //VideoState *is;




	if (step==1)//daipozhi modified 
	{  




    stream_open_is = av_mallocz(sizeof(VideoState));
    if (!stream_open_is)
        return NULL;




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
    stream_open_is->audio_volume = startup_volume;
    stream_open_is->muted = 0;
    stream_open_is->av_sync_type = av_sync_type;




    // daipozhi for sdl1
    stream_open_is->read_tid     = SDL_CreateThread(read_thread, /*"read_thread",*/ stream_open_is);




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
                    st->codecpar->channels != 0)
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




    // daipozhi for sdl1
    //is_full_screen = !is_full_screen;
    //SDL_SetWindowFullscreen(window, is_full_screen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);




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




//daipozhi modified
char m301_tmp_str1[3000];




static void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {




    // daipozhi for sound river
    struct timeval  tv;
    struct timezone tz;




    double remaining_time = 0.0;
    SDL_PumpEvents();




    //daipozhi modified
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, /*SDL_FIRSTEVENT, SDL_LASTEVENT*/SDL_ALLEVENTS)) {




	// daipozhi modified
        //if (!cursor_hidden && av_gettime_relative() - cursor_last_shown > CURSOR_HIDE_DELAY) {
        //    SDL_ShowCursor(0);
        //    cursor_hidden = 1;
        //}




        // daipozhi modified for sound river
        if (remaining_time > 0.0)
            av_usleep((int64_t)(remaining_time * 1000000.0 * 0.2 )); 




        remaining_time = REFRESH_RATE;




                // daipozhi modified for sound river
		if ((deb_sr_show_start==1)&&(deb_sr_time_set==0)&&(deb_thr_a))
		{
			gettimeofday(&tv,&tz);
			deb_sr_time1=tv.tv_sec;
			deb_sr_time2=tv.tv_usec;
			deb_sr_time3=deb_sr_time1*1000000+deb_sr_time2;
			deb_sr_time_set=1;
		}

		//   daipozhi modified 
		if (deb_st_play==1)
		{
			if (deb_frame_num<200)
			{
				deb_frame_num++;
			}
			else
			{
if ((deb_sr_show==1)&&(deb_sr_show_start==1)&&(deb_sr_show_nodisp==0)) deb_sr_river_show(is);  // daipozhi for sound river

				if ((deb_thr_v)&&(deb_thr_a)&&(deb_thr_a2)&&(deb_thr_r))
				{
					if (is->video_st)
					{




        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh))
            video_refresh(is, &remaining_time);




					}
				}
				
				deb_seek_bar_cntr++;   //daipozhi modified
				if (deb_seek_bar_cntr>=200) 
				{
					
					deb_seek_bar_cntr=0;
					
					if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
					{
						if ((deb_thr_a)&&(deb_thr_a2)&&(deb_thr_r))
						{
								deb_disp_bar(is);   
						}
					}
					else
					{
						if (is->video_st && is->show_mode == SHOW_MODE_VIDEO)
						{

							if ((deb_thr_v)&&(deb_thr_a)&&(deb_thr_a2)&&(deb_thr_r))
							{
								deb_disp_bar(is);   

							}
						}
					}
				}
			}
		}




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




// daipozhi modified
static char m302_tmp_str1[3000];




/* handle an event sent by the GUI */
static void event_loop(VideoState *cur_stream)
{
    SDL_Event event;
    double incr, pos, frac;




    //daipozhi modified
    int  xx,yy,n1,n2,n3,n4,n5;  
    char sc1;               
    int  s_dir_opened;
    int  tmp_n1;
    int  tmp_n2;
    //VideoState *is;




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
            switch (event.key.keysym.sym) {
            case SDLK_f:
                //toggle_full_screen(cur_stream);
                //cur_stream->force_refresh = 1;
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
#if CONFIG_AVFILTER
                //if (cur_stream->show_mode == SHOW_MODE_VIDEO && cur_stream->vfilter_idx < nb_vfilters - 1) {
                //    if (++cur_stream->vfilter_idx >= nb_vfilters)
                //        cur_stream->vfilter_idx = 0;
                //} else {
                //    cur_stream->vfilter_idx = 0;
                //    toggle_audio_display(cur_stream);
                //}
#else
                //toggle_audio_display(cur_stream);
#endif
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
		    deb_seek_stt=1;




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




		    //daipozhi modified
		    deb_seek_stt=0;




                break;
            default:
                break;
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
/*
            if (exit_on_mousedown) {
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

	    xx=event.button.x;
	    yy=event.button.y;

	    if (yy<cur_stream->height-deb_ch_h*2-deb_ch_d)
	    {
	      if ((deb_cover==1)&&(deb_cover_close==0))
	      {
		deb_cover_close=1;
		deb_disp_dir(cur_stream);
		//deb_disp_bar(cur_stream);
		deb_disp_scrn(cur_stream);
		break;
	      }
	      else
	      {
		if ((deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		{
		  deb_sr_show_nodisp=1;
		  deb_sr_show_start=0;
		  deb_disp_dir(cur_stream);
		  //deb_disp_bar(cur_stream);
		  deb_disp_scrn(cur_stream);
		  break;
		}
	      }
	    }

	    if ((yy>=0)&&(yy<cur_stream->height-deb_ch_h*2-deb_ch_d))       // in play list
	    {
	      if (  (yy>=deb_ch_h*1)&&(yy<cur_stream->height-deb_ch_h*3-deb_ch_d)  &&
                    (xx>=deb_ch_w*2 )&&(xx<cur_stream->width-deb_ch_w*2 )  ) // in file name
	      {
		n1=yy/deb_ch_h;
		if (n1>=1)
		{
		  n2=n1-1;

	          if (deb_filenamebuff_n+n2>=deb_filenamecnt) // more than filenamecnt 
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

                  sc1=deb_getfirstchar(deb_filenamebuff[deb_filenamebuff_n+n2]);
                  if (sc1=='|') break; // comment line
                  if (sc1==' ') break; // empty line
		  if (sc1!='<')        // not dir not empty
		  {
		    deb_get_path(deb_filenamebuff_n+n2);

		    if (deb_st_play==1)
		    {
		      if (cur_stream->paused) toggle_pause(cur_stream);

		      stream_close(cur_stream);
                      uninit_opts();
                      deb_opts_stt=0;
		    }

		    if (cur_stream->paused) toggle_pause(cur_stream);
                    if (deb_opts_stt==1)
                    {
                      uninit_opts();
                      deb_opts_stt=0;
                    }

		    deb_cover=0;
		    deb_cover_close=0;
		    deb_frame_num=0;
		    deb_border=0;

                    init_opts();
                    deb_opts_stt=1;
		    deb_ini_is(cur_stream);
		    deb_video_open_again(cur_stream,0);

		    deb_eo_stream=0;

		    deb_disp_scrn(cur_stream);

		    deb_filenameplay=deb_filenamebuff_n+n2;

		    //deb_filenameext2(deb_dir_buffer,m302_tmp_str1);
		    //tmp_n2=deb_supported_formats(m302_tmp_str1);
		    //if (tmp_n2!=1) break;

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
		    stream_open(deb_dir_buffer, file_iformat,2);

		    deb_disp_dir(cur_stream);
		    //deb_disp_bar();
		    break;
		  }
		  else //dir
		  {
		    deb_get_path(deb_filenamebuff_n+n2);

                    s_dir_opened=deb_dir_opened(deb_filenamebuff_n+n2);

                    if (s_dir_opened==0)
                    {
		      chdir(deb_dir_buffer);
		      getcwd(deb_currentpath,FN_SIZE);

		      //printf(",%s,\n",deb_dir_buffer);
		      //printf(",%s,\n",deb_currentpath);

                      if (deb_cmp_dir(deb_dir_buffer,deb_currentpath)==0)
		      {
			deb_get_dir();
        	        deb_dir_add_after(deb_filenamebuff_n+n2);
			deb_disp_dir(cur_stream);
		      }
                    }
                    else
                    {
                      deb_dir_remove_after(deb_filenamebuff_n+n2);
		      deb_disp_dir(cur_stream);
                    }
		  }
		}
	      }
	      else // in border line
	      {
		if (xx<cur_stream->width-deb_ch_w*2) break;

		n1=yy/deb_ch_h;
		if (n1==0)           // up 1 row
		{
		  if (deb_filenamebuff_n>=1) 
		  {
		    deb_filenamebuff_n--;
		    deb_disp_dir(cur_stream);
		    break;
		  }
		}
		else 
		{
		  if (yy>=cur_stream->height-deb_ch_h*3-deb_ch_d) //down 1 row
		  {
		    n3=(cur_stream->height)/deb_ch_h-2-2;
		    if (deb_filenamebuff_n+n3<deb_filenamecnt)
		    {
		      deb_filenamebuff_n++;
		      deb_disp_dir(cur_stream);
		      break;
		    }
		  }
		  else
		  {
		    n4=cur_stream->height-deb_ch_h*2-deb_ch_h*2-deb_ch_d;
		    n5=yy-deb_ch_h*1;
		    if (n5<n4/2) // up page
		    {
		      n3=(cur_stream->height)/deb_ch_h-2-2;
		      if (deb_filenamebuff_n>=n3) deb_filenamebuff_n=deb_filenamebuff_n-n3;
		      else deb_filenamebuff_n=0;

		      deb_disp_dir(cur_stream);
		      break;
		    }
		    else  // down page
		    {
		      n3=(cur_stream->height)/deb_ch_h-2-2;
		      if (deb_filenamebuff_n+n3+n3<deb_filenamecnt) deb_filenamebuff_n=deb_filenamebuff_n+n3;
		      else
		      {
			deb_filenamebuff_n=deb_filenamecnt-n3;
			if (deb_filenamebuff_n<0) deb_filenamebuff_n=0;
		      }

		      deb_disp_dir(cur_stream);
		      break;
		    }
		  }
		}
	      }
	    }

	    if ((yy>=cur_stream->height-deb_ch_h*2-deb_ch_d)&&(yy<cur_stream->height-deb_ch_h*1-deb_ch_d)) // in scroll bar
	    {
	      if (deb_st_play==1)
	      {

                x = event.button.x;


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
                }
    

		deb_frame_num=0;
		//deb_disp_bar(cur_stream);

		break;

	      }
            }

	    // daipozhi modified 
	    if (yy>=cur_stream->height-deb_ch_h*1-deb_ch_d) // in pause bar
	    {
	      if ((xx>=((cur_stream->width/deb_ch_w+1)/2-4)*deb_ch_w)&&(xx<=((cur_stream->width/deb_ch_w+1)/2+4)*deb_ch_w))
	      {
		if (deb_st_play==1)
		{
		  toggle_pause(cur_stream);

		  //deb_frame_num=0;
		  deb_disp_bar(cur_stream);
		  break;
		}
	      }
 
	      if (xx>=((cur_stream->width/deb_ch_w+1)-12)*deb_ch_w)
	      {
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
		    break;
		  }
		  else
		  {
		    deb_cover_close=1;

		    deb_disp_dir(cur_stream);
		    deb_disp_bar(cur_stream);
		    deb_disp_scrn(cur_stream);
		    break;
		  }
		}
		else
		{
		  if ((deb_sr_show_start==1)&&(deb_sr_show_nodisp==0))
		  {
		    deb_sr_show_start=0;
		    deb_sr_show_nodisp=1;

		    deb_disp_dir(cur_stream);
		    deb_disp_bar(cur_stream);
		    deb_disp_scrn(cur_stream);
		    break;
		  }
		  else
		  {
		    if ((deb_cover==1)&&(deb_cover_close==1))
		    {
		      deb_cover_close=0;
		      deb_border=0;
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
			  break;
			}
		      }
		    }
		  }
		}
	      }
            }
/*
        case SDL_MOUSEMOTION:
            if (cursor_hidden) {
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
            break;




	//daipozhi for sdl1
	/*
        case SDL_WINDOWEVENT:
            switch (event.window.event) {
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                    screen_width  = cur_stream->width  = event.window.data1;
                    screen_height = cur_stream->height = event.window.data2;
                    if (cur_stream->vis_texture) {
                        SDL_DestroyTexture(cur_stream->vis_texture);
                        cur_stream->vis_texture = NULL;
                    }
                case SDL_WINDOWEVENT_EXPOSED:
                    cur_stream->force_refresh = 1;
            }
            break;*/




        case SDL_VIDEOEXPOSE:
            cur_stream->force_refresh = 1;
            break;

        case SDL_VIDEORESIZE:

	    //daipozhi modified 
	    if (event.resize.w<1224) event.resize.w=1224;
	    if (event.resize.h<725 ) event.resize.h=725;
		
	    tmp_n1 = deb_ch_w*2*(event.resize.w /(deb_ch_w*2)) ;
	    tmp_n2 = deb_ch_h*1*(event.resize.h /(deb_ch_h*1))+7 ;

	    event.resize.w=tmp_n1;
	    event.resize.h=tmp_n2;

            screen = SDL_SetVideoMode(FFMIN(16383, event.resize.w), event.resize.h, 0,
                                          SDL_HWSURFACE|(is_full_screen?SDL_FULLSCREEN:SDL_RESIZABLE)|SDL_ASYNCBLIT|SDL_HWACCEL);
            if (!screen) {
                av_log(NULL, AV_LOG_FATAL, "Failed to set video mode\n");
                do_exit(cur_stream);
            }
            screen_width  = cur_stream->width  = screen->w;
            screen_height = cur_stream->height = screen->h;
            cur_stream->force_refresh = 1;

	    //daipozhi modified
	    deb_ch_d= cur_stream->height - deb_ch_h *( cur_stream->height / deb_ch_h ) ;

	    deb_disp_dir(cur_stream);	//daipozhi modified
	    deb_disp_bar(cur_stream);
	    deb_disp_scrn(cur_stream);

	    if ((deb_cover==1)&&(deb_cover_close==0))
	    {
	      deb_border=0;
	    }

	    deb_sr_river_f_init=0; //daipozhi modified for sound river

            break;




        case SDL_QUIT:
#if DPZ_DEBUG1
	    deb_record_close();
#endif
            do_exit(cur_stream);
            break;

        case FF_QUIT_EVENT:

            //do_exit(cur_stream);
            if (deb_stream_err==1) break;

	    if (cur_stream->paused) toggle_pause(cur_stream);

	    stream_close(cur_stream);
            uninit_opts();
            deb_opts_stt=0;

	    deb_cover=0;
	    deb_cover_close=0;
	    deb_frame_num=0;
	    deb_border=0;

            init_opts();
            deb_opts_stt=1;
	    deb_ini_is(cur_stream);
	    deb_video_open_again(cur_stream,0);

	    deb_filenameplay++;

	    deb_disp_dir(cur_stream);

            if (deb_filenameplay>=deb_filenamecnt) // more than filenamecnt 
	    {
	      deb_filenameplay=deb_filenamecnt-1;
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

            sc1=deb_getfirstchar(deb_filenamebuff[deb_filenameplay]);  //daipozhi modified for audio
            if (sc1=='|') break; // comment line
            if (sc1==' ') break; // empty line
	    if (sc1!='<')        // not dir not empty
	    {
	      deb_get_path(deb_filenameplay);

	      deb_eo_stream=0;

	      deb_video_open_again(cur_stream,0);

	      deb_disp_scrn(cur_stream);

	      //deb_filenameext2(deb_dir_buffer,m301_tmp_str1);
	      //tmp_n2=deb_supported_formats(m301_tmp_str1);
	      //if (tmp_n2!=1) break;

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
	      stream_open(deb_dir_buffer, file_iformat,2);

	      deb_disp_dir(cur_stream);  
	      //deb_disp_bar();
	      break;
	    }

            break;




	// daipozhi modified
        case FF_ALLOC_EVENT:
            alloc_picture(event.user.data1);
            break;




        default:
            break;
        }
    }
}

static int opt_frame_size(void *optctx, const char *opt, const char *arg)
{
    av_log(NULL, AV_LOG_WARNING, "Option -s is deprecated, use -video_size.\n");
    return opt_default(NULL, "video_size", arg);
}

static int opt_width(void *optctx, const char *opt, const char *arg)
{
    screen_width = parse_number_or_die(opt, arg, OPT_INT64, 1, INT_MAX);
    return 0;
}

static int opt_height(void *optctx, const char *opt, const char *arg)
{
    screen_height = parse_number_or_die(opt, arg, OPT_INT64, 1, INT_MAX);
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

static int opt_frame_pix_fmt(void *optctx, const char *opt, const char *arg)
{
    av_log(NULL, AV_LOG_WARNING, "Option -pix_fmt is deprecated, use -pixel_format.\n");
    return opt_default(NULL, "pixel_format", arg);
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

static int opt_seek(void *optctx, const char *opt, const char *arg)
{
    start_time = parse_time_or_die(opt, arg, 1);
    return 0;
}

static int opt_duration(void *optctx, const char *opt, const char *arg)
{
    duration = parse_time_or_die(opt, arg, 1);
    return 0;
}

static int opt_show_mode(void *optctx, const char *opt, const char *arg)
{
    show_mode = !strcmp(arg, "video") ? SHOW_MODE_VIDEO :
                !strcmp(arg, "waves") ? SHOW_MODE_WAVES :
                !strcmp(arg, "rdft" ) ? SHOW_MODE_RDFT  :
                parse_number_or_die(opt, arg, OPT_INT, 0, SHOW_MODE_NB-1);
    return 0;
}

static void opt_input_file(void *optctx, const char *filename)
{
    if (input_filename) {
        av_log(NULL, AV_LOG_FATAL,
               "Argument '%s' provided as input filename, but '%s' was already specified.\n",
                filename, input_filename);
        exit(1);
    }
    if (!strcmp(filename, "-"))
        filename = "pipe:";
    input_filename = filename;
}

static int opt_codec(void *optctx, const char *opt, const char *arg)
{
   const char *spec = strchr(opt, ':');
   if (!spec) {
       av_log(NULL, AV_LOG_ERROR,
              "No media specifier was specified in '%s' in option '%s'\n",
               arg, opt);
       return AVERROR(EINVAL);
   }
   spec++;
   switch (spec[0]) {
   case 'a' :    audio_codec_name = arg; break;
   case 's' : subtitle_codec_name = arg; break;
   case 'v' :    video_codec_name = arg; break;
   default:
       av_log(NULL, AV_LOG_ERROR,
              "Invalid media specifier '%s' in option '%s'\n", spec, opt);
       return AVERROR(EINVAL);
   }
   return 0;
}

static int dummy;

static const OptionDef options[] = {
    CMDUTILS_COMMON_OPTIONS
    { "x", HAS_ARG, { .func_arg = opt_width }, "force displayed width", "width" },
    { "y", HAS_ARG, { .func_arg = opt_height }, "force displayed height", "height" },
    { "s", HAS_ARG | OPT_VIDEO, { .func_arg = opt_frame_size }, "set frame size (WxH or abbreviation)", "size" },
    { "fs", OPT_BOOL, { &is_full_screen }, "force full screen" },
    { "an", OPT_BOOL, { &audio_disable }, "disable audio" },
    { "vn", OPT_BOOL, { &video_disable }, "disable video" },
    { "sn", OPT_BOOL, { &subtitle_disable }, "disable subtitling" },
    { "ast", OPT_STRING | HAS_ARG | OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_AUDIO] }, "select desired audio stream", "stream_specifier" },
    { "vst", OPT_STRING | HAS_ARG | OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_VIDEO] }, "select desired video stream", "stream_specifier" },
    { "sst", OPT_STRING | HAS_ARG | OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_SUBTITLE] }, "select desired subtitle stream", "stream_specifier" },
    { "ss", HAS_ARG, { .func_arg = opt_seek }, "seek to a given position in seconds", "pos" },
    { "t", HAS_ARG, { .func_arg = opt_duration }, "play  \"duration\" seconds of audio/video", "duration" },
    { "bytes", OPT_INT | HAS_ARG, { &seek_by_bytes }, "seek by bytes 0=off 1=on -1=auto", "val" },
    { "seek_interval", OPT_FLOAT | HAS_ARG, { &seek_interval }, "set seek interval for left/right keys, in seconds", "seconds" },
    { "nodisp", OPT_BOOL, { &display_disable }, "disable graphical display" },
    { "noborder", OPT_BOOL, { &borderless }, "borderless window" },
    { "alwaysontop", OPT_BOOL, { &alwaysontop }, "window always on top" },
    { "volume", OPT_INT | HAS_ARG, { &startup_volume}, "set startup volume 0=min 100=max", "volume" },
    { "f", HAS_ARG, { .func_arg = opt_format }, "force format", "fmt" },
    { "pix_fmt", HAS_ARG | OPT_EXPERT | OPT_VIDEO, { .func_arg = opt_frame_pix_fmt }, "set pixel format", "format" },
    { "stats", OPT_BOOL | OPT_EXPERT, { &show_status }, "show status", "" },
    { "fast", OPT_BOOL | OPT_EXPERT, { &fast }, "non spec compliant optimizations", "" },
    { "genpts", OPT_BOOL | OPT_EXPERT, { &genpts }, "generate pts", "" },
    { "drp", OPT_INT | HAS_ARG | OPT_EXPERT, { &decoder_reorder_pts }, "let decoder reorder pts 0=off 1=on -1=auto", ""},
    { "lowres", OPT_INT | HAS_ARG | OPT_EXPERT, { &lowres }, "", "" },
    { "sync", HAS_ARG | OPT_EXPERT, { .func_arg = opt_sync }, "set audio-video sync. type (type=audio/video/ext)", "type" },
    { "autoexit", OPT_BOOL | OPT_EXPERT, { &autoexit }, "exit at the end", "" },
    { "exitonkeydown", OPT_BOOL | OPT_EXPERT, { &exit_on_keydown }, "exit on key down", "" },
    { "exitonmousedown", OPT_BOOL | OPT_EXPERT, { &exit_on_mousedown }, "exit on mouse down", "" },
    { "loop", OPT_INT | HAS_ARG | OPT_EXPERT, { &loop }, "set number of times the playback shall be looped", "loop count" },
    { "framedrop", OPT_BOOL | OPT_EXPERT, { &framedrop }, "drop frames when cpu is too slow", "" },
    { "infbuf", OPT_BOOL | OPT_EXPERT, { &infinite_buffer }, "don't limit the input buffer size (useful with realtime streams)", "" },
    { "window_title", OPT_STRING | HAS_ARG, { &window_title }, "set window title", "window title" },
    { "left", OPT_INT | HAS_ARG | OPT_EXPERT, { &screen_left }, "set the x position for the left of the window", "x pos" },
    { "top", OPT_INT | HAS_ARG | OPT_EXPERT, { &screen_top }, "set the y position for the top of the window", "y pos" },
#if CONFIG_AVFILTER
    { "vf", OPT_EXPERT | HAS_ARG, { .func_arg = opt_add_vfilter }, "set video filters", "filter_graph" },
    { "af", OPT_STRING | HAS_ARG, { &afilters }, "set audio filters", "filter_graph" },
#endif
    { "rdftspeed", OPT_INT | HAS_ARG| OPT_AUDIO | OPT_EXPERT, { &rdftspeed }, "rdft speed", "msecs" },
    { "showmode", HAS_ARG, { .func_arg = opt_show_mode}, "select show mode (0 = video, 1 = waves, 2 = RDFT)", "mode" },
    { "default", HAS_ARG | OPT_AUDIO | OPT_VIDEO | OPT_EXPERT, { .func_arg = opt_default }, "generic catch all option", "" },
    { "i", OPT_BOOL, { &dummy}, "read specified file", "input_file"},
    { "codec", HAS_ARG, { .func_arg = opt_codec}, "force decoder", "decoder_name" },
    { "acodec", HAS_ARG | OPT_STRING | OPT_EXPERT, {    &audio_codec_name }, "force audio decoder",    "decoder_name" },
    { "scodec", HAS_ARG | OPT_STRING | OPT_EXPERT, { &subtitle_codec_name }, "force subtitle decoder", "decoder_name" },
    { "vcodec", HAS_ARG | OPT_STRING | OPT_EXPERT, {    &video_codec_name }, "force video decoder",    "decoder_name" },
    { "autorotate", OPT_BOOL, { &autorotate }, "automatically rotate video", "" },
    { "find_stream_info", OPT_BOOL | OPT_INPUT | OPT_EXPERT, { &find_stream_info },
        "read and decode the streams to fill missing information with heuristics" },
    { "filter_threads", HAS_ARG | OPT_INT | OPT_EXPERT, { &filter_nbthreads }, "number of filter threads per graph" },
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
    show_help_options(options, "Main options:", 0, OPT_EXPERT, 0);
    show_help_options(options, "Advanced options:", OPT_EXPERT, 0, 0);
    printf("\n");
    show_help_children(avcodec_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avformat_get_class(), AV_OPT_FLAG_DECODING_PARAM);
#if !CONFIG_AVFILTER
    show_help_children(sws_get_class(), AV_OPT_FLAG_ENCODING_PARAM);
#else
    show_help_children(avfilter_get_class(), AV_OPT_FLAG_FILTERING_PARAM);
#endif
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
    int flags;
    VideoState *is;




    //daipozhi for sdl1
    char dummy_videodriver[] = "SDL_VIDEODRIVER=dummy";
    char alsa_bufsize[] = "SDL_AUDIO_ALSA_SET_BUFFER_SIZE=1";




    // daipozhi modified
    int    arc;
    char   arv3[30][30];
    char  *arv2[10];
    char **arv;

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




    init_opts();

    signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).    */
    signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */




    //daipozhi modified
    show_banner(arc, arv, options);




    //daipozhi modified
    parse_options(NULL, arc, arv, options, opt_input_file);




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




    //daipozhi for sdl1
    /*
    flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;
    if (audio_disable)
        flags &= ~SDL_INIT_AUDIO;
    else {
        // * Try to work around an occasional ALSA buffer underflow issue when the
        // * period size is NPOT due to ALSA resampling by forcing the buffer size. 
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
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);*/




    flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;
    if (audio_disable)
        flags &= ~SDL_INIT_AUDIO;
    else {
        /* Try to work around an occasional ALSA buffer underflow issue when the
         * period size is NPOT due to ALSA resampling by forcing the buffer size. */
        if (!SDL_getenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE"))
            SDL_putenv(alsa_bufsize);
    }
    if (display_disable)
        SDL_putenv(dummy_videodriver); /* For the event queue, we always need a video driver. */
#if !defined(_WIN32) && !defined(__APPLE__)
    flags |= SDL_INIT_EVENTTHREAD; /* Not supported on Windows or Mac OS X */
#endif
    if (SDL_Init (flags)) {
        av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n", SDL_GetError());
        av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
        exit(1);
    }

    if (!display_disable) {
        const SDL_VideoInfo *vi = SDL_GetVideoInfo();
        fs_screen_width = vi->current_w;
        fs_screen_height = vi->current_h;
    }

    SDL_EventState(SDL_ACTIVEEVENT, SDL_IGNORE);
    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);




    // daipozhi for sdl1
    /*
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
        window = SDL_CreateWindow(program_name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (window) {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (!renderer) {
                av_log(NULL, AV_LOG_WARNING, "Failed to initialize a hardware accelerated renderer: %s\n", SDL_GetError());
                renderer = SDL_CreateRenderer(window, -1, 0);
            }
            if (renderer) {
                if (!SDL_GetRendererInfo(renderer, &renderer_info))
                    av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n", renderer_info.name);
            }
        }
        if (!window || !renderer || !renderer_info.num_texture_formats) {
            av_log(NULL, AV_LOG_FATAL, "Failed to create window or renderer: %s", SDL_GetError());
            do_exit(NULL);
        }
    }*/




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

    video_open(is, 0, NULL);  //daipozhi modified 

    //chdir("/");  //daipozhi modified 
		   //*****notice
    deb_get_dir_ini();  //daipozhi modified  
    deb_disp_dir(is);
    deb_disp_bar(is);
    deb_disp_scrn(is);

    // daipozhi for sound river
    deb_sr_fft_set_db(deb_sr_rate);
    deb_sr_river_f_init=0;




    event_loop(is);

    /* never returns */

    return 0;
}








static char m301_str[300];

// daipozhi modified
static int deb_load_font(void)
{
  int  n1,n2;    
  int  l1,l2,l3,l4;
  char c1,c2,c3;

  // daipozhi modified
  deb_fh=open("./ascii_bmp/ascii_bmp.data",O_RDONLY,S_IREAD);
  if (deb_fh>=0)
  {
    deb_str=(char *)deb_ascii_bmp;
    read(deb_fh,deb_str,128*13*6*3);
    close(deb_fh);
  }
  else
  {
    printf("Open file './ascii_bmp/ascii_bmp.data' error\n");
  }

  // daipozhi modified
  n1=0;
  n2=129;

  while (n1<=120)
  {
    snprintf(m301_str,300,"./ascii_bmp/chn_bmp%3d.data",n2);

    // daipozhi modified
    deb_fh=open(m301_str,O_RDONLY,S_IREAD);
    if (deb_fh>=0)
    {
      deb_str=(char *)deb_chns_bmp2;
      read(deb_fh,deb_str,6*128*13*12*3);
      close(deb_fh);
    }
    else
    {
      printf("Open file '%s' error\n",m301_str);
    }

    for (l1=0;l1<6;l1++)
      for (l2=0;l2<128;l2++)
        for (l3=0;l3<13;l3++)
          for (l4=0;l4<12;l4++)
          {
            c1=deb_chns_bmp2[l1][l2][l3][l4][0];
            c2=deb_chns_bmp2[l1][l2][l3][l4][1];
            c3=deb_chns_bmp2[l1][l2][l3][l4][2];

            if ((n1+l1<0)||(n1+l1>=128)) continue;    //*****notice

            deb_chns_bmp[n1+l1][l2][l3][l4][0]=c1;
            deb_chns_bmp[n1+l1][l2][l3][l4][1]=c2;
            deb_chns_bmp[n1+l1][l2][l3][l4][2]=c3;
          }

    n1=n1+6;
    n2=n2+6;
  }


  // daipozhi modified
  deb_fh=open("./ascii_bmp/tableline.txt",O_RDONLY,S_IREAD);
  if (deb_fh>=0)
  {
    read(deb_fh,deb_tableline,30);
    close(deb_fh);
  }
  else
  {
    printf("Open file './ascii_bmp/tableline.txt' error\n");
  }

  return(0);
}

static int deb_echo_char4seekbar(int x,int y,int ec)
{
  int l1/*,l2*/;
  //int i1,i2,i3;
  int bgcolor;

  if (ec=='-')
  {
    bgcolor = SDL_MapRGB(screen->format, 0, 0, 0);

    fill_rectangle(screen,
                   x, 
                   y+7,
                   1, 
                   1,
                   bgcolor,0);
  }

  if (ec=='+')
  {
    bgcolor = SDL_MapRGB(screen->format, 0, 0, 0);

    for (l1=0;l1<13;l1++)
    {
      fill_rectangle(screen,
                     x, 
                     y+l1,
                     1, 
                     1,
                     bgcolor,0);
    }
  }

  return(0);
}

static int deb_echo_str4seekbar(int yy,char *str)
{
  int i,j,k;
  int x,y;

  x=0;
  y=yy;
  i=(int)strlen(str);
  if (i<=0) return(0);

  for (j=0;j<i;j++)
  {
    k=str[j];
    if ((k<32)||(k>=128)) continue;
    deb_echo_char4seekbar(x,y,k);
    x=x+1;
  }

  return(0);
}

static int deb_echo_char4en(int x,int y,int ec)
{
  int l1,l2;
  int i1,i2,i3;
  int bgcolor;
  int updown=0;

  if ((ec<0)||(ec>=128)) return(0);

  if (ec=='^')
  {
    ec='V';
    updown=1;
  }

  for (l1=0;l1<13;l1++)
  {
    for (l2=0;l2<6;l2++)
    {
      i1=deb_ascii_bmp[ec][l1][l2][0];
      i2=deb_ascii_bmp[ec][l1][l2][1];
      i3=deb_ascii_bmp[ec][l1][l2][2];

      bgcolor = SDL_MapRGB(screen->format, i1, i2, i3);

      if (updown==0)
      {
        fill_rectangle(screen,
                       x+l2, y+13-l1,
                       1, 1,
                       bgcolor,0);
      }
      else
      {
        fill_rectangle(screen,
                       x+l2, y+1 +l1,
                       1, 1,
                       bgcolor,0);
      }
    }
  }

  return(0);
}

static int deb_echo_char4en_black(int x,int y,int ec)
{
  int l1,l2;
  int i1,i2,i3;
  int bgcolor;
  int updown=0;
  unsigned char uc1,uc2,uc3;

  if ((ec<0)||(ec>=128)) return(0);

  if (ec=='^')
  {
    ec='V';
    updown=1;
  }

  for (l1=0;l1<13;l1++)
  {
    for (l2=0;l2<6;l2++)
    {
      uc1=deb_ascii_bmp[ec][l1][l2][0];
      uc2=deb_ascii_bmp[ec][l1][l2][1];
      uc3=deb_ascii_bmp[ec][l1][l2][2];

      i1=255-uc1;
      i2=255-uc2;
      i3=255-uc3;

      bgcolor = SDL_MapRGB(screen->format, i1, i2, i3);

      if (updown==0)
      {
        fill_rectangle(screen,
                       x+l2, y+13-l1,
                       1, 1,
                       bgcolor,0);
      }
      else
      {
        fill_rectangle(screen,
                       x+l2, y+1 +l1,
                       1, 1,
                       bgcolor,0);
      }
    }
  }

  return(0);
}

static int deb_echo_char4en_green(int x,int y,int ec)
{
  int l1,l2;
  int i1,i2,i3;
  int bgcolor;
  int updown=0;

  if ((ec<0)||(ec>=128)) return(0);

  if (ec=='^')
  {
    ec='V';
    updown=1;
  }

  for (l1=0;l1<13;l1++)
  {
    for (l2=0;l2<6;l2++)
    {
      i1=deb_ascii_bmp[ec][l1][l2][0];
      i2=deb_ascii_bmp[ec][l1][l2][1];
      i3=deb_ascii_bmp[ec][l1][l2][2];

      bgcolor = SDL_MapRGB(screen->format, 0, 255-i2, 0);

      if (updown==0)
      {
        fill_rectangle(screen,
                       x+l2, y+13-l1,
                       1, 1,
                       bgcolor,0);
      }
      else
      {
        fill_rectangle(screen,
                       x+l2, y+1 +l1,
                       1, 1,
                       bgcolor,0);
      }
    }
  }

  return(0);
}

static int deb_echo_char4chns(int x,int y,int uc1,int uc2)
{
  int l1,l2;
  int i1,i2,i3;
  int bgcolor;
  int k,l;

  k=uc1-129;
  l=uc2-128;

  if ((k<0)||(k>=126)) return(0);
  if ((l<0)||(l>127)) return(0);

  for (l1=0;l1<13;l1++)
  {
    for (l2=0;l2<12;l2++)
    {
      i1=deb_chns_bmp[k][l][l1][l2][0];
      i2=deb_chns_bmp[k][l][l1][l2][1];
      i3=deb_chns_bmp[k][l][l1][l2][2];

      bgcolor = SDL_MapRGB(screen->format, i1, i2, i3);

      fill_rectangle(screen,
                     x+l2, y+13-l1,
                     1,1,
                     bgcolor,0);
    }
  }

  return(0);
}

static int deb_echo_char4chns_black(int x,int y,int uc1,int uc2)
{
  int l1,l2;
  int i1,i2,i3;
  int bgcolor;
  unsigned char ui1,ui2,ui3;
  int k,l;

  k=uc1-129;
  l=uc2-128;

  if ((k<0)||(k>=126)) return(0);
  if ((l<0)||(l>127)) return(0);

  for (l1=0;l1<13;l1++)
  {
    for (l2=0;l2<12;l2++)
    {
      ui1=deb_chns_bmp[k][l][l1][l2][0];
      ui2=deb_chns_bmp[k][l][l1][l2][1];
      ui3=deb_chns_bmp[k][l][l1][l2][2];

      i1=255-ui1;
      i2=255-ui2;
      i3=255-ui3;

      bgcolor = SDL_MapRGB(screen->format, i1, i2, i3);

      fill_rectangle(screen,
                     x+l2, y+13-l1,
                     1, 1,
                     bgcolor,0);
    }
  }

  return(0);
}

static int deb_echo_char4chns_green(int x,int y,int uc1,int uc2)
{
  int l1,l2;
  int i1,i2,i3;
  int bgcolor;
  int k,l;

  k=uc1-129;
  l=uc2-128;

  if ((k<0)||(k>=126)) return(0);
  if ((l<0)||(l>127)) return(0);

  for (l1=0;l1<13;l1++)
  {
    for (l2=0;l2<12;l2++)
    {
      i1=deb_chns_bmp[k][l][l1][l2][0];
      i2=deb_chns_bmp[k][l][l1][l2][1];
      i3=deb_chns_bmp[k][l][l1][l2][2];

      bgcolor = SDL_MapRGB(screen->format, 0, 255-i2, 0);

      fill_rectangle(screen,
                     x+l2, y+13-l1,
                     1, 1,
                     bgcolor,0);
    }
  }

  return(0);
}

static int deb_echo_str4screenstring(int xx,int yy,char *str,int len)
{
  int				i,j,k,l;
  int				x,y;
  unsigned char			uc1,uc2;
  char				c1;

  x=xx;
  y=yy;
  i=len;
  j=0;

  while(j<i)
  {
    c1=str[j];
    if (c1>=0)
    {
      deb_echo_char4en(x,y,c1);

      x=x+6;
      j++;
      continue;
    }
    else 
    {
      uc1=str[j+0];
      uc2=str[j+1];

      deb_echo_char4chns(x,y,uc1,uc2);

      x=x+12;
      j=j+2;
      continue;
    }
  }

  return(0);
}

static int deb_echo_str4screenstring_black(int xx,int yy,char *str,int len)
{
  int				i,j,k,l;
  int				x,y;
  unsigned char			uc1,uc2;
  char				c1;

  x=xx;
  y=yy;
  i=len;
  j=0;

  while(j<i)
  {
    c1=str[j];

    if (c1>=0)
    {
      deb_echo_char4en_black(x,y,c1);

      x=x+6;
      j++;
      continue;
    }
    else 
    {
      uc1=str[j+0];
      uc2=str[j+1];

      deb_echo_char4chns_black(x,y,uc1,uc2);

      x=x+12;
      j=j+2;
      continue;
    }
  }

  return(0);
}

static int deb_echo_str4screenstring_green(int xx,int yy,char *str,int len)
{
  int				i,j,k,l;
  int				x,y;
  unsigned char			uc1,uc2;
  char				c1;

  x=xx;
  y=yy;
  i=len;
  j=0;

  while(j<i)
  {
    c1=str[j];

    if (c1>=0)
    {
      deb_echo_char4en_green(x,y,c1);

      x=x+6;
      j++;
      continue;
    }
    else 
    {
      uc1=str[j+0];
      uc2=str[j+1];

      deb_echo_char4chns_green(x,y,uc1,uc2);

      x=x+12;
      j=j+2;
      continue;
    }
  }

  return(0);
}

// daipozhi modified 
static int deb_get_dir_ini(void)
{
  int           i;
  //char          buffer2[1000];

  for (i=0;i<MAX_FILE_NUM;i++) deb_filenamebuff[i][0]=0;

  getcwd(deb_currentpath,FN_SIZE);
  deb_filenamebuff_n=0;
  deb_filenameplay=0;
  deb_filenamecnt=0;

  strcpy(deb_filenamebuff[deb_filenamecnt],"</>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<C:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<D:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<E:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<F:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<G:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<H:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<I:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<J:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<K:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  strcpy(deb_filenamebuff[deb_filenamecnt],"<L:/>");
  deb_filenamebuffptr[deb_filenamecnt]=2;
  deb_filenamecnt++;
  if (deb_filenamecnt>=MAX_FILE_NUM) return(0);

  return(0);
}



// daipozhi modified 
/*
static int deb_get_dir(void)
{
  //int           i,j,k,l;
  //bt_opendir();
  //return(0);
}
*/

static int deb_dir_opened(int ptr )
{
  int i,j/*,k*/;

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamecnt) return(0);

  i=deb_get_space(deb_filenamebuff[ptr]);

  if (ptr+1>=deb_filenamecnt) return(0);
  else
  {
    j=deb_get_space(deb_filenamebuff[ptr+1]);
    if (j>i) return(1);
    else return(0);
  }
}

static int deb_get_space(char *buffer)
{
  int i,j,n;

  n=0;
  j=(int)strlen(buffer);
  if (j<=0) return(0);

  for (i=0;i<j;i++)
  {
    if (i>=FN_SIZE-1) break;

    if (buffer[i]==' ') n++;
    else break;
  }

  return(n);
}

static char deb_getfirstchar(char *buffer)
{
  int i,j,k,n;

  j=0;
  n=0;
  k=(int)strlen(buffer);
  if (k<=0) return(' ');

  for (i=0;i<k;i++)
  {
    if (i>=FN_SIZE-1) break;

    if (buffer[i]==' ') continue;
    else
    {
      j=1;
      n=buffer[i];
      break;
    }
  }

  if (j==0) return(' ');
  else return((char)n);
}

static char m101_s1[3000];
static char m101_s2[100];

static int deb_dir_add_after(int ptr)
{
  int i,j,k/*,l*/;
  //char s1[3000];
  //char s2[100];

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamecnt) return(0);

  if (t1_out_ptr>0)
  {
    m101_s1[0]=0;

    j=deb_get_space(deb_filenamebuff[ptr]);
    for (k=0;k<j;k++)
    {
      m101_s1[k+0]=' ';
      m101_s1[k+1]=0;
    }

    strcpy(m101_s2,"  ");

    for (i=deb_filenamecnt-1;i>ptr;i--)
    {
        if (i<0) continue;
        if (i>=MAX_FILE_NUM) continue;

        if (i+t1_out_ptr<0) continue;
        if (i+t1_out_ptr>=MAX_FILE_NUM) continue;

        if (deb_str_has_null(deb_filenamebuff[i],FN_SIZE)!=1) continue;

        if (strlen(deb_filenamebuff[i])>=FN_SIZE) continue;

      	strcpy(deb_filenamebuff[i+t1_out_ptr],deb_filenamebuff[i]);

      	strcpy(deb_filenamebuff_ext[ i+t1_out_ptr],deb_filenamebuff_ext[ i]);
      	strcpy(deb_filenamebuff_size[i+t1_out_ptr],deb_filenamebuff_size[i]);
      	strcpy(deb_filenamebuff_date[i+t1_out_ptr],deb_filenamebuff_date[i]);

	deb_filenamebuff_len[ i+t1_out_ptr]=deb_filenamebuff_len[ i];
	deb_filenamebuff_type[i+t1_out_ptr]=deb_filenamebuff_type[i];

      	deb_filenamebuff[i][0]=0;

	deb_filenamebuff_ext[i][0]=0;
	deb_filenamebuff_size[i][0]=0;
	deb_filenamebuff_date[i][0]=0;

	deb_filenamebuff_len[i]=0;
	deb_filenamebuff_type[i]=0;

    }

    for (i=0;i<=t1_out_ptr-1;i++)
    {
        if (i<0) continue;
        if (i>=BTREE1_SIZE) continue;

        if (ptr+1+i<0) continue;
        if (ptr+1+i>=MAX_FILE_NUM) continue;

        if (deb_str_has_null(t1_out_buff[i],FN_SIZE)!=1) continue;
        if (deb_str_has_null(m101_s1,3000)!=1) continue;
        if (deb_str_has_null(m101_s2,100)!=1) continue;

        if (strlen(t1_out_buff[i])+strlen(m101_s1)+strlen(m101_s2)>=FN_SIZE) continue;

      	strcpy(deb_filenamebuff[ptr+1+i],m101_s1);
      	strcat(deb_filenamebuff[ptr+1+i],m101_s2);
      	strcat(deb_filenamebuff[ptr+1+i],t1_out_buff[i]);

      	strcpy(deb_filenamebuff_ext[ ptr+1+i],t1_out_buff3[i]);
      	strcpy(deb_filenamebuff_size[ptr+1+i],t1_out_buff4[i]);
      	strcpy(deb_filenamebuff_date[ptr+1+i],t1_out_buff5[i]);

	deb_filenamebuff_len[ ptr+1+i]=0;
	deb_filenamebuff_type[ptr+1+i]=0;
    }
    
    deb_filenamecnt=deb_filenamecnt+t1_out_ptr;

    deb_filenamebuff_len[ ptr]=deb_m_info_len;
    deb_filenamebuff_type[ptr]=deb_m_info_type;

    if (ptr>=deb_filenameplay)
    {
    }
    else
    {
      deb_filenameplay=deb_filenameplay+t1_out_ptr;
    }
  }
  else
  {
    m101_s1[0]=0;

    j=deb_get_space(deb_filenamebuff[ptr]);
    for (k=0;k<j;k++)
    {
      m101_s1[k+0]=' ';
      m101_s1[k+1]=0;
    }

    strcpy(m101_s2,"  ");

    for (i=deb_filenamecnt-1;i>ptr;i--)
    {
        if (i<0) continue;
        if (i>=MAX_FILE_NUM) continue;

        if (i+1<0) continue;
        if (i+1>=MAX_FILE_NUM) continue;

        if (deb_str_has_null(deb_filenamebuff[i],FN_SIZE)!=1) continue;

        if (strlen(deb_filenamebuff[i])>=FN_SIZE) continue;

      	strcpy(deb_filenamebuff[i+1],deb_filenamebuff[i]);

      	strcpy(deb_filenamebuff_ext[ i+1],deb_filenamebuff_ext[ i]);
      	strcpy(deb_filenamebuff_size[i+1],deb_filenamebuff_size[i]);
      	strcpy(deb_filenamebuff_date[i+1],deb_filenamebuff_date[i]);

	deb_filenamebuff_len[ i+1]=deb_filenamebuff_len[ i];
	deb_filenamebuff_type[i+1]=deb_filenamebuff_type[i];

      	deb_filenamebuff[i][0]=0;

	deb_filenamebuff_ext[ i][0]=0;
	deb_filenamebuff_size[i][0]=0;
	deb_filenamebuff_date[i][0]=0;

	deb_filenamebuff_len[i]=0;
	deb_filenamebuff_type[i]=0;
    }

    strcpy(deb_filenamebuff[ptr+1],m101_s1);
    strcat(deb_filenamebuff[ptr+1],m101_s2);
    strcat(deb_filenamebuff[ptr+1],"|Empty Fold|");

    strcpy(deb_filenamebuff_ext[ptr+1],"    ");
    strcpy(deb_filenamebuff_size[ptr+1],"      ");
    strcpy(deb_filenamebuff_date[ptr+1],"                   ");
    
    deb_filenamecnt=deb_filenamecnt+1;

    deb_filenamebuff_len[ptr] =12;
    deb_filenamebuff_type[ptr]=0;

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
  if (ptr>=deb_filenamecnt) return(0);

  i=deb_get_space(deb_filenamebuff[ptr]);

  k=0;

  for (j=ptr+1;j<deb_filenamecnt;j++)
  {
    if (j<0) continue;
    if (j>=MAX_FILE_NUM) continue;

    l=deb_get_space(deb_filenamebuff[j]);
    if (l<=i)
    {
      k=1;
      p1=j;
      break;
    }
  }

  if (k==0)
  {

    for (j=ptr+1;j<deb_filenamecnt;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_type[j]=0;
    }

    deb_filenamecnt=ptr+1;

    deb_filenameplay=ptr;

  }
  else
  {
    for (j=p1;j<deb_filenamecnt;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      if (ptr+1+j-p1<0) continue;
      if (ptr+1+j-p1>=MAX_FILE_NUM) continue;

      if (deb_str_has_null(deb_filenamebuff[j],FN_SIZE)!=1) continue;

      if (strlen(deb_filenamebuff[j])>=FN_SIZE) continue;

      strcpy(deb_filenamebuff[ptr+1+j-p1],deb_filenamebuff[j]);

      strcpy(deb_filenamebuff_ext[ptr+1+j-p1],deb_filenamebuff_ext[j]);
      strcpy(deb_filenamebuff_size[ptr+1+j-p1],deb_filenamebuff_size[j]);
      strcpy(deb_filenamebuff_date[ptr+1+j-p1],deb_filenamebuff_date[j]);

      deb_filenamebuff_len[ptr+1+j-p1] =deb_filenamebuff_len[j];
      deb_filenamebuff_type[ptr+1+j-p1]=deb_filenamebuff_type[j];

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_type[j]=0;
    }

    for (j=ptr+deb_filenamecnt-p1+1;j<p1;j++)
    {
      if (j<0) continue;
      if (j>=MAX_FILE_NUM) continue;

      deb_filenamebuff[j][0]=0;

      deb_filenamebuff_ext[j][0]=0;
      deb_filenamebuff_size[j][0]=0;
      deb_filenamebuff_date[j][0]=0;

      deb_filenamebuff_len[j]=0;
      deb_filenamebuff_type[j]=0;
    }

    deb_filenamecnt=deb_filenamecnt-(p1-ptr-1);

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
  //char buffer1[3000];
  //char buffer2[3000];
  //char buffer3[3000];
  //char buffer4[3000];
  int  ns1,ns2;
  int  p1;

  if (ptr<0) return(0);
  if (ptr>=MAX_FILE_NUM) return(0);
  if (ptr>=deb_filenamecnt) return(0);

  p1=ptr;

  m5_buffer4[0]=0;

  deb_dir_buffer[0]=0;

  ns1=deb_get_space(deb_filenamebuff[ptr]);

  while (p1>=0)
  {
    if (ns1<2)
    {
      strcpy(m5_buffer1,deb_filenamebuff[p1]);

      deb_get_path2(m5_buffer1,m5_buffer2);
      //strcpy(deb_dir_buffer,"/");
      //return(0);
      strcpy(m5_buffer3,m5_buffer2);
    }
    else
    {
      if (ns1<4)
      {
        strcpy(m5_buffer1,deb_filenamebuff[p1]);
        deb_get_path2(m5_buffer1,m5_buffer2);
        strcpy(m5_buffer3,m5_buffer2);
      }
      else
      {
        strcpy(m5_buffer1,deb_filenamebuff[p1]);
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
      strcpy(m5_buffer1,deb_filenamebuff[p1]);
      ns2=deb_get_space(m5_buffer1);
	
      if (ns2>=ns1) continue;
      else
      {
        ns1=ns2;
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
  int  i,j,k;
  //char buffer3[3000];
  //char buffer4[3000];

  strcpy(m102_buffer3,buffer1);
  strcpy(m102_buffer4,"");

  for (i=0;i<(int)strlen(m102_buffer3);i++)
  {
    if (m102_buffer3[i]==' ') continue;
    else
    {
      if (m102_buffer3[i]=='<')
      {
        m102_buffer3[i]=' ';
        break;
      }
      else break;
    }
  }

  for (i=(int)strlen(m102_buffer3)-1;i>=0;i--)
  {
    if ((m102_buffer3[i]==' ')||(m102_buffer3[i]==0)) continue;
    else
    {
      if (m102_buffer3[i]=='>')
      {
        m102_buffer3[i]=0;
        break;
      }
      else break;
    }
  }

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

  strcpy(buffer2,"/");
  strcat(buffer2,m102_buffer4);

  return(0);
}

static char m103_buffer3[3000];
static char m103_buffer4[3000];

static int deb_get_path2(char *buffer1,char *buffer2)
{
  int  i,j,k;
  //char buffer3[3000];
  //char buffer4[3000];

  strcpy(m103_buffer3,buffer1);
  strcpy(m102_buffer4,"");

  for (i=0;i<(int)strlen(m103_buffer3);i++)
  {
    if (m103_buffer3[i]==' ') continue;
    else
    {
      if (m103_buffer3[i]=='<')
      {
        m103_buffer3[i]=' ';
        break;
      }
      else break;
    }
  }

  for (i=(int)strlen(m103_buffer3)-1;i>=0;i--)
  {
    if ((m103_buffer3[i]==' ')||(m103_buffer3[i]==0)) continue;
    else
    {
      if (m103_buffer3[i]=='>')
      {
        m103_buffer3[i]=0;
        break;
      }
      else break;
    }
  }

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

  strcpy(buffer2,m103_buffer4);

  return(0);
}


//static char m104_str1[3000];

static int deb_filenameext(char *name,char *fext)
{
  //char str1[3000];
  int  i,j,k,l;
  //struct stat info;

  i=(int)strlen(name);
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
  }
  else
  {
    fext[0]=0;

    for (l=k+1;l<i;l++)
    {
      fext[l-k-1]=name[l];
      fext[l-k-0]=0;
    }
  }
	
  return(0);
}

static int deb_filenameext2(char *path,char *fext)
{
  //char str1[3000];
  int  i,j,k,l;
  struct stat info;

  i=(int)strlen(path);
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
        fext[l-k-1]=path[l];
        fext[l-k-0]=0;
      }
    }
  }

  return(0);
}
/*
static int deb_supported_formats(char *p_str)
{
  deb_lower_string(p_str);

  if (strcmp(p_str,"aac")==0) return(1);
  if (strcmp(p_str,"ape")==0) return(1);
  if (strcmp(p_str,"asf")==0) return(1);
  if (strcmp(p_str,"avi")==0) return(1);
  if (strcmp(p_str,"flac")==0) return(1);
  if (strcmp(p_str,"m4a")==0) return(1);
  if (strcmp(p_str,"m4v")==0) return(1);
  if (strcmp(p_str,"mp4")==0) return(1);
  if (strcmp(p_str,"mpeg")==0) return(1);
  if (strcmp(p_str,"mkv")==0) return(1);
  if (strcmp(p_str,"mp2")==0) return(1);
  if (strcmp(p_str,"mp3")==0) return(1);
  if (strcmp(p_str,"mpc")==0) return(1);
  if (strcmp(p_str,"ogg")==0) return(1);
  if (strcmp(p_str,"rm")==0) return(1);
  if (strcmp(p_str,"rmvb")==0) return(1);
  if (strcmp(p_str,"tta")==0) return(1);
  if (strcmp(p_str,"wav")==0) return(1);
  if (strcmp(p_str,"wma")==0) return(1);
  if (strcmp(p_str,"wmv")==0) return(1);
  if (strcmp(p_str,"wv")==0) return(1);
  if (strcmp(p_str,"d9")==0) return(1);
  if (strcmp(p_str,"mpg")==0) return(1);
  if (strcmp(p_str,"vob")==0) return(1);
  if (strcmp(p_str,"ts")==0) return(1);
  if (strcmp(p_str,"mov")==0) return(1);
  if (strcmp(p_str,"dat")==0) return(1);
  if (strcmp(p_str,"part")==0) return(1);
  
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
  //char str1[3000];
  //struct stat info;

  strcpy(m105_str1,path);
  av_strlcat(m105_str1,"/" ,3000);
  av_strlcat(m105_str1,name,3000);

#if !defined(_WIN32) && !defined(__APPLE__)
  stat64(m105_str1,&deb_m_info);
#else
  stat(m105_str1,&deb_m_info);
#endif

  if (S_ISDIR(deb_m_info.st_mode)) return(1);
  else return(0);
}

static int cpy_string(char *p_s1,int p_s1_size,char *p_s2,int p_s2_size)
{
  int i;

  p_s1[0]=0;

  for (i=0;i<p_s2_size;i++)
  {
    if (i+1>=p_s1_size) break;
    else
    {
      if (p_s2[i]==0) break;
      else
      {
        p_s1[i+0]=p_s2[i];
        p_s1[i+1]=0;
      }
    }
  }

  return(0);
}

/*
static int deb_utf8_to_gb18030(char *inbuffer,char *outbuffer,int outbufferlen)
{

    iconv_t cd = iconv_open("gb18030//TRANSLIT", "utf-8");  
    int inbufferlen = strlen(inbuffer);   
    char* inbuffertmp  = inbuffer;  
    char* outbuffertmp = outbuffer;
    size_t ret = iconv(cd, &inbuffertmp, (size_t *)&inbufferlen, &outbuffertmp, (size_t *)&outbufferlen);  
    iconv_close(cd);  
    
    return(0);  
}
*/

static char  m501_str1[5000];
static char *m501_str1_ptr1[2];

static char  m501_str2[5000];
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

    for (i=0;i<5000;i++)
    {
      m501_str2[i]=0;
    }

    out_buffer_tmp_len = 5000;

    m501_str2_ptr1[0]=m501_str2;
    m501_str2_ptr1[1]=NULL;

    out_buffer_tmp=m501_str2_ptr1;

    iconv(cd, in_buffer_tmp, (size_t *)&in_buffer_tmp_len, out_buffer_tmp, (size_t *)&out_buffer_tmp_len);

    iconv_close(cd);

    cpy_string(out_buffer,out_buffer_len,m501_str2,5000);    

    return(0);  
}

static char disp_buff[1000][2000];
static char m11_str1[3000];
static char m11_str2[3000];
static char m11_str3[3000];
static char m11_str4[3000];

static int deb_disp_dir(VideoState *cur_stream)
{
  int i,j;
  int  h,w;
  int  mi;
  //char disp_buff[1000][2000];
  int leftspace,dirlen,dirtype,filelen,start;
  int  n1,n2,n3,n4,n5;
  //char str1[3000];
  //char str2[3000];
  //char str3[3000];
  //char str4[3000];
  char c1,c2,c3;
  char *strptr;
  int  bgcolor;

  start=(-1);

  for (n1=0;n1<1000;n1++)
    for (n2=0;n2<2000;n2++) disp_buff[n1][n2]=0;

  h = (cur_stream->height /deb_ch_h)-2;
  w = cur_stream->width /deb_ch_w ;

  if (h<0) return(0);
  if (h>1000-2) return(0);

  if (w<0) return(0);
  if (w>=2000) return(0);

  deb_ch_m = w;

  for (n1=0;n1<h;n1++)
  {
    if (n1>=1000) continue;

    for (n2=0;n2<w;n2++)
    {
      if (n2>=2000-1) continue;

      disp_buff[n1][n2+0]=' ';
      disp_buff[n1][n2+1]=0;
    }
  }

  for (n1=0;n1<w;n1=n1+2) 
  {
    if (n1>=2000-2) continue;

    disp_buff[0][n1+0]=deb_tableline[(10-1)*2+0];
    disp_buff[0][n1+1]=deb_tableline[(10-1)*2+1];
  }
/*
  for (n1=0;n1<w;n1=n1+2)
  {
    if (n1>=2000-2) continue;

    disp_buff[2][n1+0]=deb_tableline[(10-1)*2+0];
    disp_buff[2][n1+1]=deb_tableline[(10-1)*2+1];
  }
*/
  for (n1=0;n1<w;n1=n1+2)
  {
    if (h-1<0) continue;
    if (h-1>=1000) continue;

    if (n1>=2000-2) continue;

    disp_buff[h-1][n1+0]=deb_tableline[(10-1)*2+0];
    disp_buff[h-1][n1+1]=deb_tableline[(10-1)*2+1];
  }

  for (n1=0;n1<h;n1++)
  {
    if (n1<0) continue;
    if (n1>=1000) continue;

    disp_buff[n1][0]=deb_tableline[(11-1)*2+0];
    disp_buff[n1][1]=deb_tableline[(11-1)*2+1];
  }

  for (n1=0;n1<h;n1++)
  {
    if (n1<0) continue;
    if (n1>=1000) continue;

    if (w-2<0) continue;
    if (w-2>=2000-2) continue;

    disp_buff[n1][w-2]=deb_tableline[(11-1)*2+0];
    disp_buff[n1][w-1]=deb_tableline[(11-1)*2+1];
  }

  disp_buff[0  ][0  ]=deb_tableline[(1-1)*2+0];
  disp_buff[0  ][1  ]=deb_tableline[(1-1)*2+1];
/*
  disp_buff[2  ][0  ]=deb_tableline[(4-1)*2+0];
  disp_buff[2  ][1  ]=deb_tableline[(4-1)*2+1];
*/
  if ((h-1>=0)&&(h-1<1000))
  {
    disp_buff[h-1][0  ]=deb_tableline[(7-1)*2+0];
    disp_buff[h-1][1  ]=deb_tableline[(7-1)*2+1];
  }
/*
  if ((w-2>=0)&&(w-2<2000-2))
  {
    disp_buff[0  ][w-2]=deb_tableline[(3-1)*2+0];
    disp_buff[0  ][w-1]=deb_tableline[(3-1)*2+1];
  }
*/
  if ((w-2>=0)&&(w-2<2000-2))
  {
    disp_buff[0  ][w-2]='^';
    disp_buff[0  ][w-1]='^';
  }

  if ((h-1>=0)&&(h-1<1000))
  {
    if ((w-2>=0)&&(w-2<2000-2))
    {
      disp_buff[h-1][w-2]='V';
      disp_buff[h-1][w-1]='V';
    }
  }
/*
  if (deb_str_has_null(deb_currentpath,FN_SIZE)!=1) return(0);

  strcpy(m11_str1,deb_currentpath);

  if (deb_str_has_null(m11_str1,3000)!=1) return(0);

  for (n5=0;n5<3000;n5++) m11_str4[n5]=0;

#if !defined(_WIN32) && !defined(__APPLE__)
  deb_utf8_to_gb18030(m11_str1,m11_str4,3000);
#else
  strcpy(m11_str4,m11_str1);
#endif

  if (deb_str_has_null(m11_str4,3000)!=1) return(0);

  n1=(int)strlen(m11_str4);
  n2=0;

  m11_str3[0]=0;
  n3=0;

  while (n2<n1)
  {
    c1=m11_str4[n2];
    if (c1>=0)
    {
      if (n3+1<=w-4)
      {
        m11_str2[0]=c1;
        m11_str2[1]=0;

        m11_str3[n3+0]=c1;
	m11_str3[n3+1]=0;

	n3++;
	n2++;
      }
      else break;
    }
    else
    {
      if (n3+2<=w-4)
      {
	c2=m11_str4[n2+1];
	m11_str2[0]=c1;
	m11_str2[1]=c2;
	m11_str2[2]=0;

	m11_str3[n3+0]=c1;
	m11_str3[n3+1]=c2;
	m11_str3[n3+2]=0;

	n3=n3+2;
	n2=n2+2;
      }
      else break;
    }
  }

  for (n1=0;n1<(int)strlen(m11_str3);n1++)
  {
    disp_buff[1][n1+2]=m11_str3[n1];
  }
*/
  for (n4=0;n4<h-2;n4++)
  {
    if (deb_filenamebuff_n+n4<0) continue;
    if (deb_filenamebuff_n+n4>=MAX_FILE_NUM) continue;
    if (deb_filenamebuff_n+n4>=deb_filenamecnt) continue;

    if (deb_str_has_null(deb_filenamebuff[deb_filenamebuff_n+n4],FN_SIZE)!=1) continue;

    if (strlen(deb_filenamebuff[deb_filenamebuff_n+n4])>=FN_SIZE-4) continue;

    strcpy(m11_str1,deb_filenamebuff[deb_filenamebuff_n+n4]);

    if (deb_str_has_null(m11_str1,3000)!=1) continue;

    for (n5=0;n5<3000;n5++) m11_str4[n5]=0;

#if !defined(_WIN32) && !defined(__APPLE__)
    deb_utf8_to_gb18030(m11_str1,m11_str4,3000);
#else
    strcpy(m11_str4,m11_str1);
#endif

    if (deb_str_has_null(m11_str4,3000)!=1) return(0);

    if (start<0)
    {
      leftspace=deb_get_space(m11_str1);
      if (leftspace>0)
      {
        i      =deb_get_dir_len(deb_filenamebuff_n+n4);
	dirlen =deb_filenamebuff_len[i];
	dirtype=deb_filenamebuff_type[i];
	start  =leftspace;

	//printf("cur=%s,leftspace=%d,dirlen=%d,up=%s,\n",deb_filenamebuff[deb_filenamebuff_n+n4],leftspace,dirlen,
	//				                  deb_filenamebuff[i]);
      }
      else
      {
        dirlen =10;
	dirtype=0;
	start  =0;
      }
    }
    else
    {
      leftspace=deb_get_space(m11_str1);
      if (start!=leftspace)
      {
	leftspace=deb_get_space(m11_str1);
	if (leftspace>0)
	{
	  i      =deb_get_dir_len(deb_filenamebuff_n+n4);
	  dirlen =deb_filenamebuff_len[i];
	  dirtype=deb_filenamebuff_type[i];
	  start  =leftspace;

	  //printf("cur=%s,leftspace=%d,dirlen=%d,up=%s,\n",deb_filenamebuff[deb_filenamebuff_n+n4],leftspace,dirlen,
	  //						  deb_filenamebuff[i]);
	}
	else
	{
	  dirlen =10;
	  dirtype=0;
	  start  =0;
	}
      }
    }

    c3=deb_getfirstchar(deb_filenamebuff[deb_filenamebuff_n+n4]);

    if ((c3!='<')&&(c3!='|'))
    {
      if (leftspace+dirlen+35<=w-4)
      {
        filelen=leftspace+dirlen;
	mi=0;
      }
      else
      {
	filelen=w-4-35;
	if (filelen>=(int)strlen(m11_str4)) mi=0;
	else mi=1;
      }

      if (filelen<leftspace+3) continue;

      n1=(int)strlen(m11_str4);
      n2=0;

      m11_str3[0]=0;
      n3=0;

      if (mi==0)
      {
        while (n2<n1)
        {
          c1=m11_str4[n2];
          if (c1>=0)
          {
            if (n3+1<=filelen)
            {
              m11_str2[0]=c1;
              m11_str2[1]=0;
	
              m11_str3[n3+0]=c1;
              m11_str3[n3+1]=0;

              n3++;
              n2++;
            }
            else break;
          }
          else
          {
            if (n3+2<=filelen)
            {
              c2=m11_str4[n2+1];
              m11_str2[0]=c1;
              m11_str2[1]=c2;
              m11_str2[2]=0;

              m11_str3[n3+0]=c1;
              m11_str3[n3+1]=c2;
              m11_str3[n3+2]=0;

              n3=n3+2;
              n2=n2+2;
            }
            else break;
          }
        }
      }
      else
      {
        while (n2<n1)
	{
          c1=m11_str4[n2];
          if (c1>=0)
          {
            if (n3+1+1<=filelen)
            {
              m11_str2[0]=c1;
              m11_str2[1]=0;
	
              m11_str3[n3+0]=c1;
              m11_str3[n3+1]=0;

              n3++;
              n2++;
            }
            else break;
          }
          else
          {
            if (n3+2+1<=filelen)
            {
              c2=m11_str4[n2+1];
              m11_str2[0]=c1;
              m11_str2[1]=c2;
              m11_str2[2]=0;

              m11_str3[n3+0]=c1;
              m11_str3[n3+1]=c2;
              m11_str3[n3+2]=0;

              n3=n3+2;
              n2=n2+2;
            }
            else break;
          }
        }

        m11_str3[n3+0]='-';
        m11_str3[n3+1]=0;
        n3++;

      }

      if ((1+n4>=0)&&(1+n4<1000))
      {
        if (n1+2<0) continue;
	if (n1+2>=2000) continue;

	if (n1<0) continue;
	if (n1>=3000-1) continue;

	if (deb_str_has_null(m11_str3,3000)!=1) continue;

	for (n1=0;n1<(int)strlen(m11_str3);n1++) disp_buff[1+n4][n1+2]=m11_str3[n1];
      }

      for (j=0;j<(int)strlen(deb_filenamebuff_ext[deb_filenamebuff_n+n4]);j++)
      {
	if (j>=4) break;
	disp_buff[1+n4][filelen+2+j+2]=deb_filenamebuff_ext[deb_filenamebuff_n+n4][j];
      }

      for (j=0;j<(int)strlen(deb_filenamebuff_size[deb_filenamebuff_n+n4]);j++)
      {
	if (j>=6) break;
	disp_buff[1+n4][filelen+2+4+2+j+2]=deb_filenamebuff_size[deb_filenamebuff_n+n4][j];
      }

      for (j=0;j<(int)strlen(deb_filenamebuff_date[deb_filenamebuff_n+n4]);j++)
      {
	if (j>=19) break;
	disp_buff[1+n4][filelen+2+4+2+6+2+j+2]=deb_filenamebuff_date[deb_filenamebuff_n+n4][j];
      }
    }
    else
    {
      n1=(int)strlen(m11_str4);
      n2=0;

      m11_str3[0]=0;
      n3=0;

      while (n2<n1)
      {
	c1=m11_str4[n2];
	if (c1>=0)
	{
	  if (n3+1<=w-4)
	  {
	    m11_str2[0]=c1;
	    m11_str2[1]=0;

	    m11_str3[n3+0]=c1;
	    m11_str3[n3+1]=0;

	    n3++;
	    n2++;
	  }
	  else break;
	}
	else
	{
	  if (n3+2<=w-4)
	  {
	    c2=m11_str4[n2+1];
	    m11_str2[0]=c1;
	    m11_str2[1]=c2;
	    m11_str2[2]=0;

	    m11_str3[n3+0]=c1;
	    m11_str3[n3+1]=c2;
	    m11_str3[n3+2]=0;

	    n3=n3+2;
	    n2=n2+2;
	  }
	  else break;
	}
      }

      if ((1+n4>=0)&&(1+n4<1000))
      {
        if (n1+2<0) continue;
	if (n1+2>=2000) continue;

	if (n1<0) continue;
	if (n1>=3000-1) continue;

	if (deb_str_has_null(m11_str3,3000)!=1) continue;

	for (n1=0;n1<(int)strlen(m11_str3);n1++) disp_buff[1+n4][n1+2]=m11_str3[n1];
      }
    }
  }

  bgcolor = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);//daipozhi modified

  fill_rectangle(screen,cur_stream->width -deb_ch_w*deb_ch_m ,0 , deb_ch_w*deb_ch_m , deb_ch_h*h, bgcolor,0); // daipozhi modified 

  for (n4=0;n4<h;n4++)
  {
    if (n4>=1000) continue;

    strptr=disp_buff[n4];
    deb_echo_str4screenstring(cur_stream->width -deb_ch_w*deb_ch_m ,n4*deb_ch_h ,strptr ,deb_ch_m);
  }

  n4=deb_filenameplay-deb_filenamebuff_n;
  if ((n4>=0)&&(n4<h-2)) 
  {
    if (n4+1<1000)
    {
      strptr=disp_buff[n4+1];
      deb_echo_str4screenstring_black(cur_stream->width -deb_ch_w*deb_ch_m ,(n4+1)*deb_ch_h ,strptr ,deb_ch_m);
    }
  }

  SDL_UpdateRect(screen,cur_stream->width -deb_ch_w*deb_ch_m, 0, deb_ch_w*deb_ch_m , deb_ch_h*h);//daipozhi modified

  return(0);
}


static int deb_get_dir_len(int ptr)
{
    int i,j,k;

    i=deb_get_space(deb_filenamebuff[ptr]);

    while(ptr>0)
    {
	ptr--;
	j=deb_get_space(deb_filenamebuff[ptr]);
	if (j<i) break;
    }

    k=ptr;

    return(k);
}

static int deb_disp_bar(VideoState *is)
{
  VideoState *cur_stream2=is;

  int   i,j,k;
  int   n1,n2,n3;
  int   bgcolor2;

  float f1,f2;
  int   uns, uhh, umm, uss;
  int   tns, thh, tmm, tss;

  if (  (  (is->show_mode != SHOW_MODE_VIDEO)  &&  ((deb_thr_r!=1)||(deb_thr_a!=1)||(deb_thr_a2!=1)||(!is->audio_st))  )                 ||
        (  (is->show_mode == SHOW_MODE_VIDEO)  &&  ((deb_thr_r!=1)||(deb_thr_a!=1)||(deb_thr_a2!=1)||(deb_thr_v!=1)||(!is->video_st))  ) ||
        (  (seek_by_bytes || is->ic->duration<=0)   )   )
  {
    bgcolor2 = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);//daipozhi modified

    fill_rectangle(screen,0, cur_stream2->height-deb_ch_h*2 -deb_ch_d , cur_stream2->width, deb_ch_h*2 +deb_ch_d , bgcolor2,0); // daipozhi modified 

    uns = 0;
    uhh = 0;
    umm = 0;
    uss = 0;

    tns = 0;
    thh = 0;
    tmm = 0;
    tss = 0;

    snprintf(deb_scrn_str,300," %2d:%2d:%2d / %2d:%2d:%2d ",uhh,umm,uss,thh,tmm,tss);

    //snprintf(deb_scrn_str,300," %2d:%2d:%2d / %2d:%2d:%2d thr v=%d,thr a=%d,thr t=%d,thr d=%d,thr r=%d,timer_d=%d,frame_d=%d,bar=%d,eo_stream=%d,eo_que=%d,eo_pkt=%d,eo_buf=%d,n1=%d,n2=%d,",uhh,umm,uss,thh,tmm,tss,deb_thr_v,deb_thr_a,deb_thr_t,deb_thr_d,deb_thr_ref,deb_thr_timer_d,deb_thr_frame_d,deb_bar_cnt,deb_eo_stream,deb_eo_que,deb_eo_pkt,deb_eo_buffer,deb_n1,deb_n2);

    //deb_scrn_str[0]=0;

    n1=(cur_stream2->width)/6+1;

    if (n1<30  )  n1=30;
    if (n1>=2000) n1=2000-1;

    for (n3=/*0*/strlen(deb_scrn_str);n3<n1;n3++)
    {
      deb_scrn_str[n3+0]=' ';
      deb_scrn_str[n3+1]=0;
    }

    n2=(n1/2)-4;

    deb_scrn_str[n2+0]='[';
    deb_scrn_str[n2+1]='P';
    deb_scrn_str[n2+2]=' ';
    deb_scrn_str[n2+3]='l';
    deb_scrn_str[n2+4]=' ';
    deb_scrn_str[n2+5]='a';
    deb_scrn_str[n2+6]=' ';
    deb_scrn_str[n2+7]='y';
    deb_scrn_str[n2+8]=']';

    i=(cur_stream2->width)/1;

    if (i< 180 ) i=180;
    if (i>=8000) i=8000-1;

    j=0;

    for (k=0;k<=i;k++)
    {
      deb_scrn_str2[k+0]='-';
      deb_scrn_str2[k+1]=0;
    }

    deb_scrn_str2[j]='+';

    deb_echo_str4screenstring(0,cur_stream2->height-deb_ch_h*1-deb_ch_d,deb_scrn_str,n1);
    deb_echo_str4seekbar(cur_stream2->height-deb_ch_h*2-deb_ch_d,deb_scrn_str2);

    SDL_UpdateRect(screen, 0, cur_stream2->height-deb_ch_h*2-deb_ch_d, cur_stream2->width, deb_ch_h*2+deb_ch_d);//daipozhi modi
  }
  else
  {
    deb_thr_timer_d=deb_thr_timer_id-deb_thr_timer_id_old;
    deb_thr_timer_id_old=deb_thr_timer_id;

    deb_thr_frame_d=deb_thr_frame_id-deb_thr_frame_id_old;
    deb_thr_frame_id_old=deb_thr_frame_id;

    bgcolor2 = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);//daipozhi modified

    fill_rectangle(screen,0, cur_stream2->height-deb_ch_h*2 -deb_ch_d , cur_stream2->width, deb_ch_h*2 +deb_ch_d , bgcolor2,0); // daipozhi modified 


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

    //snprintf(deb_scrn_str,300," %2d:%2d:%2d / %2d:%2d:%2d thr v=%d,thr a=%d,thr t=%d,thr d=%d,thr r=%d,timer_d=%d,frame_d=%d,bar=%d,eo_stream=%d,eo_que=%d,eo_pkt=%d,eo_buf=%d,n1=%d,n2=%d,",uhh,umm,uss,thh,tmm,tss,deb_thr_v,deb_thr_a,deb_thr_t,deb_thr_d,deb_thr_ref,deb_thr_timer_d,deb_thr_frame_d,deb_bar_cnt,deb_eo_stream,deb_eo_que,deb_eo_pkt,deb_eo_buffer,deb_n1,deb_n2);

    //deb_scrn_str[0]=0;

    n1=(cur_stream2->width)/6+1;

    if (n1<30  )  n1=30;
    if (n1>=2000) n1=2000-1;

    for (n3=/*0*/strlen(deb_scrn_str);n3<n1;n3++)
    {
      deb_scrn_str[n3+0]=' ';
      deb_scrn_str[n3+1]=0;
    }

    n2=(n1/2)-4;

    if ((deb_st_play==1)&&(cur_stream2->paused==0)) 
    {
      deb_scrn_str[n2+0]='[';
      deb_scrn_str[n2+1]='P';
      deb_scrn_str[n2+2]=' ';
      deb_scrn_str[n2+3]='a';
      deb_scrn_str[n2+4]='u';
      deb_scrn_str[n2+5]='s';
      deb_scrn_str[n2+6]=' ';
      deb_scrn_str[n2+7]='e';
      deb_scrn_str[n2+8]=']';
    }
    else
    {
      deb_scrn_str[n2+0]='[';
      deb_scrn_str[n2+1]='P';
      deb_scrn_str[n2+2]=' ';
      deb_scrn_str[n2+3]='l';
      deb_scrn_str[n2+4]=' ';
      deb_scrn_str[n2+5]='a';
      deb_scrn_str[n2+6]=' ';
      deb_scrn_str[n2+7]='y';
      deb_scrn_str[n2+8]=']';
    }

    if (deb_cover==1)
    {
      if (deb_cover_close==0)
      {
	if (deb_sr_show==1)
        {
	  deb_scrn_str[n1-12]='[';
	  deb_scrn_str[n1-11]='R';
	  deb_scrn_str[n1-10]='i';
	  deb_scrn_str[n1-9 ]='v';
	  deb_scrn_str[n1-8 ]='e';
	  deb_scrn_str[n1-7 ]='r';
	  deb_scrn_str[n1-6 ]=' ';
	  deb_scrn_str[n1-5 ]='O';
	  deb_scrn_str[n1-4 ]='n';
	  deb_scrn_str[n1-3 ]=' ';
	  deb_scrn_str[n1-2 ]=']';
	}
	else
	{
	  //[Video On ]
	  //[Video Off]
	  deb_scrn_str[n1-12]='[';
	  deb_scrn_str[n1-11]='V';
	  deb_scrn_str[n1-10]='i';
	  deb_scrn_str[n1-9 ]='d';
	  deb_scrn_str[n1-8 ]='e';
	  deb_scrn_str[n1-7 ]='o';
	  deb_scrn_str[n1-6 ]=' ';
	  deb_scrn_str[n1-5 ]='O';
	  deb_scrn_str[n1-4 ]='f';
	  deb_scrn_str[n1-3 ]='f';
	  deb_scrn_str[n1-2 ]=']';
	}
      }
      else
      {
	if (deb_sr_show==1)
	{
	  if (deb_sr_show_nodisp==0)
	  {
	    //[Video On ]
	    //[Video Off]
	    deb_scrn_str[n1-12]='[';
	    deb_scrn_str[n1-11]='R';
	    deb_scrn_str[n1-10]='i';
	    deb_scrn_str[n1-9 ]='v';
	    deb_scrn_str[n1-8 ]='e';
	    deb_scrn_str[n1-7 ]='r';
	    deb_scrn_str[n1-6 ]=' ';
	    deb_scrn_str[n1-5 ]='O';
	    deb_scrn_str[n1-4 ]='f';
	    deb_scrn_str[n1-3 ]='f';
	    deb_scrn_str[n1-2 ]=']';
	  }
	  else
	  {
	    //[Video On ]
	    //[Video Off]
	    deb_scrn_str[n1-12]='[';
	    deb_scrn_str[n1-11]='V';
	    deb_scrn_str[n1-10]='i';
	    deb_scrn_str[n1-9 ]='d';
	    deb_scrn_str[n1-8 ]='e';
	    deb_scrn_str[n1-7 ]='o';
	    deb_scrn_str[n1-6 ]=' ';
	    deb_scrn_str[n1-5 ]='O';
	    deb_scrn_str[n1-4 ]='n';
	    deb_scrn_str[n1-3 ]=' ';
	    deb_scrn_str[n1-2 ]=']';
	  }
	}
	else
	{
	  //[Video On ]
	  //[Video Off]
	  deb_scrn_str[n1-12]='[';
	  deb_scrn_str[n1-11]='V';
	  deb_scrn_str[n1-10]='i';
	  deb_scrn_str[n1-9 ]='d';
	  deb_scrn_str[n1-8 ]='e';
	  deb_scrn_str[n1-7 ]='o';
	  deb_scrn_str[n1-6 ]=' ';
	  deb_scrn_str[n1-5 ]='O';
	  deb_scrn_str[n1-4 ]='n';
	  deb_scrn_str[n1-3 ]=' ';
	  deb_scrn_str[n1-2 ]=']';
	}
      }
    }
    else
    {
      if (deb_sr_show==1)
      {
	if (deb_sr_show_nodisp==0)
	{
	  deb_scrn_str[n1-12]='[';
	  deb_scrn_str[n1-11]='R';
	  deb_scrn_str[n1-10]='i';
	  deb_scrn_str[n1-9 ]='v';
	  deb_scrn_str[n1-8 ]='e';
	  deb_scrn_str[n1-7 ]='r';
	  deb_scrn_str[n1-6 ]=' ';
	  deb_scrn_str[n1-5 ]='O';
	  deb_scrn_str[n1-4 ]='f';
	  deb_scrn_str[n1-3 ]='f';
	  deb_scrn_str[n1-2 ]=']';
	}
	else
	{
	  deb_scrn_str[n1-12]='[';
	  deb_scrn_str[n1-11]='R';
	  deb_scrn_str[n1-10]='i';
	  deb_scrn_str[n1-9 ]='v';
	  deb_scrn_str[n1-8 ]='e';
	  deb_scrn_str[n1-7 ]='r';
	  deb_scrn_str[n1-6 ]=' ';
	  deb_scrn_str[n1-5 ]='O';
	  deb_scrn_str[n1-4 ]='n';
	  deb_scrn_str[n1-3 ]=' ';
	  deb_scrn_str[n1-2 ]=']';
	}
      }
      else
      {
	//[Video On ]
	//[Video Off]
	//deb_scrn_str[n1-12]='[';
	//deb_scrn_str[n1-11]='V';
	//deb_scrn_str[n1-10]='i';
	//deb_scrn_str[n1-9 ]='d';
	//deb_scrn_str[n1-8 ]='e';
	//deb_scrn_str[n1-7 ]='o';
	//deb_scrn_str[n1-6 ]=' ';
	//deb_scrn_str[n1-5 ]='O';
	//deb_scrn_str[n1-4 ]='f';
	//deb_scrn_str[n1-3 ]='f';
	//deb_scrn_str[n1-2 ]=']';
      }
    }

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

      i=(cur_stream2->width)/1-1;

      if (i< 180 ) i=180;
      if (i>=8000) i=8000-1;

      f1=uns;
      f2=tns;

      if ((f2>=1)&&(f1>=0))
      {
	j=i*(f1/f2);

	if (j<0) j=0;
	if (j>i) j=i;

	for (k=0;k<=i;k++)
	{
	  deb_scrn_str2[k+0]='-';
	  deb_scrn_str2[k+1]=0;
	}

	deb_scrn_str2[j]='+';
      }
      else
      {
	i=(cur_stream2->width)/1;

	if (i< 180 ) i=180;
	if (i>=8000) i=8000-1;

	j=0;

	for (k=0;k<=i;k++)
	{
		deb_scrn_str2[k+0]='-';
		deb_scrn_str2[k+1]=0;
	}

	deb_scrn_str2[j]='+';
      }
    }
    else
    {
      i=(cur_stream2->width)/1;

      if (i< 180 ) i=180;
      if (i>=8000) i=8000-1;

      j=0;

      for (k=0;k<=i;k++)
      {
	deb_scrn_str2[k+0]='-';
	deb_scrn_str2[k+1]=0;
      }

      deb_scrn_str2[j]='+';
    }

    deb_echo_str4screenstring(0,cur_stream2->height-deb_ch_h*1-deb_ch_d,deb_scrn_str,n1);
    deb_echo_str4seekbar(cur_stream2->height-deb_ch_h*2-deb_ch_d,deb_scrn_str2);

    SDL_UpdateRect(screen, 0, cur_stream2->height-deb_ch_h*2-deb_ch_d, cur_stream2->width, deb_ch_h*2+deb_ch_d);//daipozhi modi
  }

  return(0);
}

static int deb_disp_scrn(VideoState *is)
{
  VideoState *cur_stream2=is;
  int 	bgcolor;

  bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);//daipozhi modified

  fill_rectangle(screen,
                 0, 0,				//daipozhi modified
                 cur_stream2->width -deb_ch_w*deb_ch_m , 
	         cur_stream2->height-deb_ch_h*2-deb_ch_d,  //daipozhi modified
                 bgcolor,1);

  return(0);
}

static char deb_lower(char c1)
{
  if ((c1>='A')&&(c1<='Z')) return(c1-'A'+'a');
  else return(c1);
}

static int deb_lower_string(char *p_instr)
{
  int len;
  int i;

  len=(int)strlen(p_instr);
  for (i=0;i<len;i++) p_instr[i]=deb_lower(p_instr[i]);
  return(0);
}

static char deb_upper(char c1)
{
  if ((c1>='a')&&(c1<='z')) return(c1-'a'+'A');
  else return(c1);
}

static int deb_upper_string(char *p_instr)
{
  int len;
  int i;

  len=(int)strlen(p_instr);
  for (i=0;i<len;i++) p_instr[i]=deb_upper(p_instr[i]);
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

//  fclose(deb_record_fp);

  return(0);
}

static int deb_record_close(void)
{
  fclose(deb_record_fp);

  return(0);
}

static int deb_record(char *p_str1)
{
//  deb_record_fp=fopen("deb_record.txt","a");
//  if (deb_record_fp==NULL) return(1);

  fputs(p_str1,deb_record_fp);
  fputs("\n",deb_record_fp);

//  fclose(deb_record_fp);

  return(0);
}

// ---- binary tree -----------------------------------------------------------

static int t1_init_tree(void)
{
  int i,j;

  for (i=0;i<BTREE1_SIZE;i++)
  {
    t1_node_mark[i]=(-1);
  }

  t1_root_ptr=(-1);

  j=BTREE1_SIZE-1;  // init stack

  for (i=0;i<BTREE1_SIZE;i++)
  {
    t1_stack[i]=j;
    j--;
  }

  t1_stack_ptr=BTREE1_SIZE;

  return(0);
}

static int t1_new_node(void)
{
  int i,j;

  i=(-1);

  if (t1_stack_ptr>0)
  {
    t1_stack_ptr--;
    j=t1_stack[t1_stack_ptr];
    t1_node_mark[j]=0;
    i=j;
  }

  return(i);
}

static int t1_old_node(int ptr)
{
   if (t1_stack_ptr>=BTREE1_SIZE)
   {
     printf("In btree1,error at tn_old_node()\n");
     return(-1);
   }
   else
   {
     t1_stack[t1_stack_ptr]=ptr;
     t1_stack_ptr++;
     t1_node_mark[ptr]=(-1);
     return(0);
   }
}

static int t1_clear_node(int ptr)
{
  int i,j;
  
  t1_node_ptr[ptr][0]=(-1);
  t1_node_ptr[ptr][1]=(-1);
  t1_node_ptr[ptr][2]=(-1);

  for (i=0;i<FN_SIZE;i++)
  {
    t1_node_val[ptr][i]=0;
  }

  t1_node_val2[ptr]=0;
  t1_node_val3[ptr][0]=0;
  t1_node_val4[ptr][0]=0;
  t1_node_val5[ptr][0]=0;

  return(0);
}

static char m201_str1[3000];
static char m201_str2[3000];
static char m201_str3[3000];
static char m201_str4[3000];

static int t1_search_node(char *pstr,char ptype)
{
  int i,j;
  int res;

  if (deb_str_has_null(pstr,FN_SIZE)!=1) return(1);

  if (strlen(pstr)>=FN_SIZE) return(1);

  if (t1_root_ptr<0)
  {
     t1_find_ptr=(-1);
     return(1);
  }

  i=t1_root_ptr;

  t1_parent=(-1);
  t1_parent_side=2;

  t1_current=i;
  t1_child_left =t1_node_ptr[i][1];
  t1_child_right=t1_node_ptr[i][2];

  //printf("root into addr=%d,val=%d,\n",t1_current,t1_node_val[t1_current]);

  while (1)
  {

    str_lower_string(t1_node_val[i],m201_str1);
    str_lower_string(pstr,m201_str2);

    for (j=0;j<3000;j++)
    {
	m201_str3[j]=0;
	m201_str4[j]=0;
    }

#if !defined(_WIN32) && !defined(__APPLE__)
        deb_utf8_to_gb18030(m201_str1,m201_str3,3000);
        deb_utf8_to_gb18030(m201_str2,m201_str4,3000);
#else
	strcpy(m201_str3,m201_str1);
	strcpy(m201_str4,m201_str2);
#endif

    if ((string_comp(m201_str3,m201_str4)==0)&&(ptype==t1_node_val2[i]))
    {
      t1_find_ptr=i;
      return(0);
    }

    if ((ptype<t1_node_val2[i])||
	((ptype==t1_node_val2[i])&&(string_comp(m201_str3,m201_str4)>0)))
    {
      if (t1_node_ptr[i][2]<0)
      {
        t1_find_ptr=i;
        t1_find_side=2;
        return(1);
      }
      else
      {
        t1_parent=i;
        t1_parent_side=2;

        i=t1_node_ptr[i][2];

        t1_current=i;
        t1_child_left =t1_node_ptr[i][1];
        t1_child_right=t1_node_ptr[i][2];

        //printf("right into addr=%d,val=%d,\n",t1_current,t1_node_val[t1_current]);

        continue;
      }
    }
    
    if ((ptype>t1_node_val2[i])||
	((ptype==t1_node_val2[i])&&(string_comp(m201_str3,m201_str4)<0)))
    {
      if (t1_node_ptr[i][1]<0)
      {
        t1_find_ptr=i;
        t1_find_side=1;
        return(1);
      }
      else
      {
        t1_parent=i;
        t1_parent_side=1;

        i=t1_node_ptr[i][1];

        t1_current=i;
        t1_child_left =t1_node_ptr[i][1];
        t1_child_right=t1_node_ptr[i][2];

        //printf("left into addr=%d,val=%d,\n",t1_current,t1_node_val[t1_current]);

        continue;
      }
    }
    
  }

}

static int string_comp(char *ps1,char *ps2)
{
  int 		l1,l2,i;
  unsigned char c1,c2;

  if (deb_str_has_null(ps1,FN_SIZE)!=1) return(0);
  if (deb_str_has_null(ps2,FN_SIZE)!=1) return(0);

  if (strlen(ps1)>=FN_SIZE) return(0);
  if (strlen(ps2)>=FN_SIZE) return(0);

  l1=strlen(ps1);
  l2=strlen(ps2);

  for (i=0;i<=l1;i++) {
    c1=ps1[i];
    c2=ps2[i];
    if (c1==0) {
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

static int t1_insert_node(char *pstr,char ptype)
{
  int i,j;

  if (deb_str_has_null(pstr,FN_SIZE)!=1) return(1);

  if (strlen(pstr)>=FN_SIZE) return(1);

  i=t1_search_node(pstr,ptype);

  if (i==0)
  {
    t1_find_ptr2=t1_find_ptr;
    return(0);
  }
  else
  {
    if (t1_find_ptr<0)
    {
      j=t1_new_node();
      if (j<0)
      {
        printf("In btree1,error at insert_node() when call new_node()\n");
        return(1);
      }
      else
      {
        t1_root_ptr=j;
        t1_clear_node(j);

        strcpy(t1_node_val[j],pstr);
	t1_node_val2[j]=ptype;

	t1_find_ptr2=j;
        return(0);
      }
    }
    else
    {
      j=t1_new_node();
      if (j<0)
      {
        printf("In btree1,error at insert_node() when call new_node()\n");
        return(1);
      }
      else
      {
        t1_clear_node(j);

        strcpy(t1_node_val[j],pstr);
	t1_node_val2[j]=ptype;

        t1_node_ptr[j][0]=t1_find_ptr;

        if (t1_find_side==2) t1_node_ptr[t1_find_ptr][2]=j;
        else t1_node_ptr[t1_find_ptr][1]=j;

	t1_find_ptr2=j;

        return(0);
      }
    }
  }
}

static int t1_delete_node(char *pstr,char ptype)
{
  int i,j;
  int s1,sp;
  char str[300];

  i=t1_search_node(pstr,ptype);

  if (i==0)
  {
    if ((t1_child_left<0)&&(t1_child_right<0))  // no child
    {
      //printf("0 child\n");

      if (t1_parent<0)  // it is root
      {
	t1_root_ptr=(-1);
      }
      else
      {
        if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=(-1);
        if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=(-1);
      }
      t1_old_node(t1_find_ptr);

      //printf("0 child end\n");
    }

    if (((t1_child_left>=0)&&(t1_child_right<0))||  // one child
        ((t1_child_left<0)&&(t1_child_right>=0)))
    {
      //printf("1 child\n");

      if (t1_parent<0) // it is root;
      {
        if (t1_child_left>=0)
        {
          t1_root_ptr=t1_child_left;
        }

        if (t1_child_right>=0)
        {
          t1_root_ptr=t1_child_right;
        }
      }
      else
      {
        if (t1_child_left>=0)
        {
	  if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=t1_child_left;
	  if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=t1_child_left;
        }

        if (t1_child_right>=0)
        {
	  if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=t1_child_right;
	  if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=t1_child_right;
        }
      }
      t1_old_node(t1_find_ptr);

      //printf("1 child end\n");
    }

    if ((t1_child_left>=0)&&(t1_child_right>=0))  // two child
    {
      //printf("2 child\n");

      s1=t1_child_right;  // right sub tree
      sp=(-1);

      while (1)  // right sub tree's leftest node
      {
	if (t1_node_ptr[s1][1]>=0)
        {
          sp=s1;
          s1=t1_node_ptr[s1][1];
        }
	else break;
      }

      if (t1_node_ptr[s1][2]<0) // this node has no child
      {
	if (sp<0) // this node's parent is current node
        {
          if (t1_parent<0) // current node is root;
          {
	    t1_root_ptr=s1;
          }
          else
          {
	    if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=s1;
	    if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=s1;
          }
	  t1_node_ptr[s1][1]=t1_child_left;
	  //t1_node_ptr[s1][2]=(-1);
        }
	else
        {
	  t1_node_ptr[sp][1]=(-1);

          if (t1_parent<0) // current node is root;
          {
	    t1_root_ptr=s1;
          }
          else
          {
	    if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=s1;
	    if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=s1;
          }
	  t1_node_ptr[s1][1]=t1_child_left;
	  t1_node_ptr[s1][2]=t1_child_right;
        }
      }
      else // this node has one child
      {
	if (sp<0) // this node's parent is current node
        {
          if (t1_parent<0) // current node is root;
          {
	    t1_root_ptr=s1;
          }  
          else
          {
	    if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=s1;
	    if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=s1;
          }
	  t1_node_ptr[s1][1]=t1_child_left;
	  //t1_node_ptr[s1][2]=(-1);
        }
	else
        {
	  t1_node_ptr[sp][1]=t1_node_ptr[s1][2];

          if (t1_parent<0) // current node is root;
          {
	    t1_root_ptr=s1;
          }
          else
          {
	    if (t1_parent_side==1) t1_node_ptr[t1_parent][1]=s1;
	    if (t1_parent_side==2) t1_node_ptr[t1_parent][2]=s1;
          }
	  t1_node_ptr[s1][1]=t1_child_left;
	  t1_node_ptr[s1][2]=t1_child_right;
        }
      }

      t1_old_node(t1_find_ptr);

      //printf("2 child end\n");
    }

    return(0);
  }
  else return(1);

}

static int t1_smallest(void)
{
  int i,j;

  if (t1_root_ptr<0)
  {
     t1_find_ptr=(-1);
     return(1);
  }

  i=t1_root_ptr;

  while (1)
  {
    if (t1_node_ptr[i][1]<0)
    {
      t1_find_ptr=i;
      return(0);
    }
    else
    {
      i=t1_node_ptr[i][1];
      continue;
    }    
  }

}

//static char m03_str1[300];

static int t1_after_list(void)
{
  int  i,j,k;
  //char str1[300];

  t1_list_ptr=0;
  t1_out_ptr=0;
  t1_err=0;
  
  i=t1_root_ptr;
  if (i<0) return(0);

  if (t1_node_ptr[i][1]>=0)
  {
    t1_list_stack[t1_list_ptr]=t1_node_ptr[i][1];
    t1_list_stack_type[t1_list_ptr]=1;
    t1_list_ptr++;
  }

  t1_list_stack[t1_list_ptr]=i;
  t1_list_stack_type[t1_list_ptr]=2;
  t1_list_ptr++;
  
  if (t1_node_ptr[i][2]>=0)
  {
    t1_list_stack[t1_list_ptr]=t1_node_ptr[i][2];
    t1_list_stack_type[t1_list_ptr]=1;
    t1_list_ptr++;
  }

  while (t1_list_ptr>0)
  {
    t1_list_ptr--;
    j=t1_list_ptr;

    if (t1_list_stack_type[j]==1)
    {
      k=t1_list_stack[j];
      
      if (t1_node_ptr[k][1]>=0)
      {
        t1_list_stack[t1_list_ptr]=t1_node_ptr[k][1];
        t1_list_stack_type[t1_list_ptr]=1;
        t1_list_ptr++;

        //sprintf(str1,"add left tree %s,list_ptr=%d,",node_val[node_ptr[k][1]],list_ptr);
        if (t1_list_ptr>=BTREE1_LSIZE)
        {
          printf("In btree1,error in after_list(),BTREE1_LSIZE too small.\n");
          continue;
        }
      }

      t1_list_stack[t1_list_ptr]=k;
      t1_list_stack_type[t1_list_ptr]=2;
      t1_list_ptr++;

      //sprintf(str1,"add mid tree %s,list_ptr=%d,",node_val[k],list_ptr);
      if (t1_list_ptr>=BTREE1_LSIZE)
      {
        printf("In btree1,error in after_list(),BTREE1_LSIZE too small.\n");
        continue;
      }

      if (t1_node_ptr[k][2]>=0)
      {
        t1_list_stack[t1_list_ptr]=t1_node_ptr[k][2];
        t1_list_stack_type[t1_list_ptr]=1;
        t1_list_ptr++;

        //sprintf(str1,"add right tree %s,list_ptr=%d,",node_val[node_ptr[k][2]],list_ptr);
        if (t1_list_ptr>=BTREE1_LSIZE)
        {
          printf("In btree1,error in after_list(),BTREE1_LSIZE too small.\n");
          continue;
        }
      }
    }
    else
    {
      k=t1_list_stack[j];

      t1_out_list(t1_node_val[k],t1_node_val2[k],k);

      //sprintf(str1,"out val %s,",node_val[k]);
      //MessageBoxNow(0,str1,"message",MB_OK);
    }
  }

  return(0);
}

static int t1_out_list(char *pstr,char ptype,int ptr)
{
  if (t1_out_ptr<0) return(0);
  if (t1_out_ptr>=BTREE1_SIZE) return(0);

  if (deb_str_has_null(pstr,FN_SIZE)!=1) return(0);

  if (strlen(pstr)>=FN_SIZE) return(0);
  
  strcpy(t1_out_buff[t1_out_ptr],pstr);
  t1_out_buff2[t1_out_ptr]=ptype;
  strcpy(t1_out_buff3[t1_out_ptr],t1_node_val3[ptr]);
  strcpy(t1_out_buff4[t1_out_ptr],t1_node_val4[ptr]);
  strcpy(t1_out_buff5[t1_out_ptr],t1_node_val5[ptr]);

  t1_out_ptr++;

  if (t1_out_ptr>=BTREE1_SIZE) t1_out_ptr=BTREE1_SIZE-1;

  return(0);
}

// ---- end of binary tree ---------------------------------------

static char          m202_buffer1[3000];
static char          m202_buffer2[3000];
static char	     m202_buffer4[300];
static char          m202_buffer5[3000];
static char          m202_buffer7[3000];
static char          m202_type;
static char          m202_ext[6];
static char          m202_size[7];
static char          m202_date[20];

// 1.23MB
// 123456

// 2017-01-01 08:15:01
// 1234567890123456789

static int  deb_get_dir(void)
{
  DIR           *dirp;
  struct dirent *entry;

  int  	i,j;
  char 	buffer3[20];

  //deb_record_init();

  deb_m_info_len =0;
  deb_m_info_type=0;

  t1_init_tree();

  getcwd(deb_currentpath,FN_SIZE);

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

	  if (strlen(entry->d_name)>=255-2) continue;

	  strcpy(m202_buffer2,"<");
	  av_strlcat(m202_buffer2,entry->d_name,255);
	  av_strlcat(m202_buffer2,">",FN_SIZE);

	  m202_type=0;
	  m202_ext[0] =0;
	  m202_size[0]=0;
	  m202_date[0]=0;

	  t1_insert_node(m202_buffer2,m202_type);
	  //node_val2[find_ptr2]=0;
	  strcpy(t1_node_val3[t1_find_ptr2],m202_ext);
	  strcpy(t1_node_val4[t1_find_ptr2],m202_size);
	  strcpy(t1_node_val5[t1_find_ptr2],m202_date);

	  //deb_utf8_to_gb18030(m202_buffer2,m202_buffer7,3000);
	  //j=(int)strlen(m202_buffer7);
	  //if (deb_m_info_len<j) deb_m_info_len=j;
	}
	else
	{
	  if (deb_str_has_null(entry->d_name,255)!=1) continue;

	  if (strlen(entry->d_name)>=255-2) continue;

	  strcpy(m202_buffer2,entry->d_name);

	  deb_m_info_type=1;

	  // type
	  m202_type=1;

	  // ext
	  deb_filenameext(entry->d_name,m202_buffer1);
	  if (((int)strlen(m202_buffer1)>4)||
	      ((int)strlen(m202_buffer1)<=0))
	  {
	    strcpy(m202_ext,"    ");
	  }
	  else
	  {
	    strcat(m202_buffer1,"    ");
	    m202_buffer1[4]=0;
	    str_lower_string(m202_buffer1,m202_ext);
	  }

	  //size
						  //          t  g  m  k  b
	  if ((deb_m_info.st_size<0)||(deb_m_info.st_size>=1000000000000000))
	  {
	    strcpy(m202_size,"****  ");
	  }
	  else
	  {
			    //      t  g  m  k  b
	    if (deb_m_info.st_size>=1000000000000)
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
	    }
	    else
	    {
			      //      g  m  k  b
	      if (deb_m_info.st_size>=1000000000)
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
	      }
	      else
	      {
				//      m  k  b
		if (deb_m_info.st_size>=1000000)
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
		}
		else
		{
				  //      k  b
		  if (deb_m_info.st_size>=1000)
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
		}
	      }
	    }
	  }

	  //time
	  deb_m_info_tm=localtime(&(deb_m_info.st_mtime));
	  sprintf(m202_buffer5,"%4d-%2d-%2d %2d:%2d:%2d",1900+deb_m_info_tm->tm_year,1+deb_m_info_tm->tm_mon,deb_m_info_tm->tm_mday,
						       deb_m_info_tm->tm_hour,deb_m_info_tm->tm_min,deb_m_info_tm->tm_sec);
	  if ((int)strlen(m202_buffer5)!=19) strcpy(m202_buffer5,"****               ");
	  strcpy(m202_date,m202_buffer5);

	  t1_insert_node(m202_buffer2,m202_type);
	  //t1_node_val2[t1_find_ptr2]=1;
	  strcpy(t1_node_val3[t1_find_ptr2],m202_ext);
	  strcpy(t1_node_val4[t1_find_ptr2],m202_size);
	  strcpy(t1_node_val5[t1_find_ptr2],m202_date);

	  //deb_utf8_to_gb18030(m202_buffer2,m202_buffer7,3000);
	  //j=(int)strlen(m202_buffer7);
	  //if (deb_m_info_len<j) deb_m_info_len=j;

	  for (j=0;j<3000;j++)
	  {
	    m202_buffer7[j]=0;
	  }

	  #if !defined(_WIN32) && !defined(__APPLE__)
	    deb_utf8_to_gb18030(m202_buffer2,m202_buffer7,3000);
	  #else
	    strcpy(m202_buffer7,m202_buffer2);
	  #endif

	  j=(int)strlen(m202_buffer7);

	  if (deb_m_info_len<j) deb_m_info_len=j;
	}
      }
      else break;
    }

    t1_after_list();
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
  else
  {
    if (pn>=10000)
    {
      i=pn/100;
      sprintf(buffer,"%4d",i);
    }
    else
    {
      if (pn>=1000)
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
    }
  }

  if ((int)strlen(buffer)>4)
  {
    strcpy(buffer,"****");
  }
  else
  {
    if ((int)strlen(buffer)<4)
    {
      strcat(buffer,"    ");
      buffer[4]=0;
    }
  }

  return(0);
}

static char  str_lower(char c1)
{
  if ((c1>='A')&&(c1<='Z')) return(c1-'A'+'a');
  else return(c1);
}

static int  str_lower_string(char *p_instr,char *p_outstr)
{
	int len;
	int i;

	len=(int)strlen(p_instr);
	p_outstr[0]=0;

	for (i=0;i<len;i++)
	{
		p_outstr[i+0]=str_lower(p_instr[i]);
		p_outstr[i+1]=0;
	}

	return(0);
}

static int deb_str_has_null(char *s1,int s1_size)
{
	int i;
	for (i=0;i<s1_size;i++) if (s1[i]==0) return(1);
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
  //char   c1,c2,cc[2];
  //long   m1,m2,m3,m4,m5,m6,n0,q1;


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

    if (deb_sr_fft_deb_chn==0) // to playback high freq , low freq ,middle freq ,to sure fft is ok
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

/*------  fft -------------------------------------------------------------

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

//static char m603_str1[300];

static int  deb_sr_river_f_cons(void)
{
  int i,j,k,l;
  int d1,d2,d3;
  int x1,y1,x2,y2,x3,y3,x4,y4,x5,y5;
  int h;

  //sprintf(m603_str1,"w=%d,h=%d,",screen->w,screen->h);
  //deb_record(str1);

  deb_sr_river_f_init     =1;
  deb_sr_river_f_init_fail=0;

  if ((screen->w<1204)||(screen->w>7680)) return(1);
  if ((screen->h<705 )||(screen->h>4320)) return(1);

  i=screen->w;
  d1=i/(150+1+1);

  h=3.0*(float)(screen->h);
  d2=(int)(((float)(screen->h)-100)/118);

  //d3=d2*0.8;
  //d3=d2*0.6;
  d3=(float)d2*0.55;

  x1=d1*76;                // miedian      vanishing point 
  y1=10;

  x2=d1;                   // start line
  y2=y1+d2*122+d2*118;

  deb_sr_win_top=y1+d2*122-100+5;

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

    deb_sr_d_cache_clr();
    l=deb_sr_draw_line(x1,y1,x2,y2);
    if (l!=0) return(1);

    for (j=118-1-1;j>=0;j--)
    {
      x3=0;
      y3=y2-(118-1-j)*d2;

      x4=screen->w-1;
      y4=y3;

      l=deb_sr_draw_line2(x3,y3,x4,y4);
      if (l!=0) return(1);

      deb_sr_river_f[i][j][0][0]=deb_sr_d_return[0];
      deb_sr_river_f[i][j][0][1]=deb_sr_d_return[1];
    }
  }

  for (i=0;i<151;i++)    // to 3d
  {
    for (j=118-1;j>=0;j--)
    {
      x2=deb_sr_river_f[i][j][0][0];
      y2=deb_sr_river_f[i][j][0][1];

      if (j==118-1)
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
        if (i==0)
        {
	  x3=x2;
	  y3=0;

	  x4=x2;
	  y4=h-1;

	  deb_sr_d_cache_clr();
	  l=deb_sr_draw_line(x3,y3,x4,y4);
          if (l!=0) return(1);

          for (k=1;k<60;k++)
          {
            x5=deb_sr_river_f[i][118-1][k][0];
            y5=deb_sr_river_f[i][118-1][k][1];

            l=deb_sr_draw_line2(x1,y1,x5,y5);
            if (l!=0) return(1);

            deb_sr_river_f[i][j][k][0]=deb_sr_d_return[0];
            deb_sr_river_f[i][j][k][1]=deb_sr_d_return[1];
          }

        }
        else
        {
/*
          for (k=1;k<60;k++)
          {
	    x3=deb_sr_river_f[i][69][k][0];
            y3=deb_sr_river_f[i][69][k][1];

	    deb_sr_d_cache_clr();
	    l=deb_sr_draw_line(x3,y3,x1,y1);
            if (l!=0) return(1);

            x4=0;
            y4=deb_sr_river_f[0][j][k][1];

	    x5=screen->w-1;
	    y5=y4;

            l=deb_sr_draw_line2(x4,y4,x5,y5);
            if (l!=0) return(1);

            deb_sr_river_f[i][j][k][0]=deb_sr_d_return[0];
            deb_sr_river_f[i][j][k][1]=deb_sr_d_return[1];
          }
*/
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

      x3=screen->w-1;
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
  struct timezone tz;
  long long int   li,lj,lk;
  int  bgcolor;
  int  bgcolor2;
  int  bgcolor3;
  int  bgcolor4;
  int  bgcolor5;
  int  s_first,s_x1,s_x2,s_y1,s_y2;
  int  s_p1,s_p2,s_p3,s_p4,s_p5,s_p6,s_p7,s_p8,s_p9,s_p10,s_p11,s_p12;
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
    if (i!=0) deb_sr_river_f_init_fail=1;
#if DPZ_DEBUG1
    deb_record("show construction return >>>");
#endif
    return(0);
  }
/*
  else
  {
*/
  if (deb_sr_river_f_init_fail==1)
  {
#if DPZ_DEBUG1
    deb_record("show init fail ,return >>>");
#endif
    return(0);
  }
/*
    else
    {
*/
  gettimeofday(&tv,&tz);
	
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

  lk=lk+2*deb_sr_ch*44100*(float)0.2;     // bytes    delay 0.2s

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
/*
      else
      {
*/
  if ((j/6)==(deb_sr_river_last/6)) //sr2
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show j=%d, already displayed(river[j]),return >>>",j);
    deb_record(m604_str1);
#endif
    return(0); // already displayed
  }
/*
        else
        {
*/
  m=j-6; //sr2
  if (m<0) m=396+m;
  if ((m/6)!=(deb_sr_river_last/6))  //sr2 // error,not continue;return
  {
#if DPZ_DEBUG1
    sprintf(m604_str1,"show m=j-6=%d, !=last_displayed ,    return >>>",m);
    deb_record(m604_str1);
#endif
    deb_sr_river_last=(j/6)*6;  //sr2
    return(0);
  }
/*
  else
  {
*/
#if DPZ_DEBUG1
  sprintf(m604_str1,"show j=%d,displayed >>>",j);
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
/*
  }
*/
  // show start
  deb_sr_river_last=(j/6)*6;  //sr2

  bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

  fill_rectangle(screen,0 ,0 , cur_stream->width , cur_stream->height -deb_ch_h*2-deb_ch_d , bgcolor,0); 

  bgcolor = SDL_MapRGB(screen->format, 0, 255, 0);

  freq_x=deb_sr_river_f[149][2][0][0]+10;
  freq_y=deb_sr_river_f[149][2][0][1];
  fill_rectangle(screen,freq_x,freq_y-deb_sr_win_top,15,1,bgcolor,0);
  deb_echo_str4screenstring_green(freq_x+20,freq_y-deb_sr_win_top-6,"300Hz",5);

  freq_x=deb_sr_river_f[149][6][0][0]+10;
  freq_y=deb_sr_river_f[149][6][0][1];
  fill_rectangle(screen,freq_x,freq_y-deb_sr_win_top,15,1,bgcolor,0);
  deb_echo_str4screenstring_green(freq_x+20,freq_y-deb_sr_win_top-6,"1KHz",4);

  freq_x=deb_sr_river_f[149][12][0][0]+10;
  freq_y=deb_sr_river_f[149][12][0][1];
  fill_rectangle(screen,freq_x,freq_y-deb_sr_win_top,15,1,bgcolor,0);
  deb_echo_str4screenstring_green(freq_x+20,freq_y-deb_sr_win_top-6,"2KHz",4);

  freq_x=deb_sr_river_f[149][29][0][0]+10;
  freq_y=deb_sr_river_f[149][29][0][1];
  fill_rectangle(screen,freq_x,freq_y-deb_sr_win_top,15,1,bgcolor,0);
  deb_echo_str4screenstring_green(freq_x+20,freq_y-deb_sr_win_top-6,"5KHz",4);

  freq_x=deb_sr_river_f[149][59][0][0]+10;
  freq_y=deb_sr_river_f[149][59][0][1];
  fill_rectangle(screen,freq_x,freq_y-deb_sr_win_top,15,1,bgcolor,0);
  deb_echo_str4screenstring_green(freq_x+20,freq_y-deb_sr_win_top-6,"10KHz",5);

  //front face
  s_p1=0;
  s_p2=255;
  s_p3=0;

  bgcolor = SDL_MapRGB(screen->format,s_p1,s_p2,s_p3);
  bgcolor2 = SDL_MapRGB(screen->format, 0, 0, 0);

  // right side face
  s_p4=s_p1*0.7;
  s_p5=s_p2*0.7;
  s_p6=s_p3*0.7;

  bgcolor3 = SDL_MapRGB(screen->format,s_p4,s_p5,s_p6);

  //up side face
  s_p7=s_p1*0.49;
  s_p8=s_p2*0.49;
  s_p9=s_p3*0.49;

  bgcolor4 = SDL_MapRGB(screen->format,s_p7,s_p8,s_p9);

  // left side face
  s_p10=s_p1*0.273;
  s_p11=s_p2*0.273;
  s_p12=s_p3*0.273;

  bgcolor5 = SDL_MapRGB(screen->format,s_p10,s_p11,s_p12);

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
	  fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,
			 deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],bgcolor,0);

	  fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,1,
			 deb_sr_river_f[i  ][k+1][0][1]-deb_sr_river_f[i  ][k+1][n][1],bgcolor2,0);
	  fill_rectangle(screen,deb_sr_river_f[i+1][k+1][n][0],
			 deb_sr_river_f[i+1][k+1][n][1]-deb_sr_win_top,1,
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],bgcolor2,0);
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
	fill_rectangle(screen,deb_sr_river_f[i  ][k+1][0][0],
		       deb_sr_river_f[i  ][k+1][0][1]-deb_sr_win_top,
		       deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
		       1,bgcolor,0);

	fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
		       deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,
		       deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0], 
		       1,bgcolor,0);

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
	    fill_rectangle(screen,deb_sr_d_line_dot[0][0],
			   deb_sr_d_line_dot[0][1]-deb_sr_win_top,1,
			   deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1],bgcolor3,0);
	  }
	}

	if (deb_sr_river_f[i+1][k  ][0][1]-deb_sr_river_f[i+1][k  ][n][1]>0)
	{
          // note ,for smaller moniter,bgcolor2 change to bgcolor3
	  fill_rectangle(screen,deb_sr_river_f[i+1][k  ][n][0],
			 deb_sr_river_f[i+1][k  ][n][1]-deb_sr_win_top,1,
			 deb_sr_river_f[i+1][k  ][0][1]-deb_sr_river_f[i+1][k  ][n][1],/*bgcolor2*/bgcolor3,0);
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


	fill_rectangle(screen,deb_sr_d_line_dot[0][0],
		       deb_sr_d_line_dot[0][1]-deb_sr_win_top,
		       deb_sr_d_line_dot[1][0]-deb_sr_d_line_dot[0][0],
		       1,bgcolor4,0);
      }

      fill_rectangle(screen,deb_sr_river_f[i  ][k  ][n][0],
		     deb_sr_river_f[i  ][k  ][n][1]-deb_sr_win_top,
		     deb_sr_river_f[i+1][k  ][n][0]-deb_sr_river_f[i  ][k  ][n][0],
		     1,bgcolor2,0);
// / * repeated
      fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
		     deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,
		     deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0],
		     1,bgcolor2,0);
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
	  fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,
			 deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],bgcolor,0);

	  fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
			 deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,1,
			 deb_sr_river_f[i  ][k+1][0][1]-deb_sr_river_f[i  ][k+1][n][1],bgcolor2,0);
	  fill_rectangle(screen,deb_sr_river_f[i+1][k+1][n][0],
			 deb_sr_river_f[i+1][k+1][n][1]-deb_sr_win_top,1,
			 deb_sr_river_f[i+1][k+1][0][1]-deb_sr_river_f[i+1][k+1][n][1],bgcolor2,0);
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
	fill_rectangle(screen,deb_sr_river_f[i  ][k+1][0][0],
		       deb_sr_river_f[i  ][k+1][0][1]-deb_sr_win_top,
		       deb_sr_river_f[i+1][k+1][0][0]-deb_sr_river_f[i  ][k+1][0][0], 
		       1,bgcolor,0);

	fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
		       deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,
		       deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0], 
		       1,bgcolor,0);

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
	    fill_rectangle(screen,deb_sr_d_line_dot[0][0],
			   deb_sr_d_line_dot[0][1]-deb_sr_win_top,1,
			   deb_sr_d_line_dot[1][1]-deb_sr_d_line_dot[0][1],bgcolor5,0);
	  }
	}

	if (deb_sr_river_f[i  ][k  ][0][1]-deb_sr_river_f[i  ][k  ][n][1]>0)
	{
          // note ,for smaller moniter,bgcolor2 change to bgcolor5
	  fill_rectangle(screen,deb_sr_river_f[i  ][k  ][n][0],
			 deb_sr_river_f[i  ][k  ][n][1]-deb_sr_win_top,1,
			 deb_sr_river_f[i  ][k  ][0][1]-deb_sr_river_f[i  ][k  ][n][1],/*bgcolor2*/bgcolor5,0);
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

	fill_rectangle(screen,deb_sr_d_line_dot[0][0],
		       deb_sr_d_line_dot[0][1]-deb_sr_win_top,
		       deb_sr_d_line_dot[1][0]-deb_sr_d_line_dot[0][0],
		       1,bgcolor4,0);
      }

      fill_rectangle(screen,deb_sr_river_f[i  ][k  ][n][0],
		     deb_sr_river_f[i  ][k  ][n][1]-deb_sr_win_top,
		     deb_sr_river_f[i+1][k  ][n][0]-deb_sr_river_f[i  ][k  ][n][0],
		     1,bgcolor2,0);
// / *  repeated
      fill_rectangle(screen,deb_sr_river_f[i  ][k+1][n][0],
		     deb_sr_river_f[i  ][k+1][n][1]-deb_sr_win_top,
		     deb_sr_river_f[i+1][k+1][n][0]-deb_sr_river_f[i  ][k+1][n][0],
		     1,bgcolor2,0);
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

  SDL_UpdateRect(screen, 0, 0, cur_stream->width , cur_stream->height -deb_ch_h*2-deb_ch_d);
/*
        }
      }
    }
  }
*/
  return(0);
}

static int  deb_sr_river_f_cons_test(VideoState *cur_stream,int ptr)
{
  int  bgcolor;
  int  x1,y1;
  int  i,j,k;

  if (deb_sr_river_f_init==0)
  {
    i=deb_sr_river_f_cons();
    if (i!=0) deb_sr_river_f_init_fail=1;
  }

  bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

  fill_rectangle(screen,0 ,0 , cur_stream->width , cur_stream->height -deb_ch_h*2-deb_ch_d , bgcolor,0); 

  for (i=0;i<151;i++)
  {
    for (j=0;j<118;j++)
    {
      for (k=0;k<=ptr;k++)
      {
	if (k>=60) continue;

        x1=deb_sr_river_f[i][j][k][0];
        y1=deb_sr_river_f[i][j][k][1];

        bgcolor = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);

        fill_rectangle(screen,x1,y1-deb_sr_win_top, 1, 1,bgcolor,0);
      }
    }
  }

  SDL_UpdateRect(screen, 0, 0, cur_stream->width , cur_stream->height -deb_ch_h*2-deb_ch_d);

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
    else
    {
      if (y2<y1)
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
    else
    {
      if (y2>y1)
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
    else
    {
      if (y2<y1)
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
    else
    {
      if (y2>y1)
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
  }

  return(0);
}

static int deb_sr_draw_line3(int x1,int y1,int x2,int y2)
{
  // screen top and left is x=0;y=0;
  int x3,y3,x4,y4,x5,y5;
  int i,j;
  int  bgcolor;

  bgcolor = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

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
  fill_rectangle(screen,x1,y1-deb_sr_win_top,1,1,bgcolor,0);

  if ((x2<0)||(x2>=7680)) return(1);
  if ((y2<0)||(y2>=4320)) return(1);
  fill_rectangle(screen,x2,y2-deb_sr_win_top,1,1,bgcolor,0);

  if (x1==x2)
  {
    if (y1<y2)
    {
      for (i=y1+1;i<=y2-1;i++)
      {
	if ((x1<0)||(x1>=7680)) return(1);
	if ((i<0)||(i>=4320)) return(1);
	fill_rectangle(screen,x1,i-deb_sr_win_top,1,1,bgcolor,0);//deb_sr_d_buff[x1][i]=1;
      }
    }
    else
    {
      if (y2<y1)
      {
        for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--)
	{
	  if ((x1<0)||(x1>=7680)) return(1);
	  if ((i<0)||(i>=4320)) return(1);
	  fill_rectangle(screen,x1,i-deb_sr_win_top,1,1,bgcolor,0); //deb_sr_d_buff[x1][i]=1;
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
	if ((i<0)||(i>=7680)) return(1);
	if ((y1<0)||(y1>=4320)) return(1);
	fill_rectangle(screen,i,y1-deb_sr_win_top,1,1,bgcolor,0);//deb_sr_d_buff[i][y1]=1;
      }
    }
    else
    {
      if (y2>y1)
      {
        x5=x1;
        y5=y1;

        for (i=1;i<=x2-x1;i++)
        {
          x4=x1+i;
          y4=y1+(y2-y1)*i/(x2-x1);

	  if ((x4<0)||(x4>=7680)) return(1);
	  if ((y4<0)||(y4>=4320)) return(1);
          fill_rectangle(screen,x4,y4-deb_sr_win_top,1,1,bgcolor,0);//deb_sr_d_buff[x4][y4]=1;

          if (y4>y5+1)
          {
            for (j=y5+1;j<y4;j++)
	    {
		if ((x5<0)||(x5>=7680)) return(1);
		if ((j<0)||(j>=4320)) return(1);
		fill_rectangle(screen,x5,j-deb_sr_win_top,1,1,bgcolor,0);//deb_sr_d_buff[x5][j]=1;
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
          fill_rectangle(screen,x4,y4-deb_sr_win_top,1,1,bgcolor,0);//deb_sr_d_buff[x4][y4]=1;

          if (y4<y5-1)
          {
            for (j=y5-1;j>y4;j--)
	    {
		if ((x5<0)||(x5>=7680)) return(1);
		if ((j<0)||(j>=4320)) return(1);
		fill_rectangle(screen,x5,j-deb_sr_win_top,1,1,bgcolor,0);//deb_sr_d_buff[x5][j]=1;
	    }
          }

          x5=x4;
	  y5=y4;
        }
      }
    }
  }

  return(0);
}

static int deb_sr_draw_line4_ini(void)
{
/*
	int i,j;

	for (i=0;i<2;i++)
		for (j=0;j<100;j++)
		{
			deb_sr_d_line[i][j][0]=(-1);
			deb_sr_d_line[i][j][1]=(-1);
		}
*/
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
  if ((ptr2<0)||(ptr2>=100)) return(0);
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
	  if ((ptr2<0)||(ptr2>=100)) return(0);
	  if ((x1<0)||(x1>=7680)) return(0);
	  if ((i<0)||(i>=4320)) return(0);
	  deb_sr_d_line[ptr][ptr2][0]=x1;
	  deb_sr_d_line[ptr][ptr2][1]=i;
	  ptr2++;
      }
    }
    else
    {
      if (y2<y1)
      {
        for (i=y1-1/*y2+1*/;i>=y2+1/*y1-1*/;i--) //fill_rectangle(screen,x1,i,1,1,bgcolor,0); //deb_sr_d_buff[x1][i]=1;
        {
	  if ((ptr <0)||(ptr >=2)) return(0);
	  if ((ptr2<0)||(ptr2>=100)) return(0);
	  if ((x1<0)||(x1>=7680)) return(0);
	  if ((i<0)||(i>=4320)) return(0);
	  deb_sr_d_line[ptr][ptr2][0]=x1;
	  deb_sr_d_line[ptr][ptr2][1]=i;
	  ptr2++;
        }
      }
      else return(0);
    }
  }
  else
  {
    if (y1==y2)
    {
      for (i=x1+1;i<=x2-1;i++) //fill_rectangle(screen,i,y1,1,1,bgcolor,0);//deb_sr_d_buff[i][y1]=1;
      {
	  if ((ptr <0)||(ptr >=2)) return(0);
	  if ((ptr2<0)||(ptr2>=100)) return(0);
	  if ((i<0)||(i>=7680)) return(0);
	  if ((y1<0)||(y1>=4320)) return(0);
	  deb_sr_d_line[ptr][ptr2][0]=i;
	  deb_sr_d_line[ptr][ptr2][1]=y1;
	  ptr2++;
      }
    }
    else
    {
      if (y2>y1)
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
		if ((ptr2<0)||(ptr2>=100)) return(0);
		if ((x5<0)||(x5>=7680)) return(0);
		if ((j<0)||(j>=4320)) return(0);
		deb_sr_d_line[ptr][ptr2][0]=x5;
		deb_sr_d_line[ptr][ptr2][1]=j;
		ptr2++;
	    }
          }

	  if ((ptr <0)||(ptr >=2)) return(0);
	  if ((ptr2<0)||(ptr2>=100)) return(0);
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
		if ((ptr2<0)||(ptr2>=100)) return(0);
		if ((x5<0)||(x5>=7680)) return(0);
		if ((j<0)||(j>=4320)) return(0);
		deb_sr_d_line[ptr][ptr2][0]=x5;
		deb_sr_d_line[ptr][ptr2][1]=j;
		ptr2++;
	    }
          }

	  if ((ptr <0)||(ptr >=2)) return(0);
	  if ((ptr2<0)||(ptr2>=100)) return(0);
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
  }

  if ((ptr <0)||(ptr >=2)) return(0);
  if ((ptr2<0)||(ptr2>=100)) return(0);
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
		if (i>=100) break;
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
		if (i>=100) break;
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

