#include "stdio.h"
#include "stdlib.h"
#include "inttypes.h"
#include "string.h"
#include "sys/stat.h"
#include "sys/mman.h"

#include "uv.h"
#include "mad.h"

uv_loop_t* loop;

uv_fs_t stat_req;
uv_fs_t open_req;
uv_fs_t read_req;
uv_fs_t close_req;
char *path;

uv_buf_t uv_buffer;

struct mad_stream mad_stream;
struct mad_frame mad_frame;
struct mad_synth mad_synth;

void stat_cb(uv_fs_t* req);
void open_cb(uv_fs_t* req);
void read_cb(uv_fs_t* req);
void close_cb(uv_fs_t* req);

void frame_decoding_process(uv_work_t *req);
void after_frame_decoding_process(uv_work_t *req, int status);

void decode_all_frames();
static 
enum mad_flow output(struct mad_header const *header, struct mad_pcm *pcm);

uv_work_t current_frame_decoding_req;
bool frame_decoding_finish_status = false;
int iterations_num = 0;

int main(int argc, char **argv) {

    loop = uv_default_loop();
    if (argc != 2) {
        fprintf(stderr, "File path is missing as parameter%s\n", argv[0]);
        return 255;
    }
    path = argv[1];
    uv_fs_stat(loop, &stat_req, path, stat_cb);

    uv_run(loop, UV_RUN_DEFAULT);
    
    return 0;
}

void stat_cb(uv_fs_t* req) {
    if (req -> result < 0) {
        printf("Error at getting stats of the file: %s\n", path);
    }

    uv_stat_t stats = req -> statbuf;
    char *buffer = new char[stats.st_size];
    uv_buffer = uv_buf_init(buffer, stats.st_size);
    uv_fs_req_cleanup(req);
    
    printf("%"PRId64"\n", stats.st_size);
    uv_fs_open(loop, &open_req, path, O_RDWR | O_APPEND, S_IRUSR, open_cb);
};

void open_cb(uv_fs_t* req) {
    if (req -> result < 0) {
        printf("Error at openning file: %s\n", path);
    }

    uv_file uf = req -> result;
    uv_fs_req_cleanup(req);

    uv_fs_read(loop, &read_req, uf, &uv_buffer, 1, -1, read_cb);
};

void read_cb(uv_fs_t* req) {
    if (req -> result < 0) {
        printf("Error at reading file: %s\n", path);
    }

    printf("content:\n%s\n", uv_buffer.base);
    uv_fs_req_cleanup(req);

    uv_fs_close(loop, &close_req, open_req.result, close_cb);
};

void close_cb(uv_fs_t* req) {
    if (req -> result < 0) {
        printf("Error at closing file: %s\n", path);
    } else {
        printf("file %s\n", "closed");    
    }
    
    uv_fs_req_cleanup(req);

    decode_all_frames();
};

void mad_init() {
    // Initialize MAD library
    mad_stream_init(&mad_stream);
    mad_synth_init(&mad_synth);
    mad_frame_init(&mad_frame);
}

void mad_finish() {
    mad_synth_finish(&mad_synth);
    mad_frame_finish(&mad_frame);
    mad_stream_finish(&mad_stream);
}

void decode_all_frames() {
    mad_init();
    mad_stream_buffer(&mad_stream, (const unsigned char *)uv_buffer.base, uv_buffer.len);

    /*while (1) {
        
        // Decode frame from the stream
        if (mad_frame_decode(&mad_frame, &mad_stream) == -1) {
            if (!MAD_RECOVERABLE(mad_stream.error)) {
                printf("%s\n", "finish");
                break;
            }
        }
        // Synthesize PCM data of frame
        mad_synth_frame(&mad_synth, &mad_frame);
        output(&mad_frame.header, &mad_synth.pcm);
        iterations_num++;
    }
    mad_finish();
    printf("iterations_num: %d\n", iterations_num);*/
    
    uv_queue_work(loop, &current_frame_decoding_req, frame_decoding_process, after_frame_decoding_process);
}

void frame_decoding_process(uv_work_t *req) {
    if (mad_frame_decode(&mad_frame, &mad_stream) == -1) {
        if (!MAD_RECOVERABLE(mad_stream.error)) {
            frame_decoding_finish_status = true;
        }
    }
    if (!frame_decoding_finish_status) {
        mad_synth_frame(&mad_synth, &mad_frame);
    }
}

void after_frame_decoding_process(uv_work_t *req, int status) {
    if (frame_decoding_finish_status) {
        printf("iterations_num: %d\n", iterations_num);
        mad_finish();
    } else {
        output(&mad_frame.header, &mad_synth.pcm);
        iterations_num++;
        uv_queue_work(loop, req, frame_decoding_process, after_frame_decoding_process);
    }
}

static inline signed int scale(mad_fixed_t sample) {
  // round 
  sample += (1L << (MAD_F_FRACBITS - 16));

  // clip
  if (sample >= MAD_F_ONE){
    sample = MAD_F_ONE - 1;
  } else if (sample < -MAD_F_ONE){
    sample = -MAD_F_ONE;
  }

  // quantize 
  return sample >> (MAD_F_FRACBITS + 1 - 16);
}

static 
enum mad_flow output(struct mad_header const *header, struct mad_pcm *pcm) {
  unsigned int nchannels, nsamples;
  mad_fixed_t const *left_ch, *right_ch;

  //pcm->samplerate contains the sampling frequency 

  nchannels = pcm->channels;
  nsamples  = pcm->length;
  left_ch   = pcm->samples[0];
  right_ch  = pcm->samples[1];

  char *stream = new char[1152 * 4 * nchannels];
  //printf("%d\n", nsamples);
  if (nchannels == 2) {
    while (nsamples--) {
        signed int sample;
        sample = scale(*left_ch++);
        stream[(pcm->length-nsamples) * 4] = ((sample >> 0) & 0xff);
        //printf("%d\n", stream[(pcm->length-nsamples) * 4]);
        stream[(pcm->length-nsamples) * 4 + 1] = ((sample >> 8) & 0xff);
        //printf("%d\n", stream[(pcm->length-nsamples) * 4 + 1]);
        sample = scale(*right_ch++);
        stream[(pcm->length-nsamples) * 4 + 2] = ((sample >> 0) & 0xff);
        //printf("%d\n", stream[(pcm->length-nsamples) * 4 + 2]);
        stream[(pcm->length-nsamples) * 4 + 3] = ((sample >> 8) & 0xff);
        //printf("%d\n", stream[(pcm->length-nsamples) * 4 + 3]);
    }
  } else {
    printf("%s\n", "mono");
    while (nsamples--) {
        signed int sample;
        sample = scale(*left_ch++);
        stream[(pcm->length-nsamples) * 4] = ((sample >> 0) & 0xff);
        //printf("%d\n", stream[(pcm->length-nsamples) * 4]);
        stream[(pcm->length-nsamples) * 4 + 1] = ((sample >> 8) & 0xff);
        //printf("%d\n", stream[(pcm->length-nsamples) * 4 + 1]);
    }
  }
  delete [] stream;

  return MAD_FLOW_CONTINUE;
}
