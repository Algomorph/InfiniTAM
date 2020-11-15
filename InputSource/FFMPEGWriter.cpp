// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// Modified code: Copyright 2020 Gregory Kramida

// this hack is required on android
#ifdef __ANDROID__
#define __STDC_CONSTANT_MACROS
#define __STDC_LIMIT_MACROS
#endif

#include <cstdint>

#include "FFMPEGWriter.h"

// If we're using a version of Visual Studio prior to 2015, snprintf isn't supported, so fall back to the non-standard _snprintf instead.
#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#endif

#ifdef COMPILE_WITH_FFMPEG

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/imgutils.h>
}

#include <iostream>

using namespace InputSource;

#ifdef av_err2str
#undef av_err2str
av_always_inline char* av_err2str(int errnum)
{
	static char str[AV_ERROR_MAX_STRING_SIZE];
	memset(str, 0, sizeof(str));
	return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#endif

class FFMPEGWriter::PrivateData {
public:
	int open(const char* filename, int size_x, int size_y, bool is_depth, int fps);
	int init_filters();
	int encode_write_frame(AVFrame* filt_frame, unsigned int stream_index, int* got_frame);
	int filter_encode_write_frame(AVFrame* frame, unsigned int stream_index);
	int flush_encoder(unsigned int stream_index);
	int close();

	AVFrame* getFrame() { return frame; }

private:
	typedef struct FilteringContext {
		AVFilterContext* buffersink_ctx;
		AVFilterContext* buffersrc_ctx;
		AVFilterGraph* filter_graph;
	} FilteringContext;

	void allocFrame(bool isDepth);
	void freeFrame();
	static int init_filter(FilteringContext* fctx, AVCodecContext* enc_ctx, const char* filter_spec);


	AVFormatContext* output_format_context;
	AVCodecContext* encoder_context;
	AVFrame* frame;

	FilteringContext filter_ctx;
};

#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"

int FFMPEGWriter::PrivateData::open(const char* filename, int size_x, int size_y, bool isDepth, int fps) {
	printf("saving to video file: %s\n", filename);


	output_format_context = nullptr;
	avformat_alloc_output_context2(&output_format_context, nullptr, nullptr, filename);
	if (!output_format_context) {
		std::cerr << "Could not create ffmpeg output context" << std::endl;
		return -1;
	}

	AVCodec* encoder = avcodec_find_encoder(AV_CODEC_ID_FFV1);
	if (!encoder) {
		std::cerr << "Necessary encoder not found in ffmpeg" << std::endl;
		return -1;
	}

	AVStream* out_stream = avformat_new_stream(output_format_context, encoder);

	if (!out_stream) {
		std::cerr << "Failed allocating ffmpeg output stream" << std::endl;
		return -1;
	}

	encoder_context = avcodec_alloc_context3(encoder);

	if (!encoder_context) {
		std::cerr << "Failed to allocate context" << std::endl;
		return -1;
	}
	AVRational framerate = {1, fps};

	encoder_context->codec_type = AVMEDIA_TYPE_VIDEO;
	encoder_context->width = size_x;
	encoder_context->height = size_y;
	encoder_context->codec_id = AV_CODEC_ID_FFV1;
	encoder_context->time_base = framerate;
	encoder_context->gop_size = 12;

	encoder_context->sample_aspect_ratio.num = 1;
	encoder_context->sample_aspect_ratio.den = 1;
	encoder_context->pix_fmt = isDepth ? AV_PIX_FMT_GRAY16LE : AV_PIX_FMT_YUV422P;

	out_stream->time_base = framerate;
	out_stream->avg_frame_rate = framerate;

	/* Third parameter can be used to pass settings to encoder */
	AVDictionary* dict = nullptr;
	av_dict_set(&dict, "coder", "1", 0);
	av_dict_set(&dict, "context", "0", 0);
	av_dict_set(&dict, "level", "3", 0);
	av_dict_set(&dict, "threads", "8", 0);
	av_dict_set(&dict, "slices", "16", 0);

	int ret = avcodec_open2(encoder_context, encoder, &dict);

	AVCodecParameters* encoding_parameters = out_stream->codecpar;
	if (avcodec_parameters_from_context(encoding_parameters, encoder_context) < 0) {
		std::cerr << "Failed to copy codec context to parameters." << std::endl;
		return -1;
	}

	av_dict_free(&dict);
	if (ret < 0) {
		std::cerr << "Cannot open video encoder for stream" << std::endl;
		return ret;
	}

	if (output_format_context->oformat->flags & AVFMT_GLOBALHEADER) {
		encoder_context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	}

	if (!(output_format_context->oformat->flags & AVFMT_NOFILE)) {
		ret = avio_open(&output_format_context->pb, filename, AVIO_FLAG_WRITE);
		if (ret < 0) {
			std::cerr << "Could not open output file '" << filename << "'" << std::endl;
			return ret;
		}
	}

	output_format_context->video_codec = encoder;

	if ( output_format_context->oformat->flags & AVFMT_GLOBALHEADER )
		output_format_context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

	ret = avformat_write_header(output_format_context, &dict);
	if (ret < 0) {
		std::cerr << "Error occurred when opening output file" << std::endl;
		return ret;
	}

	allocFrame(isDepth);

	return 0;
}

int FFMPEGWriter::PrivateData::init_filter(FilteringContext* fctx, AVCodecContext* enc_ctx, const char* filter_spec) {
	char args[512];
	int ret;
	const AVFilter* buffersrc;
	const AVFilter* buffersink;
	AVFilterContext* buffersrc_ctx = nullptr;
	AVFilterContext* buffersink_ctx = nullptr;
	AVFilterInOut* outputs = avfilter_inout_alloc();
	AVFilterInOut* inputs = avfilter_inout_alloc();
	AVFilterGraph* filter_graph = avfilter_graph_alloc();
	if (!outputs || !inputs || !filter_graph) {
		ret = AVERROR(ENOMEM);
		goto end;
	}

	buffersrc = avfilter_get_by_name("buffer");
	buffersink = avfilter_get_by_name("buffersink");
	if (!buffersrc || !buffersink) {
		std::cerr << "filtering source or sink element not found" << std::endl;
		ret = AVERROR_UNKNOWN;
		goto end;
	}
	sprintf(args,
	        "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
			// TODO: depending on host endianness, the desired format should
			//       maybe be set to AV_PIX_FMT_GRAY16BE
			enc_ctx->width, enc_ctx->height,
			(enc_ctx->pix_fmt == AV_PIX_FMT_GRAY16LE) ? AV_PIX_FMT_GRAY16LE : AV_PIX_FMT_RGBA /*RGB24*/ /*AV_PIX_FMT_RGBA */,
			enc_ctx->time_base.num, enc_ctx->time_base.den,
			enc_ctx->sample_aspect_ratio.num,
			enc_ctx->sample_aspect_ratio.den);
	ret = avfilter_graph_create_filter(&buffersrc_ctx, buffersrc, "in", args, nullptr, filter_graph);
	if (ret < 0) {
		std::cerr << "Cannot create buffer source" << std::endl;
		goto end;
	}
	ret = avfilter_graph_create_filter(&buffersink_ctx, buffersink, "out", nullptr, nullptr, filter_graph);
	if (ret < 0) {
		std::cerr << "Cannot create buffer sink" << std::endl;
		goto end;
	}
	ret = av_opt_set_bin(buffersink_ctx, "pix_fmts",
	                     (uint8_t*) &enc_ctx->pix_fmt, sizeof(enc_ctx->pix_fmt),
	                     AV_OPT_SEARCH_CHILDREN);
	if (ret < 0) {
		std::cerr << "Cannot set output pixel format" << std::endl;
		goto end;
	}

	/* Endpoints for the filter graph. */
	outputs->name = av_strdup("in");
	outputs->filter_ctx = buffersrc_ctx;
	outputs->pad_idx = 0;
	outputs->next = nullptr;
	inputs->name = av_strdup("out");
	inputs->filter_ctx = buffersink_ctx;
	inputs->pad_idx = 0;
	inputs->next = nullptr;
	if (!outputs->name || !inputs->name) {
		ret = AVERROR(ENOMEM);
		goto end;
	}
	if ((ret = avfilter_graph_parse_ptr(filter_graph, filter_spec, &inputs, &outputs, nullptr)) < 0) goto end;
	if ((ret = avfilter_graph_config(filter_graph, nullptr)) < 0) goto end;

	/* Fill FilteringContext */
	fctx->buffersrc_ctx = buffersrc_ctx;
	fctx->buffersink_ctx = buffersink_ctx;
	fctx->filter_graph = filter_graph;
	end:
	avfilter_inout_free(&inputs);
	avfilter_inout_free(&outputs);
	return ret;
}

int FFMPEGWriter::PrivateData::init_filters() {
	const char* filter_spec;
	int ret;
	filter_ctx.buffersrc_ctx = nullptr;
	filter_ctx.buffersink_ctx = nullptr;
	filter_ctx.filter_graph = nullptr;
	filter_spec = "null";
	ret = init_filter(&filter_ctx, encoder_context, filter_spec);
	return ret;
}

int FFMPEGWriter::PrivateData::encode_write_frame(AVFrame* filt_frame, unsigned int stream_index, int* got_frame) {
	int ret;
	int got_frame_local;
	AVPacket enc_pkt;
	if (!got_frame) got_frame = &got_frame_local;

	/* encode filtered frame */
	enc_pkt.data = nullptr;
	enc_pkt.size = 0;
	av_init_packet(&enc_pkt);

	//ret = avcodec_encode_video2(encoder_context, &enc_pkt, filt_frame, got_frame);
	ret = avcodec_send_frame(encoder_context, filt_frame);
	if (ret < 0) {
		return ret;
	}

	ret = avcodec_receive_packet(encoder_context, &enc_pkt);
	if (ret == 0) {
		*got_frame = 1;
	}
	if (ret == AVERROR(EAGAIN)) {
		ret = 0;
	}

	av_frame_free(&filt_frame);

	if (ret < 0) return ret;

	if (!(*got_frame)) return 0;

	/* prepare packet for muxing */
	enc_pkt.stream_index = static_cast<int>(stream_index);
	av_packet_rescale_ts(&enc_pkt,
	                     encoder_context->time_base,
	                     output_format_context->streams[stream_index]->time_base);

	/* mux encoded frame */
	ret = av_interleaved_write_frame(output_format_context, &enc_pkt);
	return ret;
}

int FFMPEGWriter::PrivateData::filter_encode_write_frame(AVFrame* frame, unsigned int stream_index) {
	int ret;
	AVFrame* filt_frame;

	// filter
	ret = av_buffersrc_add_frame_flags(filter_ctx.buffersrc_ctx, frame, 0);
	if (ret < 0) {
		std::cerr << "Error while feeding the filtergraph" << std::endl;
		return ret;
	}

	// write frames
	while (true) {
		filt_frame = av_frame_alloc();
		if (!filt_frame) {
			ret = AVERROR(ENOMEM);
			break;
		}
		ret = av_buffersink_get_frame(filter_ctx.buffersink_ctx, filt_frame);
		if (ret < 0) {
			/* if no more frames for output - returns AVERROR(EAGAIN)
			 * if flushed and no more frames for output - returns AVERROR_EOF
			 * rewrite retcode to 0 to show it as normal procedure completion
			*/
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) ret = 0;
			av_frame_free(&filt_frame);
			break;
		}
		filt_frame->pict_type = AV_PICTURE_TYPE_NONE;
		ret = encode_write_frame(filt_frame, stream_index, nullptr);
		if (ret < 0) break;
	}
	return ret;
}

int FFMPEGWriter::PrivateData::flush_encoder(unsigned int stream_index) {
	int ret;
	int got_frame;
	if (!(encoder_context->codec->capabilities & AV_CODEC_CAP_DELAY)) return 0;

	while (true) {
		ret = encode_write_frame(nullptr, stream_index, &got_frame);
		if (ret < 0) break;
		if (!got_frame) return 0;
	}
	return ret;
}




int FFMPEGWriter::PrivateData::close() {
	int ret = 0;
	/* flush filter */
	bool can_write_trailer = true;

	if (filter_ctx.filter_graph != nullptr) {
		ret = filter_encode_write_frame(nullptr, 0);
		if (ret < 0 && ret != AVERROR_EOF) {
			std::cerr << "Flushing filter failed" << std::endl;
			can_write_trailer = false;
		} else {
			/* flush encoder */
			ret = flush_encoder(0);
			if (ret < 0 && ret != AVERROR_EOF) {
				std::cerr << "Flushing encoder failed" << std::endl;
				can_write_trailer = false;
			}
		}
	}
	if (can_write_trailer) {
		av_write_trailer(output_format_context);
	}

	avcodec_close(encoder_context);
	if (filter_ctx.filter_graph) {
		avfilter_graph_free(&filter_ctx.filter_graph);
	}
	if (output_format_context && !(output_format_context->oformat->flags & AVFMT_NOFILE))
		avio_closep(&output_format_context->pb);
	avformat_free_context(output_format_context);
	if(ret == AVERROR_EOF) ret = 0;
	if (ret < 0){
		std::cerr << "Error occurred: " << std::string(av_err2str(ret)) << std::endl;
	}

	freeFrame();

	return ret;
}

#pragma clang diagnostic pop

void FFMPEGWriter::PrivateData::allocFrame(bool isDepth) {
	AVCodecParameters* enc_ctx = output_format_context->streams[0]->codecpar;


	frame = av_frame_alloc();
	if (!frame) {
		std::cerr << "Could not allocate video frame" << std::endl;
		return;
	}
	frame->format = isDepth ? AV_PIX_FMT_GRAY16LE : AV_PIX_FMT_RGBA;
	frame->width = enc_ctx->width;
	frame->height = enc_ctx->height;

	int ret = av_image_alloc(frame->data, frame->linesize, frame->width, frame->height,
	                     (AVPixelFormat) frame->format, 32);

	if (ret < 0) {
		fprintf(stderr, "Could not allocate raw picture buffer\n");
		return;
	}
}

void FFMPEGWriter::PrivateData::freeFrame() {
	av_freep(&frame->data[0]);
	av_frame_free(&frame);
}


FFMPEGWriter::FFMPEGWriter() {
	mData = new PrivateData();
	counter = -1;
}

FFMPEGWriter::~FFMPEGWriter() {
	close();

	delete mData;
}

bool FFMPEGWriter::open(const char* filename, int size_x, int size_y, bool isDepth, int fps) {
	if (isOpen()) return false;

	av_register_all();
	avfilter_register_all();
	if (mData->open(filename, size_x, size_y, isDepth, fps) < 0) return false;
	if (mData->init_filters() < 0) return false;

	counter = 0;
	return true;
}


bool FFMPEGWriter::writeFrame(UChar4Image* rgbImage) {
	if (!isOpen()) return false;

	AVFrame* frame = mData->getFrame();

	if ((frame->format != AV_PIX_FMT_RGBA) || (frame->width != rgbImage->dimensions.x) || (frame->height != rgbImage->dimensions.y)) {
		std::cerr << "FFMPEGWriter: wrong image format for rgb stream" << std::endl;
	}

	Vector4u* rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	for (int y = 0; y < frame->height; ++y)
		for (int x = 0; x < frame->width; ++x) {
			frame->data[0][x * 4 + y * frame->linesize[0] + 0] = rgb[x + y * rgbImage->dimensions.x].x;
			frame->data[0][x * 4 + y * frame->linesize[0] + 1] = rgb[x + y * rgbImage->dimensions.x].y;
			frame->data[0][x * 4 + y * frame->linesize[0] + 2] = rgb[x + y * rgbImage->dimensions.x].z;
			frame->data[0][x * 4 + y * frame->linesize[0] + 3] = rgb[x + y * rgbImage->dimensions.x].w;
		}

	frame->pts = counter++;
	int ret = mData->filter_encode_write_frame(frame, /*stream_index*/0);
	return (ret >= 0);
}

bool FFMPEGWriter::writeFrame(ShortImage* depthImage) {
	if (!isOpen()) return false;

	AVFrame* frame = mData->getFrame();

	if ((frame->format != AV_PIX_FMT_GRAY16LE) || (frame->width != depthImage->dimensions.x) || (frame->height != depthImage->dimensions.y)) {
		std::cerr << "FFMPEGWriter: wrong image format for depth stream" << std::endl;
	}

	short* depth = depthImage->GetData(MEMORYDEVICE_CPU);
	unsigned char* tmp = frame->data[0];
	frame->data[0] = (unsigned char*) depth;

	frame->pts = counter++;
	int ret = mData->filter_encode_write_frame(frame, /*stream_index*/0);
	frame->data[0] = tmp;
	return (ret >= 0);
}

bool FFMPEGWriter::close() {
	if (!isOpen()) return false;

	counter = -1;
	int ret = mData->close();
	return (ret >= 0);
}

bool FFMPEGWriter::isOpen() const {
	return (counter >= 0);
}

#else

using namespace InputSource;

FFMPEGWriter::FFMPEGWriter()
{}
FFMPEGWriter::~FFMPEGWriter()
{}
bool FFMPEGWriter::open(const char *filename, int size_x, int size_y, bool isDepth, int fps)
{ printf("compiled without FFMPEG\n"); return false; }
bool FFMPEGWriter::writeFrame(UChar4Image *rgbImage)
{ return false; }
bool FFMPEGWriter::writeFrame(ShortImage *depthImage)
{ return false; }
bool FFMPEGWriter::close()
{ return false; }
bool FFMPEGWriter::isOpen() const
{ return false; }

#endif

