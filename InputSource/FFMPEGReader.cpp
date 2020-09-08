// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// Modified code: Copyright 2020 Gregory Kramida

// this hack is required on android
#ifdef __ANDROID__
#define __STDC_CONSTANT_MACROS
#define __STDC_LIMIT_MACROS
#endif

#include <stdint.h>

#include "FFMPEGReader.h"

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
}

#include <deque>
#include <iostream>

using namespace InputSource;


namespace InputSource {

class FFMPEGReader::PrivateData {
private:
	typedef struct FilteringContext {
		AVFilterContext* buffer_sink_context;
		AVFilterContext* buffer_source_context;
		AVFilterGraph* filter_graph;
	} FilteringContext;

public:
	PrivateData() : packet() {
		av_log_set_level(AV_LOG_QUIET);
		depth_stream_idx = color_stream_idx = -1;
		av_init_packet(&packet);
		packet.data = nullptr;
		packet.size = 0;
		frame = nullptr;
		format_context = nullptr;
		filtering_context = nullptr;
	}

	~PrivateData() {
		av_packet_unref(&packet);
		av_frame_free(&frame);
		for (int i_stream = 0; i_stream < decoding_contexts.size(); i_stream++) {
			AVCodecContext* codec_context = decoding_contexts[i_stream];
			avcodec_free_context(&codec_context);
		}
	}

	bool Open(const char* filename);
	bool ReadFrames();
	bool Close();

	Vector2i GetDepthImageSize() const {
		if (!ProvidesDepth()) return Vector2i(0, 0);
		AVCodecParameters* decoding_parameters = format_context->streams[depth_stream_idx]->codecpar;
		return Vector2i(decoding_parameters->width, decoding_parameters->height);
	}

	Vector2i GetColorImageSize() const {
		if (!ProvidesColor()) return Vector2i(0, 0);
		AVCodecParameters* decoding_parameters = format_context->streams[color_stream_idx]->codecpar;
		return Vector2i(decoding_parameters->width, decoding_parameters->height);
	}

	bool ProvidesDepth() const { return (depth_stream_idx >= 0); }

	bool HasQueuedDepth() const { return (!depth_frames.empty()); }

	AVFrame* GetFromDepthQueue() {
		if (depth_frames.empty()) return nullptr;
		AVFrame* ret = depth_frames.front();
		depth_frames.pop_front();
		return ret;
	}

	bool ProvidesColor() const { return (color_stream_idx >= 0); }

	bool HasQueuedColor() const { return (!color_frames.empty()); }

	AVFrame* GetFromColorQueue() {
		if (color_frames.size() == 0) return nullptr;
		AVFrame* ret = color_frames.front();
		color_frames.pop_front();
		return ret;
	}

	bool HasMoreImages() {
		//fprintf(stderr, "check: %i %i %i %i\n", ProvidesColor(), HasQueuedColor(), ProvidesDepth(), HasQueuedDepth());
		if (ProvidesColor()) {
			if (!HasQueuedColor()) ReadFrames();
			if (!HasQueuedColor()) return false;
		}
		if (ProvidesDepth()) {
			if (!HasQueuedDepth()) ReadFrames();
			if (!HasQueuedDepth()) return false;
		}
		return true;
	}

	void FlushQueue(bool depth) {
		while (true) {
			AVFrame* tmp;
			if (depth) tmp = GetFromDepthQueue();
			else tmp = GetFromColorQueue();
			if (tmp == nullptr) break;
			av_frame_free(&tmp);
		}
	}

private:
	int OpenInputFile(const char* filename);
	static int
	InitFilter(FilteringContext* fctx, AVCodecParameters* decoding_parameters, AVRational& time_base, const char* filter_spec, bool isDepth);
	int InitFilters();
	int FilterDecodeFrame(AVFrame* frame, int stream_index);
	void FlushDecoderAndFilter();

	AVFormatContext* format_context;
	FilteringContext* filtering_context;

	std::vector<AVCodecContext*> decoding_contexts;

	AVPacket packet;
	AVFrame* frame;

	int depth_stream_idx;
	int color_stream_idx;
	std::deque<AVFrame*> depth_frames;
	std::deque<AVFrame*> color_frames;
};

int FFMPEGReader::PrivateData::OpenInputFile(const char* filename) {
	int ret;
	if ((ret = avformat_open_input(&format_context, filename, nullptr, nullptr)) < 0) {
		std::cerr << "Cannot open input file" << std::endl;
		return ret;
	}
	if ((ret = avformat_find_stream_info(format_context, nullptr)) < 0) {
		std::cerr << "Cannot find stream information" << std::endl;
		return ret;
	}

	decoding_contexts.resize(format_context->nb_streams);

	for (int i_stream = 0; i_stream < format_context->nb_streams; i_stream++) {
		AVStream* stream = format_context->streams[i_stream];
		AVCodecParameters* codec_parameters = stream->codecpar;

		// If codec type is video, open decoder and determine whether the stream fits allowed formats
		// for depth, RGB, or none at all.
		if (codec_parameters->codec_type == AVMEDIA_TYPE_VIDEO) {
			AVCodec* decoder = avcodec_find_decoder(codec_parameters->codec_id);
			if (!decoder) {
				std::cerr << "Cannot find codec by codec_id" << std::endl;
				return -1;
			}
			decoding_contexts[i_stream] = avcodec_alloc_context3(decoder);
			AVCodecContext* decoder_context = decoding_contexts[i_stream];
			if (!decoder_context) {
				std::cerr << "Failed to allocate context" << std::endl;
				return -1;
			}
			if (avcodec_parameters_to_context(decoder_context, stream->codecpar) < 0) {
				std::cerr << "Failed to copy parameters to codec context" << std::endl;
				return -1;
			}
			av_codec_set_pkt_timebase(decoder_context, stream->time_base);
			decoder_context->time_base.den = stream->time_base.den;
			decoder_context->time_base.num = stream->time_base.num;
			ret = avcodec_open2(decoder_context, decoder, nullptr);
			if (ret < 0) {
				std::cerr << "Failed to open decoder for stream #" << i_stream << std::endl;
				continue;
			}
			if (decoder_context->pix_fmt == AV_PIX_FMT_GRAY16LE) depth_stream_idx = i_stream;
			if ((decoder_context->pix_fmt == AV_PIX_FMT_YUV422P) ||
			    (decoder_context->pix_fmt == AV_PIX_FMT_RGB24) ||
			    (decoder_context->pix_fmt == AV_PIX_FMT_RGBA) ||
			    (decoder_context->pix_fmt == AV_PIX_FMT_BGR0))
				color_stream_idx = i_stream;
		}
	}
	return 0;
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"

int FFMPEGReader::PrivateData::InitFilter(FilteringContext* fctx, AVCodecParameters* decoding_parameters, AVRational& time_base,
                                          const char* filter_specification, bool is_depth) {
	char args[512];
	int ret;
	const AVFilter* buffer_source;
	const AVFilter* buffer_sink;
	AVFilterContext* buffer_source_context = nullptr;
	AVFilterContext* buffer_sink_context = nullptr;
	AVFilterInOut* outputs = avfilter_inout_alloc();
	AVFilterInOut* inputs = avfilter_inout_alloc();
	AVFilterGraph* filter_graph = avfilter_graph_alloc();

	// TODO: depending on endianness, requiredOutput should maybe be set to
	//       AV_PIX_FMT_GRAY16BE
	AVPixelFormat required_output = is_depth ? AV_PIX_FMT_GRAY16LE : AV_PIX_FMT_RGBA;

	if (!outputs || !inputs || !filter_graph) {
		ret = AVERROR(ENOMEM);
		goto end;
	}
	buffer_source = avfilter_get_by_name("buffer");
	buffer_sink = avfilter_get_by_name("buffersink");
	if (!buffer_source || !buffer_sink) {
		std::cerr << "filtering source or sink element not found" << std::endl;
		ret = AVERROR_UNKNOWN;
		goto end;
	}
	sprintf(args,
	        "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
	        decoding_parameters->width, decoding_parameters->height, decoding_parameters->format,
	        time_base.num, time_base.den,
	        decoding_parameters->sample_aspect_ratio.num,
	        decoding_parameters->sample_aspect_ratio.den);
	ret = avfilter_graph_create_filter(&buffer_source_context, buffer_source, "in", args, nullptr, filter_graph);
	if (ret < 0) {
		std::cerr << "Cannot create buffer source" << std::endl;
		goto end;
	}
	ret = avfilter_graph_create_filter(&buffer_sink_context, buffer_sink, "out", nullptr, nullptr, filter_graph);
	if (ret < 0) {
		std::cerr << "Cannot create buffer sink" << std::endl;
		goto end;
	}

	ret = av_opt_set_bin(buffer_sink_context, "pix_fmts", (uint8_t*) &required_output, sizeof(required_output),
	                     AV_OPT_SEARCH_CHILDREN);
	if (ret < 0) {
		std::cerr << "Cannot set output pixel format" << std::endl;
		goto end;
	}
	/* Endpoints for the filter graph. */
	outputs->name = av_strdup("in");
	outputs->filter_ctx = buffer_source_context;
	outputs->pad_idx = 0;
	outputs->next = nullptr;
	inputs->name = av_strdup("out");
	inputs->filter_ctx = buffer_sink_context;
	inputs->pad_idx = 0;
	inputs->next = nullptr;
	if (!outputs->name || !inputs->name) {
		ret = AVERROR(ENOMEM);
		goto end;
	}
	if ((ret = avfilter_graph_parse_ptr(filter_graph, filter_specification, &inputs, &outputs, nullptr)) < 0) goto end;
	if ((ret = avfilter_graph_config(filter_graph, nullptr)) < 0) goto end;

	/* Fill FilteringContext */
	fctx->buffer_source_context = buffer_source_context;
	fctx->buffer_sink_context = buffer_sink_context;
	fctx->filter_graph = filter_graph;
	end:
	avfilter_inout_free(&inputs);
	avfilter_inout_free(&outputs);
	return ret;
}

#pragma clang diagnostic pop

int FFMPEGReader::PrivateData::InitFilters() {
	filtering_context = (FilteringContext*) av_malloc_array(format_context->nb_streams, sizeof(*filtering_context));
	if (!filtering_context) return AVERROR(ENOMEM);
	for (int i_stream = 0; (unsigned int) i_stream < format_context->nb_streams; i_stream++) {
		filtering_context[i_stream].buffer_source_context = nullptr;
		filtering_context[i_stream].buffer_sink_context = nullptr;
		filtering_context[i_stream].filter_graph = nullptr;
		if ((i_stream != depth_stream_idx) && (i_stream != color_stream_idx)) continue;
		const char* filter_specification = "null"; /* passthrough (dummy) filter for video */
		int ret = InitFilter(&filtering_context[i_stream], format_context->streams[i_stream]->codecpar,
		                     format_context->streams[i_stream]->time_base, filter_specification,
		                     (i_stream == depth_stream_idx));
		if (ret) return ret;
	}
	return 0;
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"

int FFMPEGReader::PrivateData::FilterDecodeFrame(AVFrame* frame, int stream_index) {
	int ret;

	/* push the decoded frame into the filtergraph */
	ret = av_buffersrc_add_frame_flags(filtering_context[stream_index].buffer_source_context, frame, 0);
	if (ret < 0) {
		std::cerr << "Error while feeding the filtergraph" << std::endl;
		return ret;
	}

	/* pull filtered frames from the filtergraph */
	while (true) {
		AVFrame* filt_frame = av_frame_alloc();
		if (!filt_frame) {
			ret = AVERROR(ENOMEM);
			break;
		}
		ret = av_buffersink_get_frame(filtering_context[stream_index].buffer_sink_context, filt_frame);
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

		if (stream_index == depth_stream_idx) depth_frames.push_back(filt_frame);
		else if (stream_index == color_stream_idx) color_frames.push_back(filt_frame);
		else av_frame_free(&filt_frame);
	}
	return ret;
}

bool FFMPEGReader::PrivateData::Open(const char* filename) {
	av_register_all();
	avfilter_register_all();
	if (OpenInputFile(filename) < 0) return false;
	if (InitFilters() < 0) return false;
	return true;
}


void FFMPEGReader::PrivateData::FlushDecoderAndFilter() {
	AVPacket flush_packet;
	av_init_packet(&flush_packet);
	flush_packet.data = nullptr;
	flush_packet.size = 0;
	int ret;
	bool error_encountered = false;

	/* flush filters and decoders */
	for (int i_stream = 0; (unsigned int) i_stream < format_context->nb_streams && !error_encountered; i_stream++) {
		if ((i_stream != color_stream_idx) && (i_stream != depth_stream_idx)) continue;

		if (filtering_context[i_stream].filter_graph == nullptr) continue;

		ret = avcodec_send_packet(this->decoding_contexts[i_stream], &flush_packet);
		if (ret < 0) {
			std::cerr << "Flushing filter failed" << std::endl;
			break;
		}

		/* flush decoder */
		while (ret >= 0) {
			ret = avcodec_send_packet(decoding_contexts[i_stream], &flush_packet);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
				break;
			} else if (ret < 0) {
				av_frame_free(&frame);
				if (ret < 0) std::cerr << "Decoding failed" << std::endl;
				error_encountered = true;
				break;
			}
			frame->pts = av_frame_get_best_effort_timestamp(frame);
			ret = FilterDecodeFrame(frame, i_stream);
			av_frame_free(&frame);
		}

		/* flush filter */
		ret = FilterDecodeFrame(nullptr, i_stream);
		if (ret < 0) {
			std::cerr << "Flushing filter failed" << std::endl;
			break;
		}
	}
}

bool FFMPEGReader::PrivateData::ReadFrames() {
	int ret;
	int i_stream;

	while (true) {
		// do we have to read more images?
		bool wait_for_color = ProvidesColor() && (!HasQueuedColor());
		bool wait_for_depth = ProvidesDepth() && (!HasQueuedDepth());
		if ((!wait_for_color) && (!wait_for_depth)) return false;

		// read packets
		if ((ret = av_read_frame(format_context, &packet)) < 0) {
			FlushDecoderAndFilter();
			break;
		}
		i_stream = packet.stream_index;
		if ((i_stream != color_stream_idx) && (i_stream != depth_stream_idx)) continue;
		if (filtering_context[i_stream].filter_graph) {
			frame = av_frame_alloc();
			if (!frame) {
				ret = AVERROR(ENOMEM);
				break;
			}
			av_packet_rescale_ts(&packet,
			                     format_context->streams[i_stream]->time_base,
			                     decoding_contexts[i_stream]->time_base);

			ret = avcodec_send_packet(decoding_contexts[i_stream], &packet);
			if (ret < 0) {
				av_frame_free(&frame);
				std::cerr << "Decoding failed" << std::endl;
				break;
			} else {
				ret = avcodec_receive_frame(decoding_contexts[i_stream], frame);
				if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
					av_frame_free(&frame);
					break;
				} else {
					frame->pts = av_frame_get_best_effort_timestamp(frame);
					ret = FilterDecodeFrame(frame, i_stream);
					av_frame_free(&frame);
					//if (ret < 0) goto end;
					break;
				}
			}
		}
		av_packet_unref(&packet);
	}

	return (ret == 0);
}

#pragma clang diagnostic pop

bool FFMPEGReader::PrivateData::Close() {
	if (format_context != nullptr) {
		for (unsigned int i_stream = 0; i_stream < format_context->nb_streams; i_stream++) {
			avcodec_close(decoding_contexts[i_stream]);
			if (filtering_context && filtering_context[i_stream].filter_graph) avfilter_graph_free(&filtering_context[i_stream].filter_graph);
		}
		avformat_close_input(&format_context);
	}
	if (filtering_context != nullptr) av_free(filtering_context);
	format_context = nullptr;
	filtering_context = nullptr;
	return true;
}
} // namespace InputSource

FFMPEGReader::FFMPEGReader(
		const char* calibFilename,
		const char* filename1,
		const char* filename2)
		: BaseImageSourceEngine(calibFilename) {
	mData1 = new PrivateData();
	is_valid = mData1->Open(filename1);

	if (is_valid && (filename2 != nullptr)) {
		mData2 = new PrivateData();
		mData2->Open(filename2);
	} else {
		mData2 = nullptr;
	}
}

FFMPEGReader::~FFMPEGReader() {
	if (is_valid) mData1->Close();
	delete mData1;
	if (mData2 != nullptr) {
		if (is_valid) mData2->Close();
		delete mData2;
	}
}

bool FFMPEGReader::HasMoreImages() const {
	if (!is_valid) return false;
	if (!mData1->HasMoreImages()) return false;
	if (mData2 != nullptr) if (!mData2->HasMoreImages()) return false;
	return true;
}

static void CopyRGBA(const AVFrame* frame, Vector4u* rgb) {
	for (int y = 0; y < frame->height; ++y)
		for (int x = 0; x < frame->width; ++x) {
			Vector4u tmp;
			tmp.x = frame->data[0][x * 4 + frame->linesize[0] * y + 0];
			tmp.y = frame->data[0][x * 4 + frame->linesize[0] * y + 1];
			tmp.z = frame->data[0][x * 4 + frame->linesize[0] * y + 2];
			tmp.w = frame->data[0][x * 4 + frame->linesize[0] * y + 3];
			rgb[x + y * frame->width] = tmp;
		}
}

static void CopyDepth(const AVFrame* frame, short* depth) {
	memcpy(depth, frame->data[0], frame->height * frame->width * 2);
}

void FFMPEGReader::GetImages(UChar4Image& rgb_image, ShortImage& depth_image) {
	Vector4u* rgb = rgb_image.GetData(MEMORYDEVICE_CPU);
	short* depth = depth_image.GetData(MEMORYDEVICE_CPU);

	bool has_color = false;
	bool has_depth = false;
	if (is_valid) {
		if (mData1->ProvidesColor()) {
			if (mData2 != nullptr) mData2->FlushQueue(false);

			if (!mData1->HasQueuedColor()) mData1->ReadFrames();
			if (mData1->HasQueuedColor()) {
				AVFrame* frame = mData1->GetFromColorQueue();
				CopyRGBA(frame, rgb);
				av_frame_free(&frame);

				has_color = true;
			}
		} else if (mData2 != nullptr)
			if (mData2->ProvidesColor()) {
				if (!mData2->HasQueuedColor()) mData2->ReadFrames();
				if (mData2->HasQueuedColor()) {
					AVFrame* frame = mData2->GetFromColorQueue();
					CopyRGBA(frame, rgb);
					av_frame_free(&frame);

					has_color = true;
				}
			}

		if (mData1->ProvidesDepth()) {
			if (mData2 != nullptr) mData2->FlushQueue(true);

			if (!mData1->HasQueuedDepth()) mData1->ReadFrames();
			if (mData1->HasQueuedDepth()) {
				AVFrame* frame = mData1->GetFromDepthQueue();
				CopyDepth(frame, depth);
				av_frame_free(&frame);

				has_depth = true;
			}
		} else if (mData2 != nullptr)
			if (mData2->ProvidesDepth()) {
				if (!mData2->HasQueuedDepth()) mData2->ReadFrames();
				if (mData2->HasQueuedDepth()) {
					AVFrame* frame = mData2->GetFromDepthQueue();
					CopyDepth(frame, depth);
					av_frame_free(&frame);

					has_depth = true;
				}
			}
	}
	if (!has_color) memset(rgb, 0, rgb_image.size() * sizeof(Vector4u));
	if (!has_depth) memset(depth, 0, depth_image.size() * sizeof(short));
}

Vector2i FFMPEGReader::GetDepthImageSize() const {
	if (mData1->ProvidesDepth()) return mData1->GetDepthImageSize();
	if (mData2 != nullptr) if (mData2->ProvidesDepth()) return mData2->GetDepthImageSize();
	return Vector2i(0, 0);
}

Vector2i FFMPEGReader::GetRGBImageSize() const {
	if (mData1->ProvidesColor()) return mData1->GetColorImageSize();
	if (mData2 != nullptr) if (mData2->ProvidesColor()) return mData2->GetColorImageSize();
	return Vector2i(0, 0);
}

#else

using namespace InputSource;

FFMPEGReader::FFMPEGReader(const char *calibFilename, const char *filename1, const char *filename2)
	: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without FFMPEG\n");
}
FFMPEGReader::~FFMPEGReader()
{}
void FFMPEGReader::GetImages(UChar4Image& rgbImage, ShortImage& rawDepthImage)
{ return; }
bool FFMPEGReader::HasMoreImages() const
{ return false; }
Vector2i FFMPEGReader::GetDepthImageSize() const
{ return Vector2i(0,0); }
Vector2i FFMPEGReader::GetRGBImageSize() const
{ return Vector2i(0,0); }

#endif

