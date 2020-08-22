#include <obs-module.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

struct rs_depth_source {
	uint32_t width;
	uint32_t height;

	obs_source_t *src;
	obs_source_frame2 obs_frame;

	rs2::pipeline pipeline;

	//rs2::decimation_filter decimation_filter;
	rs2::spatial_filter spatial_filter;
	rs2::temporal_filter temporal_filter;
	rs2::hole_filling_filter hole_filling_filter;
	rs2::colorizer color_map;

	std::mutex mutex;
	void OnDepthData(const rs2::frame &frame);

	inline rs_depth_source(obs_source_t *source) : src(source)
	{
		memset(&obs_frame, 0, sizeof(obs_frame));
	}
};

static const char *rs_depth_source_get_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return "RS Depth Source";
}

static void rs_depth_source_update(void *data, obs_data_t *settings)
{
	struct rs_depth_source *context = reinterpret_cast<rs_depth_source *>(data);
	uint32_t width = (uint32_t)obs_data_get_int(settings, "width");
	uint32_t height = (uint32_t)obs_data_get_int(settings, "height");
	context->width = width;
	context->height = height;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, width, height,
					RS2_FORMAT_Z16, 30);

	context->pipeline.start(cfg,
				std::bind(&rs_depth_source::OnDepthData, context,
					  std::placeholders::_1));

	/*context->decimation_filter.set_option(
		RS2_OPTION_FILTER_MAGNITUDE, 2);*/
	context->spatial_filter.set_option(
		RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
	context->spatial_filter.set_option(
		RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
	context->spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,
						2);
	context->temporal_filter.set_option(
		RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
	context->temporal_filter.set_option(
		RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
	context->temporal_filter.set_option(RS2_OPTION_HOLES_FILL, 4);
	context->hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL,
						2);

	context->color_map.set_option(RS2_OPTION_COLOR_SCHEME, 3);
}

static void *rs_depth_source_create(obs_data_t *settings, obs_source_t *source)
{
	rs_depth_source *context = new rs_depth_source(source);

	rs_depth_source_update(context, settings);

	return context;
}

static void rs_depth_source_destroy(void *data)
{
	rs_depth_source *context = reinterpret_cast<rs_depth_source *>(data);
	context->pipeline.stop();
	delete context;
}

static obs_properties_t *rs_depth_source_properties(void *unused)
{
	UNUSED_PARAMETER(unused);
	obs_properties_t *props = obs_properties_create();
	return props;
}

void rs_depth_source::OnDepthData(const rs2::frame &frame)
{
	std::lock_guard<std::mutex> lock(mutex);
	
	
	rs2::depth_frame depth = NULL;
	if (rs2::frameset fs = frame.as<rs2::frameset>()) {
		depth = fs.get_depth_frame()
				.apply_filter(spatial_filter)
				.apply_filter(hole_filling_filter);
	} else {
		// Only process depth frames
		return;
	}


	int width = depth.get_width();
	int height = depth.get_height();

	rs2::frame depth_color = color_map.colorize(depth);
	cv::Mat dMat = cv::Mat(cv::Size(width, height), CV_8UC3,
			       (void *)depth_color.get_data());
	cv::Mat dMat_color = cv::Mat(cv::Size(width, height), CV_8UC4);
	cv::cvtColor(dMat, dMat_color, cv::COLOR_RGB2RGBA);


	obs_frame.data[0] = dMat_color.ptr();
	obs_frame.linesize[0] = width*4;
	obs_frame.format = VIDEO_FORMAT_RGBA;
	obs_frame.width = width;
	obs_frame.height = height;

	
	obs_source_output_video2(src, &obs_frame);
}

static void rs_depth_source_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "width", 848);
	obs_data_set_default_int(settings, "height", 480);
}

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("rs-depth-source", "en-US")
MODULE_EXPORT const char *obs_module_description(void)
{
	return "RS Depth Source";
}

bool obs_module_load(void)
{
	obs_source_info rs_depth_source_info = {};
	rs_depth_source_info.id = "rs_depth_source";
	rs_depth_source_info.type = OBS_SOURCE_TYPE_INPUT;
	rs_depth_source_info.output_flags = OBS_SOURCE_ASYNC_VIDEO;
	rs_depth_source_info.create = rs_depth_source_create;
	rs_depth_source_info.destroy = rs_depth_source_destroy;
	rs_depth_source_info.update = rs_depth_source_update;
	rs_depth_source_info.get_name = rs_depth_source_get_name;
	rs_depth_source_info.get_defaults = rs_depth_source_defaults;
	rs_depth_source_info.get_properties = rs_depth_source_properties;
	rs_depth_source_info.icon_type = OBS_ICON_TYPE_COLOR;

	obs_register_source(&rs_depth_source_info);
	return true;
}
