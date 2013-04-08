#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <ImageAnalysis/ImageAnalysisConfig.h>

typedef ImageAnalysis::ImageAnalysisConfig Config;

#define FALSE	0
#define TRUE	(!FALSE)

namespace enc = sensor_msgs::image_encodings;


// ImageAnalysis
//
// This ROS node subscribes to the specified image topic (default image_raw), and publishes
// a modified image on the topic image_analysis.  The published image can have the contrast
// autocorrected, and some various pixel values displayed.  Pixel values from a FLIR camera
// that are in units of 10 millikelvin can be converted from Kelvin to Celsius.
//

class CImageAnalysis
{
private:
	ros::NodeHandle					m_node;
	image_transport::ImageTransport	m_imagetransport;
	image_transport::Subscriber		m_subImageTopic;
	image_transport::Publisher		m_pubImageAnalysis;
	Config							m_config;

	int m_changedTopic;
	int m_changedContrastAuto;
	int m_changedContrastRange;
	int m_changedAnnotateMinMax;
	int m_changedx;
	int m_changedy;
	int m_changedAnnotateXY;
	int m_changedConvertKtoCforFLIR;

	void Imagetopic_callback(const sensor_msgs::ImageConstPtr& image);
	void ros_reconfigure_callback(Config &config, uint32_t level);

public:
	CImageAnalysis();
	~CImageAnalysis();
	void Main(void);

}; // class CImageAnalysis


CImageAnalysis::~CImageAnalysis()
{
}

CImageAnalysis::CImageAnalysis() : m_imagetransport(m_node)
{
	m_pubImageAnalysis = m_imagetransport.advertise("image_analysis", 1);

}

void CImageAnalysis::ros_reconfigure_callback(Config &config, uint32_t level)
{
	m_changedTopic 				= (m_config.Topic != config.Topic);
	m_changedContrastAuto 		= (m_config.ContrastAuto != config.ContrastAuto);
	m_changedContrastRange 		= (m_config.ContrastRange != config.ContrastRange);
	m_changedAnnotateMinMax 	= (m_config.AnnotateMinMax != config.AnnotateMinMax);
	m_changedAnnotateXY 		= (m_config.AnnotateXY != config.AnnotateXY);
	m_changedx 					= (m_config.x != config.x);
	m_changedy 					= (m_config.y != config.y);
	m_changedConvertKtoCforFLIR = (m_config.ConvertKtoCforFLIR != config.ConvertKtoCforFLIR);

	if (m_changedContrastAuto && config.ContrastAuto)
		config.ContrastRange = FALSE;

	if (m_changedContrastRange && config.ContrastRange)
		config.ContrastAuto = FALSE;

	if (m_changedTopic)
		m_subImageTopic = m_imagetransport.subscribe(config.Topic, 1, &CImageAnalysis::Imagetopic_callback, this);

    m_config = config;
} // ros_reconfigure_callback()



void CImageAnalysis::Imagetopic_callback(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr	cv_ptr;
	double 					valMin, valMax, valXY;
	double					valMinScaled, valMaxScaled, valXYscaled;
	cv::Point				locMin, locMax, locXY;
	unsigned				minPixel, maxPixel;
	char 					buff[100];
	std::string				stLo, stHi, stXY;
	double					scaleText = 1.0;
	unsigned				colorMin, colorMax, colorXY;


	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, image->encoding);//enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Get min,max pixel values.
	minMaxLoc(cv_ptr->image, &valMin, &valMax, &locMin, &locMax);
	maxPixel = (1<<(cv_ptr->image.elemSize()*8))-1;
	minPixel = 0;

	// Get value at (x,y).
	locXY.x = m_config.x;
	locXY.y = m_config.y;
	switch(cv_ptr->image.elemSize())
	{
	case 1:
		valXY = cv_ptr->image.at<char>(locXY.y, locXY.x);
		break;
	case 2:
		valXY = cv_ptr->image.at<short>(locXY.y, locXY.x);
		break;
	case 4:
		valXY = cv_ptr->image.at<int>(locXY.y, locXY.x);
		break;
	}


	// Text for the pixel values.
	if (m_config.ConvertKtoCforFLIR)
	{
		sprintf(buff, "%0.2f", (valMin/100.0-273.15)); // Assumes that pixel values are in units of 10 millikelvin.
		stLo = buff;
		sprintf(buff, "%0.2f", (valMax/100.0-273.15));
		stHi = buff;
		sprintf(buff, "%0.2f", (valXY/100.0-273.15));
		stXY = buff;
	}
	else
	{
		sprintf(buff, "%0.0f", valMin);
		stLo = buff;
		sprintf(buff, "%0.0f", valMax);
		stHi = buff;
		sprintf(buff, "%0.0f", valXY);
		stXY = buff;
	}


	// Rerange the image.
	if (m_config.ContrastAuto)
		cv_ptr->image = (cv_ptr->image - valMin) * maxPixel / (valMax-valMin);

	// Rerange the image.
	if (m_config.ContrastRange)
		cv_ptr->image = (cv_ptr->image - m_config.ContrastLo) * maxPixel / (m_config.ContrastHi - m_config.ContrastLo);


	// Get pixel values after scaling & text colors.
	switch(cv_ptr->image.elemSize())
	{
	case 1:
		valXYscaled = cv_ptr->image.at<char>(locXY.y, locXY.x);
		valMinScaled = cv_ptr->image.at<char>(locMin.y, locMin.x);
		valMaxScaled = cv_ptr->image.at<char>(locMax.y, locMax.x);
		break;
	case 2:
		valXYscaled = cv_ptr->image.at<short>(locXY.y, locXY.x);
		valMinScaled = cv_ptr->image.at<short>(locMin.y, locMin.x);
		valMaxScaled = cv_ptr->image.at<short>(locMax.y, locMax.x);
		break;
	case 4:
		valXYscaled = cv_ptr->image.at<int>(locXY.y, locXY.x);
		valMinScaled = cv_ptr->image.at<int>(locMin.y, locMin.x);
		valMaxScaled = cv_ptr->image.at<int>(locMax.y, locMax.x);
		break;
	}
	colorXY = ((unsigned)valXYscaled>(maxPixel/2)) ? minPixel : maxPixel;
	colorMin = ((unsigned)valMinScaled>(maxPixel/2)) ? minPixel : maxPixel;
	colorMax = ((unsigned)valMaxScaled>(maxPixel/2)) ? minPixel : maxPixel;



	// Draw text indicating the min,max pixel values.
	if (m_config.AnnotateMinMax)
	{
		locMin.y += 10;
		locMax.y += 10;
		putText(cv_ptr->image, stLo, locMin, cv::FONT_HERSHEY_PLAIN, scaleText, cv::Scalar::all(colorMin));
		putText(cv_ptr->image, stHi, locMax, cv::FONT_HERSHEY_PLAIN, scaleText, cv::Scalar::all(colorMax));
		locMin.y -= 10;
		locMax.y -= 10;

		// Draw dots on min,max pixels.
		switch(cv_ptr->image.elemSize())
		{
		case 1:
			cv_ptr->image.at<char>(locMin.y, locMin.x) = colorMin;
			cv_ptr->image.at<char>(locMax.y, locMax.x) = colorMax;
			break;
		case 2:
			cv_ptr->image.at<short>(locMin.y, locMin.x) = colorMin;
			cv_ptr->image.at<short>(locMax.y, locMax.x) = colorMax;
			break;
		case 4:
			cv_ptr->image.at<int>(locMin.y, locMin.x) = colorMin;
			cv_ptr->image.at<int>(locMax.y, locMax.x) = colorMax;
			break;
		}
	}

	// Draw text indicating the pixel value.
	if (m_config.AnnotateXY)
	{
		locXY.y += 10;
		putText(cv_ptr->image, stXY, locXY, cv::FONT_HERSHEY_PLAIN, scaleText, cv::Scalar::all(colorXY));
		locXY.y -= 10;

		// Draw dot on pixel.
		switch(cv_ptr->image.elemSize())
		{
		case 1:
			cv_ptr->image.at<char>(locXY.y, locXY.x) = colorXY;
			break;
		case 2:
			cv_ptr->image.at<short>(locXY.y, locXY.x) = colorXY;
			break;
		case 4:
			cv_ptr->image.at<int>(locXY.y, locXY.x) = colorXY;
			break;
		}
	}


	m_pubImageAnalysis.publish(cv_ptr->toImageMsg());

} // Imagetopic_callback()


void CImageAnalysis::Main(void)
{
	// Start the dynamic_reconfigure server.
	dynamic_reconfigure::Server<Config>					srv;
	dynamic_reconfigure::Server<Config>::CallbackType	fnCallback;

	fnCallback = boost::bind(&CImageAnalysis::ros_reconfigure_callback, this, _1, _2);
	srv.setCallback(fnCallback);

	ros::spin();

} // Main()


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ImageAnalysis");
	CImageAnalysis imageanalysis;

	imageanalysis.Main();


	return 0;
}


