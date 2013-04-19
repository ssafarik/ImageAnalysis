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
//#define MIN(a,b)	(((a)>(b)) ? (b) : (a))
//#define MAX(a,b)	(((a)<(b)) ? (b) : (a))

#define CfromK(k)	((k)/100.0-273.15)

using namespace cv;
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
	image_transport::Publisher		m_pubImageHistogram;
	Config							m_config;

	int m_changedTopic;
	int m_changedContrastAuto;
	int m_changedContrastRange;
	int m_changedAnnotateMinMax;
	int m_changedAnnotateROIMean;
	int m_changedPublishROIHistogram;
	int m_changedXROI;
	int m_changedYROI;
	int m_changedWROI;
	int m_changedHROI;
	int m_changedConvertKtoCforFLIR;

	int	m_width;
	int	m_height;

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
	m_width = 0;
	m_height = 0;
	m_pubImageAnalysis = m_imagetransport.advertise("image_analysis", 1);
	m_pubImageHistogram = m_imagetransport.advertise("image_histogram", 1);

}

void CImageAnalysis::ros_reconfigure_callback(Config &config, uint32_t level)
{
	m_changedTopic 					= (m_config.topic != config.topic);
	m_changedContrastAuto 			= (m_config.ContrastAuto != config.ContrastAuto);
	m_changedContrastRange 			= (m_config.ContrastRange != config.ContrastRange);
	m_changedAnnotateMinMax 		= (m_config.AnnotateMinMax != config.AnnotateMinMax);
	m_changedAnnotateROIMean 			= (m_config.AnnotateROIMean != config.AnnotateROIMean);
	m_changedPublishROIHistogram 	= (m_config.PublishROIHistogram != config.PublishROIHistogram);
	m_changedConvertKtoCforFLIR 	= (m_config.ConvertKtoCforFLIR != config.ConvertKtoCforFLIR);
	m_changedXROI                   = (m_config.xROI != config.xROI);
	m_changedYROI                   = (m_config.yROI != config.yROI);
	m_changedWROI                   = (m_config.wROI != config.wROI);
	m_changedHROI                   = (m_config.hROI != config.hROI);

	if (m_changedContrastAuto && config.ContrastAuto)
		config.ContrastRange = FALSE;

	if (m_changedContrastRange && config.ContrastRange)
		config.ContrastAuto = FALSE;

	if (config.colorHi <= config.colorLo)
		config.colorHi = config.colorLo+1;

	// Don't let the histogram ROI go offimage.
	if (m_changedXROI && m_width && config.xROI+config.wROI > m_width)
		config.xROI = MAX(0,m_width-config.wROI-1);
	if (m_changedYROI && m_height && config.yROI+config.hROI > m_height)
		config.yROI = MAX(0,m_height-config.hROI-1);
	if (m_changedWROI && m_width && config.xROI+config.wROI > m_width)
		config.wROI = MAX(0,m_width-config.xROI-1);
	if (m_changedHROI && m_height && config.yROI+config.hROI > m_height)
		config.hROI = MAX(0,m_height-config.yROI-1);

	if (m_changedTopic)
		m_subImageTopic = m_imagetransport.subscribe(config.topic, 1, &CImageAnalysis::Imagetopic_callback, this);

    m_config = config;
} // ros_reconfigure_callback()



void CImageAnalysis::Imagetopic_callback(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr	cv_ptr;
	double 					valMin, valMax;//, valMean;
	double					valMinScaled, valMaxScaled;//, valMeanscaled;
	cv::Point				ptMin, ptMax, ptXY, ptROI1, ptROI2;
	unsigned				minPixel, maxPixel;
	char 					buff[100];
	std::string				stLo, stHi, stXY;
	double					scaleText = 1.0;
	unsigned				colorMin, colorMax, colorXY;
	int 					xOffset, yOffset;
	cv::Scalar				valMean, valMeanScaled;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat 				mask(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1, cv::Scalar(0,0,0));

	ptROI1.x = m_config.xROI;
	ptROI1.y = m_config.yROI;
	ptROI2.x = MIN(m_config.xROI+m_config.wROI, cv_ptr->image.cols);
	ptROI2.y = MIN(m_config.yROI+m_config.hROI, cv_ptr->image.rows);

	m_width = image->width;
	m_height = image->height;
	cv::rectangle(mask, ptROI1, ptROI2, cv::Scalar(1,1,1), 1, 4, 0);


	// Get min,max pixel values.
	minMaxLoc(cv_ptr->image, &valMin, &valMax, &ptMin, &ptMax);
	maxPixel = (1<<(cv_ptr->image.elemSize()*8))-1;
	minPixel = 0;

	// Get value at (x,y).
	ptXY.x = (ptROI1.x + ptROI2.x)/2;
	ptXY.y = (ptROI1.y + ptROI2.y)/2;
	valMean = cv::mean(cv_ptr->image, mask);


	// Text for the pixel values.
	if (m_config.ConvertKtoCforFLIR)
	{
		sprintf(buff, "%0.2f", CfromK(valMin)); // Assumes that pixel values are in units of 10 millikelvin.  FLIR camera feature IRFormat="TemperatureLinear10mK"
		stLo = buff;
		sprintf(buff, "%0.2f", CfromK(valMax));
		stHi = buff;
		sprintf(buff, "%0.2f", CfromK(valMean[0]));
		stXY = buff;
	}
	else
	{
		sprintf(buff, "%0.0f", valMin);
		stLo = buff;
		sprintf(buff, "%0.0f", valMax);
		stHi = buff;
		sprintf(buff, "%0.0f", valMean[0]);
		stXY = buff;
	}

	if (m_config.PublishROIHistogram)
	{
		int 		 histSize = 256;
		float 		 range[2];
		range[0] = (float)m_config.colorLo;
		range[1] = (float)m_config.colorHi;
		const float	*histRange = {range};
		bool 		 uniform = true;
		bool 		 accumulate = false;
		cv::Mat 	 hist;

		/// Compute the histogram.
		calcHist( &cv_ptr->image, 1, 0, mask, hist, 1, &histSize, &histRange, uniform, accumulate );

		// Draw the histogram.
		int 		 hist_w = 512;
		int 		 hist_h = 400;
		int 		 bin_w = cvRound( (double) hist_w/histSize );
		Mat   		 histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );

		/// Normalize the result to [ 0, histImage.rows ]
		cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

		/// Draw.
		for (int i=1; i<histSize; i++ )
		{
		  cv::line(histImage, cv::Point(bin_w*(i-1), hist_h-cvRound(hist.at<float>(i-1))),
				  cv::Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
				  cv::Scalar(255, 0, 0), 1, 8, 0
				  );
		}

		m_pubImageHistogram.publish(cv_bridge::CvImage(image->header, "mono8", histImage).toImageMsg());

	}


	///////////////////////////////////////////////
	// Original image above, Modified image below.
	///////////////////////////////////////////////


	// Rerange the image.
	if (m_config.ContrastAuto)
		cv_ptr->image = (cv_ptr->image - valMin) * maxPixel / (valMax-valMin);

	// Rerange the image.
	if (m_config.ContrastRange)
		cv_ptr->image = (cv_ptr->image - m_config.colorLo) * maxPixel / (m_config.colorHi - m_config.colorLo);


	// Get pixel values after scaling & text colors.
	switch(cv_ptr->image.elemSize())
	{
	case 1:
//		valMeanscaled = cv_ptr->image.at<unsigned char>(ptXY.y, ptXY.x);
		valMinScaled = cv_ptr->image.at<unsigned char>(ptMin.y, ptMin.x);
		valMaxScaled = cv_ptr->image.at<unsigned char>(ptMax.y, ptMax.x);
		break;
	case 2:
//		valMeanscaled = cv_ptr->image.at<unsigned short>(ptXY.y, ptXY.x);
		valMinScaled = cv_ptr->image.at<unsigned short>(ptMin.y, ptMin.x);
		valMaxScaled = cv_ptr->image.at<unsigned short>(ptMax.y, ptMax.x);
		break;
	case 4:
//		valMeanscaled = cv_ptr->image.at<unsigned int>(ptXY.y, ptXY.x);
		valMinScaled = cv_ptr->image.at<unsigned int>(ptMin.y, ptMin.x);
		valMaxScaled = cv_ptr->image.at<unsigned int>(ptMax.y, ptMax.x);
		break;
	}
	valMeanScaled = cv::mean(cv_ptr->image, mask);
	colorXY  = ((unsigned)valMeanScaled[0]>(maxPixel/2)) ? minPixel : maxPixel;
	colorMin = ((unsigned)valMinScaled>(maxPixel/2)) ? minPixel : maxPixel;
	colorMax = ((unsigned)valMaxScaled>(maxPixel/2)) ? minPixel : maxPixel;


	if (m_config.PublishROIHistogram || m_config.AnnotateROIMean)
		cv::rectangle(cv_ptr->image, ptROI1, ptROI2, cv::Scalar::all(maxPixel-colorXY), 1, 4, 0);

	// Draw text indicating the min,max pixel values.
	if (m_config.AnnotateMinMax)
	{
		xOffset = (ptMin.x>image->width/2) ? -50 : 5;
		yOffset = (ptMin.y>image->height/2) ? -2 : 12;
		ptMin.x += xOffset;
		ptMin.y += yOffset;
		putText(cv_ptr->image, stLo, ptMin, cv::FONT_HERSHEY_PLAIN, scaleText, cv::Scalar::all(colorMin));
		ptMin.y -= yOffset;
		ptMin.x -= xOffset;

		xOffset = (ptMax.x>image->width/2) ? -50 : 5;
		yOffset = (ptMax.y>image->height/2) ? -2 : 12;
		ptMax.x += xOffset;
		ptMax.y += yOffset;
		putText(cv_ptr->image, stHi, ptMax, cv::FONT_HERSHEY_PLAIN, scaleText, cv::Scalar::all(colorMax));
		ptMax.y -= yOffset;
		ptMax.x -= xOffset;

		// Draw dots on min,max pixels.
		switch(cv_ptr->image.elemSize())
		{
		case 1:
			cv_ptr->image.at<unsigned char>(ptMin.y, ptMin.x) = colorMin;
			cv_ptr->image.at<unsigned char>(ptMax.y, ptMax.x) = colorMax;
			break;
		case 2:
			cv_ptr->image.at<unsigned short>(ptMin.y, ptMin.x) = colorMin;
			cv_ptr->image.at<unsigned short>(ptMax.y, ptMax.x) = colorMax;
			break;
		case 4:
			cv_ptr->image.at<unsigned int>(ptMin.y, ptMin.x) = colorMin;
			cv_ptr->image.at<unsigned int>(ptMax.y, ptMax.x) = colorMax;
			break;
		}
	}

	// Draw text indicating the pixel value.
	if (m_config.AnnotateROIMean)
	{
		xOffset = (ptXY.x>image->width/2) ? -50 : 5;
		yOffset = (ptXY.y>image->height/2) ? -2 : 12;
		ptXY.y += yOffset;
		ptXY.x += xOffset;
		putText(cv_ptr->image, stXY, ptXY, cv::FONT_HERSHEY_PLAIN, scaleText, cv::Scalar::all(colorXY));
		ptXY.y -= yOffset;
		ptXY.x -= xOffset;

		// Draw dot on pixel.
		switch(cv_ptr->image.elemSize())
		{
		case 1:
			cv_ptr->image.at<unsigned char>(ptXY.y, ptXY.x) = colorXY;
			break;
		case 2:
			cv_ptr->image.at<unsigned short>(ptXY.y, ptXY.x) = colorXY;
			break;
		case 4:
			cv_ptr->image.at<unsigned int>(ptXY.y, ptXY.x) = colorXY;
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


