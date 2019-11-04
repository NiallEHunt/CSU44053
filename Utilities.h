/*
 * This code is provided as part of "A Practical Introduction to Computer Vision with OpenCV"
 * by Kenneth Dawson-Howe © Wiley & Sons Inc. 2014.  All rights reserved.
 */
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <iostream>
#define PI 3.14159265358979323846

using namespace std;
using namespace cv;

void FindLocalMaxima(Mat& input_image, Mat& local_maxima, double threshold_value);

void ShowHLS(Mat image);
void writeText( Mat image, char* text, int row, int column, Scalar colour=-1.0, double scale=0.4, int thickness=1 );
void writeText( Mat image, string text, int row, int column, Scalar pcolour = -1.0, double scale = 0.4, int thickness=1);
Mat JoinSingleImage(Mat& image1, string name1, Scalar passed_colour=-1.0);
Mat JoinImagesHorizontally(Mat& image1, string name1, Mat& image2, string name2, int spacing, Scalar passed_colour=-1.0);
Mat JoinImagesHorizontally( Mat& image1, const char* name1, Mat& image2, const char* name2, int spacing=0, Scalar colour=-1.0 );
Mat JoinImagesVertically(Mat& image1, string name1, Mat& image2, string name2, int spacing = 0, Scalar colour = -1.0);
Mat JoinImagesVertically(Mat& image1, const char* name1, Mat& image2, const char* name2, int spacing = 0, Scalar colour = -1.0);
void addGaussianNoise(Mat &image, double average=0.0, double standard_deviation=10.0);

VideoWriter* OpenVideoFile( char* filename, VideoCapture& video_to_emulate, int horizontal_multiple=1, int vertical_multiple=1, int spacing=0 );
VideoWriter* OpenVideoFile( char* filename, int codec, Size image_size, double fps, int horizontal_multiple=1, int vertical_multiple=1, int spacing=0 );
void WriteVideoFrame( VideoWriter* output_video, Mat& video_frame );
void CloseVideoFile( VideoWriter* video );

void invertImage(Mat &image, Mat &result_image);
void ImagesDemos( Mat& image1, Mat& image2, Mat& logo_image, Mat& people_image );
void HistogramsDemos( Mat& dark_image, Mat& fruit_image, Mat& people_image, Mat& skin_image, Mat all_images[], int number_of_images );
void BinaryDemos(Mat& pcb_image, Mat& stationery_image );
void RegionDemos(Mat& image1, Mat& image2, Mat& image3);
void GeometricDemos( Mat& image1, Mat& image2, Mat& image3 );
void VideoDemos( VideoCapture& surveillance_video, int starting_frame, bool clean_binary_images );
void EdgeDemos( Mat& image1, Mat& image2 );
void FeaturesDemos( Mat& image1, Mat& image2, Mat& image3 );
void RecognitionDemos( Mat& full_image, Mat& template1, Mat& template2, Mat& template1locations, Mat& template2locations, VideoCapture& bicycle_video, Mat& bicycle_background, Mat& bicycle_model, VideoCapture& people_video, CascadeClassifier& cascade, Mat& numbers, Mat& good_orings, Mat& bad_orings, Mat& unknown_orings );
void MeanShiftDemo( VideoCapture& video, Rect& starting_position, int starting_frame, int end_frame);
int CameraCalibration( string passed_settings_filename );
void TrackFeaturesDemo( VideoCapture& video, int starting_frame_number, int ending_frame_number );

Mat BackProjection(Mat& query_image, Mat all_images[], int number_of_images);
Mat BackProjection(Mat& query_image, Mat sample_image);
Mat kmeans_clustering(Mat& image, int k, int iterations);
double ComputeOTSUThreshold(Mat src, Mat mask);

void floodFillPostprocess(Mat& img, const Scalar& colorDiff = Scalar::all(1));

class TimestampEvent {
private:
	String mEventName;
	double mAverageDuration;
	double mLastDuration;
	int mEventCount;
public:
	TimestampEvent();
	void Reset(String event_name);
	void RecordEvent(int duration);
	double getLastTime();
	double getAverageTime();
	String getEventName();
	String getString(bool average=true, bool last=true);
};


class Timestamper {
private:
#define MAX_EVENTS 20
	TimestampEvent mEvents[MAX_EVENTS];
	int mEventCount;
	double mLastTickCount;
	double mTickFrequency;
public:
	Timestamper();
	void reset();
	void ignoreTimeSinceLastRecorded();
	void recordTime(String event = "");
	void putTimes(Mat output_image);
};

void invertImage(Mat &image, Mat &result_image);
Mat StretchImage( Mat& image );
Mat convert_32bit_image_for_display(Mat& passed_image, double zero_maps_to=0.0, double passed_scale_factor=-1.0 );
void show_32bit_image( char* window_name, Mat& passed_image, double zero_maps_to=0.0, double passed_scale_factor=-1.0 );
Mat ComputeDefaultImage( Mat& passed_image );
void DrawHistogram( MatND histograms[], int number_of_histograms, Mat& display_image );
void MyApplication();

double DistanceBetweenPoints(Point2d point1, Point2d point2);
double DistanceBetweenPoints(Point2i point1, Point2i point2);
