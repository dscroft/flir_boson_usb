#ifndef BOSON_CAMERA_HPP
#define BOSON_CAMERA_HPP

#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>


enum Video_Mode
{
  YUV = 0,
  RAW16 = 1
};

enum Sensor_Types {
  Boson320, Boson640
};


/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(cv::Mat input_16, cv::Mat output_8, int height, int width) {
	int i, j;  // aux variables

	// auxiliary variables for AGC calcultion
	unsigned int max1=0;         // 16 bits
	unsigned int min1=0xFFFF;    // 16 bits
	unsigned int value1, value2, value3, value4;

	// RUN a super basic AGC
	for (i=0; i<height; i++) {
		for (j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			if ( value3 <= min1 ) {
				min1 = value3;
			}
			if ( value3 >= max1 ) {
				max1 = value3;
			}
			//printf("%X.%X.%X  ", value1, value2, value3);
		}
	}
	//printf("max1=%04X, min1=%04X\n", max1, min1);

	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
			// printf("%04X \n", value4);

			output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
		}
	}

}



class BosonCamera
{
private:
    int fd = -1;
    int width = -1, height = -1;
    struct v4l2_capability cap;

	// Declarations for RAW16 representation
        // Will be used in case we are reading RAW16 format
	// Boson320 , Boson 640
	cv::Mat thermal16;               // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	cv::Mat thermal16_linear;         // OpenCV output buffer : Data used to display the video

	// Declarations for 8bits YCbCr mode
        // Will be used in case we are reading YUV format
	// Boson320, 640 :  4:2:0
 	cv::Mat thermal_luma;  // OpenCV input buffer
	cv::Mat thermal_rgb;    // OpenCV output buffer , BGR -> Three color spaces (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)

    Video_Mode video_mode;
    struct v4l2_buffer bufferinfo;

    void _open_device( const std::string video);
    void _set_format( Sensor_Types my_thermal );
    void* _alloc_buffers();

public:
    BosonCamera();

    void init( const std::string video, Sensor_Types my_thermal=Boson320, Video_Mode video_mode=RAW16 )
    {
        this->stop();

        this->video_mode = video_mode;

        _open_device( video );
        _set_format( my_thermal );
        void * buffer_start = _alloc_buffers();

        this->thermal16 = cv::Mat(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	    this->thermal16_linear = cv::Mat(height,width, CV_8U, 1);         // OpenCV output buffer : Data used to display the video

        this->thermal_luma = cv::Mat(height+height/2, width,  CV_8UC1, buffer_start);  // OpenCV input buffer
	    this->thermal_rgb = cv::Mat(height, width, CV_8UC3, 1);    // OpenCV output buffer , BGR -> Three color spaces (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)
    }

    void stop()
    {
        if( this->fd >= 0 ) 
        {
            // Deactivate streaming
            if( ioctl(fd, VIDIOC_STREAMOFF, &(bufferinfo.type)) < 0 ){
                throw std::runtime_error( "ERROR : VIDIOC_STREAMOFF. Cannot deactivate streaming" );
            };

            close( this->fd );
            this->fd = -1;
        }
    }

    void read()
    {
	    // Put the buffer in the incoming queue.
		if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0)
        {
            throw std::runtime_error("ERROR : VIDIOC_QBUF. Cannot put buffer in incoming queue");
		}

		// The buffer's waiting in the outgoing queue.
		if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) 
        {
            throw std::runtime_error("ERROR : VIDIOC_DQBUF. Cannot put buffer in outgoing queue");
		}

		// -----------------------------
		// RAW16 DATA
		if( video_mode == RAW16 ) 
        {
			AGC_Basic_Linear(thermal16, thermal16_linear, height, width);
        }
		else 
        {  
            // Video is in 8 bits YUV
        	cvtColor(thermal_luma, thermal_rgb, cv::COLOR_YUV2BGR_I420, 0 );   // 4:2:0 family instead of 4:2:2 ...
		}
    }

    const cv::Mat& get_linear() const;

    const cv::Mat& get_rgb() const;

    Video_Mode get_video_mode() const;

    ~BosonCamera();
};

#endif // BOSON_CAMERA_HPP