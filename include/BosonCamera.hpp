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

    void _open_device( const std::string video)
    {
        if((fd = open(video.c_str(), O_RDWR)) < 0)
        {
            throw std::logic_error( "Error : OPEN. Invalid Video Device" );
        }

        // Check VideoCapture mode is available
        if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
        {
            throw std::logic_error( "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" );
        }

        if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            throw std::logic_error( "The device does not handle single-planar video capture." );
        }
    }

    void _set_format( Sensor_Types my_thermal )
    {
        struct v4l2_format format;
        memset(&format, 0, sizeof(format));

        // Common varibles
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        
        // Two different FORMAT modes, 8 bits vs RAW16
        if( video_mode == RAW16 ) 
        {
            // I am requiring thermal 16 bits mode
            format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

            // Select the frame SIZE (will depend on the type of sensor)
            switch (my_thermal) 
            {
                case Boson640:  // Boson640
                    width=640;
                    height=512;
                    break;
                default:  // Boson320
                    width=320;
                    height=256;
                    break;
            }
        } 
        else 
        { // 8- bits is always 640x512 (even for a Boson 320)
            width = 640;
            height = 512;
        }

        // request desired FORMAT
        if(ioctl(fd, VIDIOC_S_FMT, &format) < 0)
        {
            throw std::logic_error( "ERROR : VIDIOC_S_FMT. Cannot set format" );
        }
    }

    void* _alloc_buffers()
    {
        // we need to inform the device about buffers to use.
        // and we need to allocate them.
        // we’ll use a single buffer, and map our memory using mmap.
        // All this information is sent using the VIDIOC_REQBUFS call and a
        // v4l2_requestbuffers structure:
        struct v4l2_requestbuffers bufrequest;
        bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufrequest.memory = V4L2_MEMORY_MMAP;
        bufrequest.count = 1;   // we are asking for one buffer

        if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0)
        {
            throw std::runtime_error("ERROR : VIDIOC_REQBUFS. Cannot allocate buffers");
        }

        // Now that the device knows how to provide its data,
        // we need to ask it about the amount of memory it needs,
        // and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
        // and its v4l2_buffer structure.

        
        memset(&bufferinfo, 0, sizeof(bufferinfo));

        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = 0;

        if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0)
        {
            throw std::runtime_error("ERROR : VIDIOC_QUERYBUF. Cannot query buffer");
        }


        // map fd+offset into a process location (kernel will decide due to our NULL). lenght and
        // properties are also passed
        void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);

        if(buffer_start == MAP_FAILED)
        {
            throw std::runtime_error("ERROR : mmap. Cannot map buffer");
        }

        // Fill this buffer with ceros. Initialization. Optional but nice to do
        memset(buffer_start, 0, bufferinfo.length);

        // Activate streaming
        int type = bufferinfo.type;
        if(ioctl(fd, VIDIOC_STREAMON, &type) < 0)
        {
            throw std::runtime_error("ERROR : VIDIOC_STREAMON. Cannot activate streaming");    
        }

        return buffer_start;
    }

public:
    BosonCamera()
    {}

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
        if( this->fd < 0 )
        {
            throw std::runtime_error("Device not initalised");
        }

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

    const cv::Mat& get_linear() const
    {
        return this->thermal16_linear;
    }

    const cv::Mat& get_rgb() const
    {
        return this->thermal_rgb;
    }

    Video_Mode get_video_mode() const
    {
        return this->video_mode;
    }

    ~BosonCamera()
    {
        this->stop();
    }
};

#endif // BOSON_CAMERA_HPP