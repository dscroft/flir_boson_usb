#include "BosonCamera.hpp"

BosonCamera::BosonCamera()
{}

BosonCamera::~BosonCamera()
{
    this->stop();
}

const cv::Mat& BosonCamera::get_linear() const
{
    return this->thermal16_linear;
}

const cv::Mat& BosonCamera::get_rgb() const
{
    return this->thermal_rgb;
}

Video_Mode BosonCamera::get_video_mode() const
{
    return this->video_mode;
}



void BosonCamera::_open_device( const std::string video)
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

void BosonCamera::_set_format( Sensor_Types my_thermal )
{
    struct v4l2_format format;
    memset(&format, 0, sizeof(format));

    // Common varibles
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    
    width = 640;
    height = 512;

    if( video_mode == RAW16 )
    {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
    
        if( my_thermal != Boson640 )
        {
            width = 320;
            height = 256;
        }
    }

    // request desired FORMAT
    if(ioctl(fd, VIDIOC_S_FMT, &format) < 0)
    {
        throw std::logic_error( "ERROR : VIDIOC_S_FMT. Cannot set format" );
    }
}

void* BosonCamera::_alloc_buffers()
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
