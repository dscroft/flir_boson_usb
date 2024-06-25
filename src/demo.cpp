/*
------------------------------------------------------------------------
-  FLIR Systems - Linux Boson  Capture & Recording                     -
------------------------------------------------------------------------
-  This code is using part of the explanations from this page          -
-  https://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/  -
-                                                                      -
-  and completed to be used with FLIR Boson cameras in 16 and 8 bits.  -
-  Internal AGC for 16bits mode is very basic, with just the intention -
-  of showing how to make that image displayable                       - 
------------------------------------------------------------------------

 BosonUSB [r/y/z/s/t/f] [0..9]
	r    : raw16 bits video input (default)
	y    : agc-8 bits video input
	z    : zoom mode to 640x480 (only applies to raw16 input)
        f<name> : record TIFFS in Folder <NAME>
        t<number> : number of frames to record
	s[b,B]  : camera size : b=boson320, B=boson640
	[0..9]  : linux video port

./BosonUSB   ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB r ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB y ->  opens Boson320 /dev/video0  in AGC-8bits mode
./BosonUSB sB 1    ->  opens Boson640 /dev/video1  in RAW16 mode
./BosonUSB sB y 2  ->  opens Boson640 /dev/video2  in AGC-8bits mode
./BosonUSB fcap -> creates a folder named 'cap' and inside TIFF files (raw16, agc, yuv) will be located.

*/

#include "BosonCamera.hpp"
#include <opencv2/opencv.hpp>

#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

// Need to clean video for linux structs to avoid some random initializations problems (not always present)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

// Global variables to keep this simple
int width;
int height;

// Types of sensors supported




/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT
int main(int argc, char** argv )
{
    const std::string video = "/dev/video0";

    BosonCamera camera;
    camera.init( video, Sensor_Types::Boson320, Video_Mode::RAW16 );

	// Reaad frame, do AGC, paint frame
	for (;;) 
    {
        camera.read();


        // Display the image

        cv::imshow("Thermal", camera.get_linear() );
        cv::imshow("Thermal RGB", camera.get_rgb() );
	

		// Press 'q' to exit
		if( cv::waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}

	}
	// Finish Loop . Exiting.

	return EXIT_SUCCESS;
}
