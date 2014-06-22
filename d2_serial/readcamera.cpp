// ==============================================================================================
// This file is part of the VRmagic VRmUsbCam2 C API Demo Application
// ==============================================================================================
// Camera Reading / Main Loop
// ----------------------------------------------------------------------------------------------

#include "demo.h"

#ifdef __linux__
#include <SDL/SDL.h>
#define sprintf_s snprintf
#else
#include <SDL.h>
#endif

#ifndef min
#define min(a, b)            (((a) < (b)) ? (a) : (b))
#endif

enum OBJECT_MODE  { OBJECT_TEACH , OBJECT_SEARCH, OBJECT_INSPECT_TEMPLATE};

#include "ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

char *rosSrvrIp = "192.168.1.30";

void readCamera(VRmUsbCamDevice device, VRmDWORD port, VRmImageFormat target_format, VRmRectI src_cropping_region)
{
	// create gray images to use in vm_lib
	VRmImage* p_gray_src_img=0;
	VRmImage* p_gray_dst_img=0;
	VRmImageFormat gray_format = target_format;
	gray_format.m_color_format = VRM_GRAY_8;
	// actual allocation, use VRmUsbCamFreeImage to free images allocated with VRmUsbCam Lib
	VRMEXECANDCHECK(VRmUsbCamNewImage(&p_gray_src_img, gray_format));
	VRMEXECANDCHECK(VRmUsbCamNewImage(&p_gray_dst_img, gray_format));


	// ROS
	ros::NodeHandle  nh;
	sensor_msgs::Image ros_image;
	ros_image.height       = p_gray_src_img->m_image_format.m_height;
	ros_image.width        = p_gray_src_img->m_image_format.m_width;
	ros_image.step         = ros_image.width;
	int bytes = ros_image.height*ros_image.step;

	int packet_size = 128;
	ros_image.data_length = packet_size;
	//ros_image.st_data = bytes;
	unsigned char* buf = new unsigned char[bytes];
	ros_image.data = new unsigned char[packet_size];

	ros::Publisher pub("image", &ros_image);

    nh.initNode(rosSrvrIp);
    nh.advertise(pub);

    int seq = 0;



	VD_IMAGE   in_image;     /* declare VM_Lib image structures, wrap around VRmImage */
	in_image.st     = (long)((const unsigned char*)p_gray_src_img->mp_buffer);
	in_image.dx     = p_gray_src_img->m_image_format.m_width;
	in_image.dy     = p_gray_src_img->m_image_format.m_height;
	in_image.pitch  = p_gray_src_img->m_pitch;

	VD_IMAGE   out_image;     /* declare VM_Lib image structures, wrap around VRmImage */
	out_image.st     = (long)((const unsigned char*)p_gray_dst_img->mp_buffer);
	out_image.dx     = p_gray_dst_img->m_image_format.m_width;
	out_image.dy     = p_gray_dst_img->m_image_format.m_height;
	out_image.pitch  = p_gray_dst_img->m_pitch;

	//VD_IMAGE   template_image;  // this buffer will be allocated later
	//template_image.st=0;

	//the ROI of the pattern we want to learn
	/*VRmRectI template_roi;
	template_roi.m_left=min((gray_format.m_width/3) & ~7, 240);
	template_roi.m_top=min((gray_format.m_height/3) & ~7, 240);
	template_roi.m_width= 64;
	template_roi.m_height=64;*/

	OR_HND     or_hnd;                  /* handle object recognition */

	or_init(&or_hnd);    // open VM_LIB object recognition, the handle holds the learned pattern information for reuse

	// ------------------------------------------------------------------------
	// main loop: read images and draw them to screen
	// ------------------------------------------------------------------------

	std::cout << "Reading from camera..." << std::endl;


	VRMEXECANDCHECK(VRmUsbCamResetFrameCounter(device));
	// start grabber at first
	VRMEXECANDCHECK(VRmUsbCamStart(device));

	char ch=0;
	OBJECT_MODE mode = OBJECT_TEACH;

	// and enter the loop
	do
	{
		// lock next (raw) image for read access, convert it to the desired
		// format and unlock it again, so that grabbing can
		// go on
		VRmImage* p_source_img=0;
		VRmDWORD frames_dropped;
		if(!VRmUsbCamLockNextImageEx2(device, port, &p_source_img, &frames_dropped, 5000))
		{
			int error_code = VRmUsbCamGetLastErrorCode();
			switch(VRmUsbCamGetLastErrorCode())
			{
			case VRM_ERROR_CODE_FUNCTION_CALL_TIMEOUT:
			case VRM_ERROR_CODE_TRIGGER_TIMEOUT:
			case VRM_ERROR_CODE_TRIGGER_STALL:
				std::cout << "VRmUsbCamLockNextImageEx2() failed with " << VRmUsbCamGetLastError() << std::endl;
				break;

			case VRM_ERROR_CODE_GENERIC_ERROR:
			default:
				LogExit();
			}
		}

		// note: p_source_img may be null in case a recoverable error
		// (like a trigger timeout) occurred.
		// in this case, we just pump GUI events and then continue with the loop
		if (p_source_img)
		{
			VRmImage* p_cropped_src_img=0;
			VRMEXECANDCHECK(VRmUsbCamCropImage(&p_cropped_src_img, p_source_img, &src_cropping_region));

			//convert source image into gray image for further processing with VM_LIB
			VRMEXECANDCHECK(VRmUsbCamConvertImage(p_cropped_src_img, p_gray_src_img));

			//ros_image.header.stamp = ros::Time::now();
			ros_image.header.seq   = seq++;


			ros_image.encoding = "mono8";

			std::cout << bytes << std::endl;
			memcpy(buf, p_gray_src_img->mp_buffer, bytes);

			/*switch(mode)
			{
			case OBJECT_TEACH: // teaching mode
				teach_template(&in_image, &out_image, &template_image, &or_hnd, ch, &template_roi);
				if(ch == ' ')
					//switch to search mode
					mode=OBJECT_SEARCH;
				break;

			case OBJECT_INSPECT_TEMPLATE: //display the image section we have learned the pattern from
				show_template(&out_image, &template_image, template_roi, ch);
				switch(ch){
			case ' ': mode=OBJECT_SEARCH; break;
			case 't': mode= OBJECT_TEACH; break;
			default: break;
				}
				break;

			case OBJECT_SEARCH:
				search_template(&in_image, &out_image, &or_hnd, ch);
				switch(ch){
			case 'i': mode=OBJECT_INSPECT_TEMPLATE; break;
			case 't': mode= OBJECT_TEACH; break;
			default: break;
				}

				break;

			default: std::cout << "Unknown mode!" << std::endl; LogExit();

			}*/

			// lock the SDL off-screen buffer to output the image to the screen.
			// The screen_buffer_pitch variable will receive the pitch (byte size of
			// one line) of the buffer.
			VRmDWORD screen_buffer_pitch;
			VRmBYTE* p_screen_buffer=SDLLockBuffer(screen_buffer_pitch);

			// now, wrap a VRmImage around the locked screen buffer to receive the converted image
			VRmImage* p_target_img=0;
			VRmUsbCamSetImage(&p_target_img, target_format, p_screen_buffer, screen_buffer_pitch);

			VRMEXECANDCHECK(VRmUsbCamConvertImage(p_gray_dst_img, p_target_img));

			VRMEXECANDCHECK(VRmUsbCamFreeImage(&p_cropped_src_img));
			VRMEXECANDCHECK(VRmUsbCamUnlockNextImage(device, &p_source_img));

			// see, if we had to drop some frames due to data transfer stalls. if so,
			// output a message
			//if (frames_dropped)
			//	std::cout << "- " << frames_dropped << " frame(s) dropped -" << std::endl;

			// free the resources of the target image
			VRMEXECANDCHECK(VRmUsbCamFreeImage(&p_target_img));
			// give the off-screen buffer back to SDL
			SDLUnlockBuffer();

			// and update the screen
			SDLUpdate();

			int packets = bytes/packet_size;
			if(bytes%packet_size) packets++;
			int cnt=0;
			for(int i=0; i<bytes; i++)
			{
				ros_image.data[cnt] = i % 255;
				if(++cnt==128)
				{
					cnt = 0;
					pub.publish(&ros_image);
				}
			}
			nh.spinOnce();
		}

		// check keyboard input
		ch=GetKey();


	} while (!(ch == 'q' || ch =='Q'));

	or_exit(&or_hnd);      // close object recognition

	// stop grabber
	VRMEXECANDCHECK(VRmUsbCamStop(device));
	VRMEXECANDCHECK(VRmUsbCamFreeImage(&p_gray_src_img));
	VRMEXECANDCHECK(VRmUsbCamFreeImage(&p_gray_dst_img));
	//if(template_image.st)
	//	ip_free_img(&template_image);


	delete [] ros_image.data;

	// close SDL output window
	SDLWindowClose();
}

void teach_template(VD_IMAGE* in_image, VD_IMAGE* out_image, VD_IMAGE* template_image, OR_HND* or_hnd, char ch, VRmRectI* p_template_roi)
{
	char  test_text[100];
	test_text[0]=0;

	//use the 'asdw' keys to place the ROI over the part of the image you want to learn
	adaptTemplateROI(ch, p_template_roi, in_image->dx, in_image->dy);

	// here, we do not want to destroy the original image, therefore we copy it and draw into the copy
	ip_img_copy(in_image, out_image);

	//draw box for ROI
	dr_box(out_image, p_template_roi->m_left, p_template_roi->m_top, p_template_roi->m_width, p_template_roi->m_height, 255);

	// menu
	dr_fill_box(out_image, 0, 0, 330, 85, 255);
	sprintf_s(test_text, 100, "Teaching mode: place ROI on object!");
	dr_text(out_image, test_text, 35, 25, 0, 255);
	sprintf_s(test_text, 100, "Press SPACE to learn pattern in ROI!");
	dr_text(out_image, test_text, 35, 40, 0, 255);
	sprintf_s(test_text, 100, "Move with wasd + WASD, q to quit");
	dr_text(out_image, test_text, 35, 55, 0, 255);
	sprintf_s(test_text, 100, "ROI: %d, %d, %d, %d", p_template_roi->m_left, p_template_roi->m_top, p_template_roi->m_width, p_template_roi->m_height);
	dr_text(out_image, test_text, 35, 70, 0, 255);

	if(ch==' ')//copy template and learn it
	{
		/* object recognition learn parameters, see VM_LIB manual for more information*/
		//vm_lib or_learn/search requires minimal search template size of 32x32
		LEARN_PAR  lpar;

		lpar.min_edge_h      = 19;  /* min edge height: 7, .., 15, .., 127              */
		/*  default = 19                                */
		lpar.min_edge_g      = 16;  /* min edge gradient: 6, .., 15, .., min_edge_h     */
		/*  default = 16                                */
		lpar.min_chain       =  8;  /* min edge contour length: 4, .., 24 pixels      */
		/*  default = 8                                 */
		lpar.save_edge_image =  0;  /* bool: !=0: save edge image at x0, y0 into     */
		/* video-image, no further pattern processing   */
		/*  default = 0                                 */
		lpar.lrn_mode        =  0;  /* learn mode:                                  */
		/*  0: normal learn mode (edg_img ignored)      */
		/*  1: learn & generate output chain image in   */
		/*     edg_img                                  */
		/*  2: learn, use input edg_img to mask the     */
		/*     unwanted pattern contours                */
		lpar.dr_clr          =  0;  /* drawing color [0, 255]                        */

		//copying is only needed for later inspection of template image
		if(template_image->st)
			ip_free_img(template_image);
		//VM_LIB image allocation, use "ip_free_img" to deallocate
		ip_alloc_img(p_template_roi->m_width, p_template_roi->m_height, template_image);
		//copy the ROI from original image into template image
		VM_BOX box = {p_template_roi->m_left, p_template_roi->m_top, p_template_roi->m_width, p_template_roi->m_height};
		ip_imgbox_to_img(in_image, template_image, &box);

		//actual learning step
		int retval=or_learn(or_hnd, in_image, 0, 0, p_template_roi->m_left, p_template_roi->m_top, p_template_roi->m_width, p_template_roi->m_height, &lpar);
		if(retval)
			std::cout << "Teaching error (" << retval << ")" << std::endl;

	}
}


void show_template(VD_IMAGE* out_image, VD_IMAGE* template_image, VRmRectI template_roi, char ch)
{
	char  test_text[100];
	test_text[0]=0;

	//fill image with background color
	dr_fill_img(out_image, 127);

	// menu
	dr_fill_box(out_image, 0, 0, 270, 85, 255);
	sprintf_s(test_text, 100, "Showing template!");
	dr_text(out_image, test_text, 35, 25, 0, 255);
	sprintf_s(test_text, 100, "t to teach new pattern!");
	dr_text(out_image, test_text, 35, 40, 0, 255);
	sprintf_s(test_text, 100, "SPACE to return to search!");
	dr_text(out_image, test_text, 35, 55, 0, 255);
	sprintf_s(test_text, 100, "q to quit");
	dr_text(out_image, test_text, 35, 70, 0, 255);

	//copy template below menu box
	VM_BOX box = {270, 90, template_roi.m_width, template_roi.m_height};
	ip_img_to_imgbox(template_image, out_image, &box);

}

void search_template(VD_IMAGE* in_image, VD_IMAGE* out_image, OR_HND* or_hnd, char ch)
{
	/* object recognition search parameters, see VM_LIB manual for more information */
	//vm_lib or_learn/search requires minimal search template size of 32x32
	SEARCH_PAR spar;
	static int min_rate =512;
	spar.min_edge_h      = 19;  /* min edge height: 7, .., 15, .., 127              */
	/*  default = 19                                */
	spar.min_edge_g      = 16;  /* min edge gradient, =6, .., 15, .., min_edge_h    */
	/*  default = 16                                */
	spar.min_chain       =  8;  /* min edge contour length, =4, .., 24 pixels     */
	/*  default = 8                                 */
	spar.restr_search    =  1;  /* 1 = restricted_searching_mode                */
	/*  default = 0                                 */
	spar.max_pm_items    =  10;  /* max allowed num of pattern matching items    */
	/*  default = 1                                 */
	spar.min_rate        = min_rate;
	/* min matching rate, =160, .., 1023, 1024==100%  */
	/*  default = 552                               */
	spar.contour_width   =  5;  /* min edge contour width, =3, 5, 7, 9 pixels      */
	/*  default = 5                                 */
	spar.min_scale       = 1024;/* min scale, 512-1024(=100%), keep <=max_scale!*/
	/*  default = 1024                              */
	spar.max_scale       = 1024;/* max scale, 1024(=100%)-1536, keep <200% !    */
	/*  default = 1024                              */
	spar.rot_sect_start  = -5;  /* rotation sector start, -180..179 deg         */
	/*  default = -5                                */
	spar.rot_sect_width  = 360; /* rotation sector width, 0..360 deg            */
	/*  default = 360                               */
	spar.save_edge_image =  0;  /* bool: 1 = save edge image at x0, y0 into      */
	/* video-image, no further search processing    */
	/*  default = 0                                 */
	spar.dr_clr          = 255;
	/* drawing color [0, 255]                        */

	OR_RES     res_buf[10];              /* object recognition results - make sure the array size is >= spar.max_pm_items */
	int nres=0;
	char  test_text[100];
	test_text[0]=0;

	// here, we do not want to destroy the original image, therefore we copy it and draw into the copy
	// remember: in_image holds the same image as p_gray_src_img, out_image as p_gray_dst_img
	ip_img_copy(in_image, out_image);

	//actual searching step
	int retval=or_search(or_hnd, in_image, out_image, 0, 0, in_image->dx, in_image->dy, &spar, res_buf, &nres);
	if(retval)
		std::cout << "Search error (" << retval << ")" << std::endl;

	// menu
	dr_fill_box(out_image, 0, 0, 370, 85, 255);
	sprintf_s(test_text, 100, "Found %d objects! t to teach new pattern!", nres);
	dr_text(out_image, test_text, 35, 25, 0, 255);
	sprintf_s(test_text, 100, "i to inspect search pattern!");
	dr_text(out_image, test_text, 35, 40, 0, 255);
	sprintf_s(test_text, 100, "-/+ to alter matching rate, q to quit");
	dr_text(out_image, test_text, 35, 55, 0, 255);
	sprintf_s(test_text, 100, "min. matching rate (160-1024): %d", spar.min_rate);
	dr_text(out_image, test_text, 35, 70, 0, 255);

	switch(ch){
		case '-': if(min_rate >= 176 ) min_rate -= 16; break;
		case '+': if(min_rate <= 1008) min_rate += 16; break;
		default: break;
	}

}

void adaptTemplateROI(char ch, VRmRectI* rect, int width, int height)
{
	//vm_lib or_search requires minimal search template size of 32x32
	const int step = 8;
	const int min_size=32;
	switch(ch)
	{

	case 'a': if(rect->m_left >= step )
				  rect->m_left -= step;
		break;
	case 'd': if(rect->m_left <= width - min_size - step) {
		rect->m_left+=step;
		if (rect->m_left + rect->m_width > width)
			rect->m_width -= step;
			  }
			  break;
	case 'w': if(rect->m_top >= step)
				  rect->m_top -= step;
		break;
	case 's': if(rect->m_top <= height - min_size - step){
		rect->m_top += step;
		if( rect->m_top +rect->m_height > height)
			rect->m_height -= step;
			  }
			  break;
	case 'A': if(rect->m_width >= min_size + step)
				  rect-> m_width -= step;
		break;
	case 'D': if(rect->m_left + rect->m_width <= width-step)
				  rect->m_width += step;
		break;
	case 'W': if(rect->m_height >= min_size + step)
				  rect->m_height -=step;
		break;
	case 'S': if(rect->m_top+rect->m_height <= height-step)
				  rect->m_height += step;
		break;
	default:
		break;
	}
}
