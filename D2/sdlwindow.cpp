// ==============================================================================================
// This file is part of the VRmagic USB2.0 Camera Demo Application
// ==============================================================================================
// SDL Window
// ----------------------------------------------------------------------------------------------

#include "demo.h"

#include <iostream>

#ifdef __linux__
#include <SDL/SDL.h>
#else
#include <SDL.h>
#endif

#ifdef __arm__
#include <unistd.h>
#include <termios.h>

struct termios initial_settings, new_settings;
#endif

SDL_Surface* surface;
SDL_Surface* screen;

static int pixel_depth=0;
static int flags=0;

bool SDLQueryDisplayCaps(  VRmSizeI& screen_size, VRmColorFormat& screen_colorformat)
{

	//set some environment vars - not recommended by SDL (because of platform compatibility)
#ifdef __arm__
	SDL_putenv((char*)"SDL_NOMOUSE=1");
#endif

#ifdef D2_PLATFORM	
SDL_putenv((char*)"SDL_VIDEODRIVER=directfb");
#endif

	// initialize the SDL library
	if ( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		std::cout << "Failed to initialize the SDL library: " << SDL_GetError() << std::endl;
		return false;
	}

	// set flags
	flags = SDL_HWSURFACE;
#ifdef __arm__
	flags |= SDL_FULLSCREEN;
#endif


	// get pixel format of video display
	const SDL_VideoInfo* pf = SDL_GetVideoInfo();
	pixel_depth = pf->vfmt->BitsPerPixel;
	switch(pixel_depth){
		case 16: screen_colorformat = VRM_RGB_565;break;
		case 24: screen_colorformat = VRM_BGR_3X8;break;
		case 32: screen_colorformat = VRM_ARGB_4X8;break;
		default: std::cout << "Unhandled pixel format!" << std::endl; return false;
	}

	// determine the size of the video display
	SDL_Rect** modes = SDL_ListModes(0, flags);
	if(modes==NULL)
	{
		std::cout << "no screen format obtainable!" << std::endl;
		return false;
	}
	else if(modes==(SDL_Rect**)-1) // free to chose, usually the case on PC
	{
		screen_size.m_height=0;
		screen_size.m_width=0;
	}
	else // format fixed, usually the case on ARM (Davinci)
	{
		screen_size.m_height=modes[0]->h;
		screen_size.m_width=modes[0]->w;
	}

	return true;

}

bool SDLWindowInit(const char* fp_caption, VRmImageFormat f_format)
{

	SDL_WM_SetCaption( fp_caption, NULL );

	SDL_ShowCursor(false);

	screen = SDL_SetVideoMode( f_format.m_width, f_format.m_height, pixel_depth, flags );

	if ( screen == NULL )
	{
		std::cout << "Failed to set the SDL video mode: "  << SDL_GetError() << std::endl;
		return false;
	}

	Uint32 rmask;
	Uint32 gmask;
	Uint32 bmask;
	Uint32 amask;

	switch(pixel_depth){
case 16:rmask = 0xF800;
	gmask = 0x07E0;
	bmask = 0x001F;
	amask = 0x0000;
	break;
case 24:rmask = 0x0000ff;
	gmask = 0x00ff00;
	bmask = 0xff0000;
	amask = 0x000000;
	break;
case 32:rmask = 0x00ff0000;
	gmask = 0x0000ff00;
	bmask = 0x000000ff;
	amask = 0xff000000;
	break;
default: std::cout << "Unhandled pixel format!" << std::endl; return false;
	}

	surface = SDL_CreateRGBSurface( flags, f_format.m_width, f_format.m_height, pixel_depth, rmask, gmask, bmask, amask);

	/* Enable Unicode translation */
	SDL_EnableUNICODE( 1 );
	SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

#ifdef __arm__
	// Prepare kbhit() for console inputs (
	tcgetattr(0, &initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
#endif

	return true;
}


void 	SDLWindowClose()
{
	if ( surface != NULL )
	{
		SDL_FreeSurface( surface );
		surface = NULL;
	}
	if ( screen != NULL )
	{
		SDL_FreeSurface( screen );
		screen = NULL;
	}

	SDL_Quit();

#ifdef __arm__
	// Restore initial keyboard settings of the remote console
	tcsetattr(0, TCSANOW, &initial_settings);
#endif
}

unsigned char* SDLLockBuffer( unsigned int& f_pitch )
{
	SDL_LockSurface(surface);

	// offset to same pixel in next row (32bit surface)
	f_pitch= surface->pitch;
	return static_cast<unsigned char*>(surface->pixels);
}

void SDLUnlockBuffer()
{
	SDL_UnlockSurface(surface);
}

void SDLUpdate()
{

	SDL_BlitSurface(surface, 0, screen, 0);
	SDL_UpdateRect( screen, 0, 0, 0, 0);

}

char GetKey()
{
	// check local keyboard
	SDL_Event event;
	while(SDL_PollEvent(&event)){
		switch (event.type){
case SDL_KEYDOWN:
	char ch;
	if ( (event.key.keysym.unicode & 0xFF80) == 0 ) {
		ch = event.key.keysym.unicode & 0x7F;
		return ch;
	}
default:
	break;
		}
	}
#ifdef __arm__
	// Check for remote console keyboard input
	unsigned char ch;
	int nread;

	new_settings.c_cc[VMIN]=0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0, &ch, 1);
	new_settings.c_cc[VMIN]=1;
	tcsetattr(0, TCSANOW, &new_settings);
	if (nread == 1)
		return ch;
#endif

	return 0;
}
