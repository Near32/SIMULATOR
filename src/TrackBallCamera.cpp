#include "TrackBallCamera.h"





TrackBallCamera::TrackBallCamera()
{
    const char *_hand1[] =
        {
            /* width height num_colors chars_per_pixel */
            " 16 16 3 1 ",
            /* colors */
            "X c #000000",
            ". c #ffffff",
            "  c None",
            /* pixels */
            "       XX       ",
            "   XX X..XXX    ",
            "  X..XX..X..X   ",
            "  X..XX..X..X X ",
            "   X..X..X..XX.X",
            "   X..X..X..X..X",
            " XX X.......X..X",
            "X..XX..........X",
            "X...X.........X ",
            " X............X ",
            "  X...........X ",
            "  X..........X  ",
            "   X.........X  ",
            "    X.......X   ",
            "     X......X   ",
            "     X......X   ",
            "0,0"
        };

    const char *_hand2[] =
        {
            /* width height num_colors chars_per_pixel */
            " 16 16 3 1 ",
            /* colors */
            "X c #000000",
            ". c #ffffff",
            "  c None",
            /* pixels */
            "                ",
            "                ",
            "                ",
            "                ",
            "    XX XX XX    ",
            "   X..X..X..XX  ",
            "   X........X.X ",
            "    X.........X ",
            "   XX.........X ",
            "  X...........X ",
            "  X...........X ",
            "  X..........X  ",
            "   X.........X  ",
            "    X.......X   ",
            "     X......X   ",
            "     X......X   ",
            "0,0"
        };
        
    //hand1 = cursorFromXPM(_hand1); //création du curseur normal
    //hand2 = cursorFromXPM(_hand2); //création du curseur utilisé quand le bouton est enfoncé

    //SDL_SetCursor(hand1); //activation du curseur normal
    hold = false; //au départ on part du principe que le bouton n'est pas maintenu

    angleY = 0;
    angleZ = 0;

    distance = 20; //distance initiale de la caméra avec le centre de la scène
    motionSensitivity = 0.3;
    scrollSensitivity = 1.0;

}

void TrackBallCamera::onMouseMotion( const SDL_MouseMotionEvent& mmevent)
{
	if(hold)
	{
		angleZ += mmevent.xrel*motionSensitivity;
		angleY += mmevent.yrel*motionSensitivity;
		
		if(angleY > 90)
			angleY = 90;
		else if(angleY < -90)
			angleY = -90;
			
	}
}


void TrackBallCamera::onMouseButton( const SDL_MouseButtonEvent& mbevent)
{
	
	if(mbevent.button == SDL_BUTTON_LEFT)
	{
		if((hold)&&(mbevent.type == SDL_MOUSEBUTTONUP))
		{
			hold = false;
			//SDL_SetCursor(hand1);
		}
		else if( (!hold)&&(mbevent.type == SDL_MOUSEBUTTONDOWN))
		{
			hold = true;
			//SDL_SetCursor(hand2);
		}
	}
	else if( ( (int)mbevent.button == SDL_BUTTON_WHEELUP) ) //&&(mbevent.type == SDL_MOUSEBUTTONDOWN) )
	{
		distance -= scrollSensitivity;
		if(distance < 0.1)
			distance = 0.1;
	}
	else if( ( (int)mbevent.button == SDL_BUTTON_WHEELDOWN) )//&&(mbevent.type == SDL_MOUSEBUTTONDOWN) )
	{
		distance += scrollSensitivity;
	}
	
}

/*
void TrackBallCamera::onMouseWheel( const SDL_MouseWheelEvent& mwevent)
{
	if( (mwevent.direction == SDL_MOUSEWHEEL_NORMAL) ) //&&(mbevent.type == SDL_MOUSEBUTTONDOWN) )
	{
		std::cout << "WHEELUP" << std::endl;
		distance -= scrollSensitivity;
		if(distance < 0.1)
			distance = 0.1;
	}
	else if( (mwevent.direction == SDL_MOUSEWHEEL_FLIPPED) )//&&(mbevent.type == SDL_MOUSEBUTTONDOWN) )
	{
		std::cout << "WHEELDOWN" << std::endl;
		distance += scrollSensitivity;
	}
	
}
*/

void TrackBallCamera::onKeyboard( const SDL_KeyboardEvent& kevent)
{
	if( (kevent.type == SDL_KEYDOWN)&&(kevent.keysym.sym == SDLK_h) )
	{
		angleY = 0;
		angleZ = 0;
	}
}


void TrackBallCamera::look()
{
	gluLookAt(distance, 0, 0,
			0,0,0,
			0,0,1);
			
	glRotated(angleY,0,1,0);
	glRotated(angleZ,0,0,1);
}


TrackBallCamera::~TrackBallCamera()
{
	//SDL_FreeCursor(hand1);
	//SDL_FreeCursor(hand2);
	
	//SDL_SetCursor(NULL);
}


