#ifdef __cplusplus
    #include <cstdlib>
#else
    #include <stdlib.h>
#endif
#ifdef __APPLE__
#include <SDL/SDL.h>
#else
#include </usr/local/include/SDL/SDL.h>
#endif
#include "cells.h"

using namespace std;

Cell ** CellManager::_cells;
uint32_t CellManager::_screenWidth;
uint32_t CellManager::_screenHeight;
int32_t  CellManager::_rows;
int32_t  CellManager::_columns;

const int32_t  numColumns = 75;
const int32_t  numRows    = 75;

int main(int argc, char **argv)
{
    SDL_Surface * screen = NULL;
    const SDL_VideoInfo * info = NULL;
    uint32_t vflags = 0;
    uint32_t screenWidth = numColumns * boxWidthInPx + numColumns - 1;
    uint32_t screenHeight = numRows * boxHeightInPx + numRows - 1;

    if(SDL_Init(SDL_INIT_VIDEO) < 0) {
        cerr<<"SDL Error: Failed to initialize SDL: "<<SDL_GetError()<<endl;
        return 1;
    }
    atexit(SDL_Quit);
    info = SDL_GetVideoInfo();

    if (info == NULL) {
        cerr<<"SDL Error: Error Getting video info: "<<SDL_GetError()<<endl;
        return 1;
    }

    vflags |= SDL_HWPALETTE;

    if (info->hw_available)
        vflags |= SDL_HWSURFACE;
    else
        vflags |= SDL_SWSURFACE;

    if (info->blit_hw )
        vflags |= SDL_HWACCEL;

    screen = SDL_SetVideoMode(screenWidth, screenHeight, 32, vflags);

    if(screen == NULL) {
        cerr<<"SDL Error: Failed to set video mode: "<<SDL_GetError()<<endl;
        return 1;
    }

    SDL_WM_SetCaption (programName, NULL);

    if (SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL) != 0) {
        cerr<<"SDL Error: Error enabling key repeat: "<<SDL_GetError()<<endl;
        return 1;
    }

    CellManager::init(screenWidth, screenHeight, boxWidthInPx, boxHeightInPx, numColumns, numRows);
    CellManager::run(screen);
    CellManager::release();
    SDL_FreeSurface(screen);
    return 0;
}
