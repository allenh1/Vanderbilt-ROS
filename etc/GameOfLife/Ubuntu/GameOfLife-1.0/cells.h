#include <iostream>
#include <string>
#include <cstdlib>
#include <stdlib.h>
#include </usr/local/include/SDL/SDL.h>
#include <ctime>
#include <sstream>
#include <cmath>

using namespace std;

#ifndef CLASSES_H_INCLUDED
#define CLASSES_H_INCLUDED

class Cell
{
    public:
        Cell();
        ~Cell();
        void init();
        void drawMe(SDL_Surface *screen);
        void setPosition(int32_t x, int32_t y);
        void setDims(int32_t w, int32_t h);
        void setNeighbors(Cell * tl, Cell * tt, Cell * tr, Cell * lt, Cell * rt, Cell * bl, Cell * bb, Cell * br);
        bool alive();
        bool willBeAlive();
        int32_t x(), y(), w(), h();
        int32_t numLiveNeighbors();
        void kill();
        void resurrect();
        void toggleState();
        void grimReaper();
        void setColor(uint32_t color);
    private:
        uint32_t _color;
        int32_t _x, _y, _w, _h;
        bool _alive, _willBeAlive;
        SDL_Surface *_cell;
        Cell *_tl, *_tt, *_tr, *_lt, *_rt, *_bl, *_bb, *_br;
};

class CellManager {
    public:
        static void init(const uint32_t screenWidth, const uint32_t screenHeight, const uint32_t cellWidth, const uint32_t cellHeight, const int32_t columns, const int32_t rows);
        static void run(SDL_Surface * screen);
        static void release();

    private:
        static Cell **_cells;
        static uint32_t _screenWidth, _screenHeight;
        static int32_t _rows, _columns;
};
//ubuntu colors
static uint32_t colorDead  = 0x3d3a38;
static uint32_t colorAlive = 0xe6c212;

//origional colors
const uint32_t origionalDead = 0x646464;
const uint32_t origionalAlive = 0xc8c8c8;

//mac osx colors
static uint32_t colorDeadm = 0xc8c8c8;
static uint32_t colorAlivem = 0x072FB5;

//windows colors
static uint32_t colorDeadw = 0xaafafd;
static uint32_t colorAlivew = 0x0F43CD;

const uint32_t boxWidthInPx  = 10;
const uint32_t boxHeightInPx = 10;

const char programName[] = "Game of Life";

#endif // CLASSES_H_INCLUDED
