#include "cells.h"

Cell::Cell()
{
    _color = colorDead;
    _x = _y = _w = _h = 0x0000;
    _alive = _willBeAlive = false;
    _cell = NULL;
    _tl = _tt = _tr = _lt = _rt = _bl = _bb = _br = NULL;
}

Cell::~Cell()
{
    SDL_FreeSurface( _cell );
    _cell = NULL;
}

void Cell::init()
{
    _cell = SDL_CreateRGBSurface(SDL_HWSURFACE, _w, _h, 32, 0, 0, 0, 0);
    if (_cell == NULL) {
        cerr << "SDL Error: Surface " << _cell << " failed to initialize: " << SDL_GetError() << endl;
        exit(1);
    }
    if (SDL_FillRect(_cell, NULL, _color ) != 0 ) {
        cerr << "SDL Error: Surface " << _cell << " failed to fill: " << SDL_GetError() << endl;
        exit(1);
    }
}

void Cell::drawMe(SDL_Surface *screen)
{
    SDL_Rect rect;
    rect.x = _x;
    rect.y = _y;
    rect.w = _cell->w;
    rect.h = _cell->h;

    SDL_BlitSurface(_cell, NULL, screen, &rect);
}

void Cell::setColor(uint32_t color)
{
    _color = color;
    _color |= 0xff << 24;
    if (SDL_FillRect(_cell, NULL, _color ) != 0 ) {
        cerr << "SDL Error: Surface " << _cell << " failed to fill: " << SDL_GetError() << endl;
        exit(1);
    }
}

void Cell::setPosition(int32_t x, int32_t y)
{
    _x = x;
    _y = y;
}

void Cell::setDims(int32_t w, int32_t h)
{
    _w = w;
    _h = h;
}

void Cell::setNeighbors(Cell * tl, Cell * tt, Cell * tr, Cell * lt, Cell * rt, Cell * bl, Cell * bb, Cell * br)
{
    _tl = tl;
    _tt = tt;
    _tr = tr;
    _lt = lt;
    _rt = rt;
    _bl = bl;
    _bb = bb;
    _br = br;
}

bool Cell::alive()
{
    return _alive;
}

bool Cell::willBeAlive()
{
    return _willBeAlive;
}

int32_t Cell::x()
{
    return _x;
}

int32_t Cell::y()
{
    return _y;
}

int32_t Cell::w()
{
    return _w;
}

int32_t Cell::h()
{
    return _h;
}

int32_t Cell::numLiveNeighbors()
{
    int32_t count = 0;
    if( _tl && _tl->alive() )
        count++;
    if( _tt && _tt->alive() )
        count++;
    if( _tr && _tr->alive() )
        count++;
    if( _lt && _lt->alive() )
        count++;
    if( _rt && _rt->alive() )
        count++;
    if( _bl && _bl->alive() )
        count++;
    if( _bb && _bb->alive() )
        count++;
    if( _br && _br->alive() )
        count++;
    return count;
}

void Cell::kill()
{
    _willBeAlive = false;
}

void Cell::resurrect()
{
    _willBeAlive = true;
}

void Cell::toggleState()
{
    _willBeAlive = !_willBeAlive;
}

void Cell::grimReaper()
{
    if (_willBeAlive == true && _alive == false) {
        _alive = true;
        setColor(colorAlive);
    }
    else if (_willBeAlive == false && _alive == true) {
        _alive = false;
        setColor(colorDead);
    }
}

void CellManager::init(const uint32_t screenWidth, const uint32_t screenHeight, const uint32_t cellWidth, const uint32_t cellHeight, const int32_t columns, const int32_t rows)
{
    // figure out cell width
    int32_t x_pos = 0, y_pos = 0;

    // allocate memory
    _cells = new Cell*[columns];
    for (int32_t a = 0; a < columns; a++)
        _cells[a] = new Cell[rows];

    // set up some vars
    _screenWidth = screenWidth;
    _screenHeight = screenHeight;
    _rows = rows;
    _columns = columns;

        // determine neighbors
    for(int32_t y = 0; y < _rows; y++ ) {
        for (int32_t x = 0; x < _columns; x++) {
            Cell *tl, *t, *tr, *l, *r, *bl, *b, *br;
            tl = ( x - 1 < 0 || y - 1 < 0 ? NULL : &_cells[x - 1][y - 1] );
            t =  ( y - 1 < 0 ? NULL : &_cells[x][y - 1] );
            tr = ( x + 1 >= _columns || y - 1 < 0 ? NULL : &_cells[x + 1][y - 1] );
            l =  ( x - 1 < 0 ? NULL : &_cells[x - 1][y] );
            r =  ( x + 1 >= _columns ? NULL : &_cells[x + 1][y] );
            bl = ( x - 1 < 0 || y + 1 >= _rows ? NULL : &_cells[x - 1][y + 1] );
            b =  ( y + 1 >= _rows ? NULL : &_cells[x][y + 1] );
            br = ( x + 1 >= _columns || y + 1 >= _rows ? NULL : &_cells[x + 1][y + 1] );

            // set up all cells
            _cells[x][y].setDims( cellWidth, cellHeight );
            _cells[x][y].setNeighbors( tl, t, tr, l, r, bl, b, br );
            _cells[x][y].setPosition( x_pos, y_pos );
            _cells[x][y].init();
            _cells[x][y].kill();

            x_pos += cellWidth + 1;
        }
        x_pos = 0;
        y_pos += cellWidth + 1;
    }
}

void CellManager::run(SDL_Surface * screen)
{
    // Create events
    SDL_Event event;
    int x, y, frames = 0;
    uint8_t mouseState;
    uint32_t ticks = SDL_GetTicks(), generations = 0, alive = 0;
    bool go = false;
    SDL_PollEvent( &event );
    srand(time(NULL));
    int centerx = _rows / 2;
    int centery = _columns / 2;
    // Loop until quit
    while (event.type != SDL_QUIT) {
        if (event.type == SDL_KEYDOWN && SDL_GetTicks() - ticks >= 100) {
            if (event.key.keysym.sym == SDLK_ESCAPE) {
                SDL_Event quit;
                quit.type = SDL_QUIT;
                if (SDL_PushEvent(&quit) != 0)
                    exit(1);
            }
            else if(event.key.keysym.sym == SDLK_SPACE || event.key.keysym.sym == SDLK_RETURN)
                go = !go;
            else if(event.key.keysym.sym == SDLK_r) {
                generations = 0;
                for (int32_t x = 0; x < _columns; x++) {
                    for(int32_t y = 0; y < _rows; y++ ) {
                        _cells[x][y].kill();
                    }
                }
            }
            else if(event.key.keysym.sym == SDLK_TAB) {
                for (int32_t x = 0; x < _columns; x++) {
                    for(int32_t y = 0; y < _rows; y++ ) {
                        int random = rand() % 10000;
                        if( random < 5000 )
                            _cells[x][y].kill();
                        else if( random >= 5000 )
                            _cells[x][y].resurrect();
                    }
                }
            }
            else if(event.key.keysym.sym == SDLK_f) {
                for (int32_t x = 0; x < _columns; x++) {
                    for(int32_t y = 0; y < _rows; y++ ) {
                        _cells[x][y].toggleState();
                    }
                }
            }
            else if(event.key.keysym.sym == SDLK_c) {
                colorAlive = rand();
                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        if(_cells[x][y].alive())
                            _cells[x][y].setColor(colorAlive);
                        else
                            _cells[x][y].setColor(colorDead);
                    }//end for y.
                }//end for x.
            }//end else.

            else if(event.key.keysym.sym == SDLK_o)
            {
                colorAlive = origionalAlive;
                colorDead = origionalDead;
                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        if(_cells[x][y].alive())
                            _cells[x][y].setColor(colorAlive);
                        else
                            _cells[x][y].setColor(colorDead);
                    }//end for y.
                }//end for x.
            }//end else

            else if(event.key.keysym.sym == SDLK_g)
            {
                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        _cells[x][y].kill();
                    }//end for y.
                }//end for x.

                _cells[centerx][centery].resurrect();
                _cells[centerx + 1][centery - 1].resurrect();
                _cells[centerx + 2][centery - 1].resurrect();
                _cells[centerx + 2][centery].resurrect();
                _cells[centerx + 2][centery + 1].resurrect();
                go = false;
            }//end else.

            else if(event.key.keysym.sym == SDLK_n)
            {
                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        _cells[x][y].kill();
                    }//end for y.
                }//end for x.

                _cells[centerx - 1][centery - 1].resurrect();
                _cells[centerx - 2][centery - 1].resurrect();
                _cells[centerx - 2][centery].resurrect();
                _cells[centerx - 2][centery - 2].resurrect();
                _cells[centerx - 3][centery + 1].resurrect();
                _cells[centerx - 3][centery - 3].resurrect();
                _cells[centerx - 4][centery - 1].resurrect();
                _cells[centerx - 5][centery + 2].resurrect();
                _cells[centerx - 5][centery - 4].resurrect();
                _cells[centerx - 6][centery + 2].resurrect();
                _cells[centerx - 6][centery - 4].resurrect();
                _cells[centerx - 7][centery - 3].resurrect();
                _cells[centerx - 7][centery + 1].resurrect();
                _cells[centerx - 8][centery].resurrect();
                _cells[centerx - 8][centery - 1].resurrect();
                _cells[centerx - 8][centery - 2].resurrect();
                _cells[centerx - 18][centery - 1].resurrect();
                _cells[centerx - 19][centery - 1].resurrect();
                _cells[centerx - 18][centery - 2].resurrect();
                _cells[centerx - 19][centery - 2].resurrect();
                //the left half is now drawn.
                _cells[centerx + 2][centery - 2].resurrect();
                _cells[centerx + 2][centery - 3].resurrect();
                _cells[centerx + 2][centery - 4].resurrect();
                _cells[centerx + 3][centery - 2].resurrect();
                _cells[centerx + 3][centery - 3].resurrect();
                _cells[centerx + 3][centery - 4].resurrect();
                _cells[centerx + 4][centery - 5].resurrect();
                _cells[centerx + 4][centery - 1].resurrect();
                _cells[centerx + 6][centery - 5].resurrect();
                _cells[centerx + 6][centery - 6].resurrect();
                _cells[centerx + 6][centery - 1].resurrect();
                _cells[centerx + 6][centery].resurrect();
                _cells[centerx + 15][centery - 4].resurrect();
                _cells[centerx + 15][centery - 3].resurrect();
                _cells[centerx + 16][centery - 4].resurrect();
                _cells[centerx + 16][centery - 3].resurrect();

                go = false;
            }

            else if(event.key.keysym.sym == SDLK_v) {
                colorAlive = 0xdd4814;
                colorDead = 0x3c3b37;
            }

            else if(event.key.keysym.sym == SDLK_F12) {
                colorDead = rand();
                colorAlive = rand();
            }

            else if(event.key.keysym.sym == SDLK_q)
            {
                colorDead = rand();
                colorAlive = rand();

                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        if(_cells[x][y].alive())
                            _cells[x][y].setColor(colorAlive);
                        else
                            _cells[x][y].setColor(colorDead);
                    }//end for y.
                }//end for x.
            }//end else

            else if(event.key.keysym.sym == SDLK_d)
            {
                colorDead = rand();

                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        if(_cells[x][y].alive())
                            _cells[x][y].setColor(colorAlive);
                        else
                            _cells[x][y].setColor(colorDead);
                    }//end for y.
                }//end for x.
            }//end else.

            else if(event.key.keysym.sym == SDLK_m)
            {
                colorDead = colorDeadm;
                colorAlive = colorAlivem;

                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        if(_cells[x][y].alive())
                            _cells[x][y].setColor(colorAlive);
                        else
                            _cells[x][y].setColor(colorDead);
                    }//end for y.
                }//end for x.
            }//end else.

            else if(event.key.keysym.sym == SDLK_w)
            {
                colorDead = colorDeadw;
                colorAlive = colorAlivew;

                for (int32_t x = 0; x < _columns; x++)
                {
                    for (int32_t y = 0; y < _rows; y++)
                    {
                        if(_cells[x][y].alive())
                            _cells[x][y].setColor(colorAlive);
                        else
                            _cells[x][y].setColor(colorDead);
                    }//end for y.
                }//end for x.
            }//end else.
            ticks = SDL_GetTicks();
        }//end if

        mouseState = SDL_GetMouseState(&x, &y);

        if (mouseState & SDL_BUTTON(SDL_BUTTON_LEFT)) {
            x /= (boxWidthInPx + 1);
            y /= (boxHeightInPx + 1);

            _cells[x][y].resurrect();
        }

        if (mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT)) {
            x /= (boxWidthInPx + 1);
            y /= (boxHeightInPx + 1);

            _cells[x][y].kill();
        }

        // Draw EVERYTHING. Run the game!
        for (int32_t x = 0; x < _columns; x++) {
            for(int32_t y = 0; y < _rows; y++ ) {
                //These are the rules of the game. Apply them if necessary!
                if( go )
                {
                    int32_t liveNeighbors = _cells[x][y].numLiveNeighbors();
                    if (_cells[x][y].alive() && (liveNeighbors < 2 || liveNeighbors > 3))
                        _cells[x][y].kill();
                    else if (!_cells[x][y].alive() && liveNeighbors == 3)
                        _cells[x][y].resurrect();
                }
            }
        }

        // Draw it
        for (int32_t x = 0; x < _columns; x++)
        {
            for(int32_t y = 0; y < _rows; y++ )
            {
            _cells[x][y].grimReaper();
            if( _cells[x][y].alive())
                alive++;
            _cells[x][y].drawMe(screen);
            }
        }

        // Flip screen and poll events
        SDL_Flip(screen);
        SDL_PollEvent( &event );

        // Update FPS display
        frames++;

        // Update generations
        if( go )
            generations++;

        // If the time has exceeded 1 second, update the FPS display.
        if(SDL_GetTicks() - ticks >= 200)
        {
            ostringstream fps;
            fps << programName << " (FPS: " << frames * 5 << ", generations: " << generations << ", alive: " << alive << ')';
            SDL_WM_SetCaption( fps.str().c_str(), NULL );
            frames = 0;
            ticks = SDL_GetTicks();
        }
        alive = 0;
    }
}

void CellManager::release()
{
    for (int32_t a = 0; a < _columns; a++)
        delete[] _cells[a];

    delete[] _cells;
}
