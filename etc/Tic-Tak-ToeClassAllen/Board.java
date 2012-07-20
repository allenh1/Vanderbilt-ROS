
/**
 * All data and code related to a tic-tac-toe board.
 * 
 * @author Hunter Allen 
 * @version November 16, 2010
 */
public class Board
{    
    public Square[] Board = new Square[9];
    
    public Board()
    {
        for (int x = 0; x < 9; x++)
        {
            Board[x] = new Square();
        }//end for x.
    }//constructor for a new board.
    
    public void setX(int cell)
    {
        Board[cell].makeX();
    }//set square as an X.
   
    public void setO(int square)
    {
       Board[square].makeO();
    }//set the square as an O.
    
    public void showBoard()
    {
       System.out.println("|"+Board[0].getVal()+"|"+Board[1].getVal()+"|"+Board[2].getVal()+"|");
       System.out.println("-------");
       System.out.println("|"+Board[3].getVal()+"|"+Board[4].getVal()+"|"+Board[5].getVal()+"|");
       System.out.println("-------");
       System.out.println("|"+Board[6].getVal()+"|"+Board[7].getVal()+"|"+Board[8].getVal()+"|");
    }//print out the board.
    
    public boolean hasSameThree(String Side, int a, int b, int c)
    {
        if (Board[a].getVal() == Side && Board[b].getVal() == Side && Board[c].getVal() == Side)
            return true;
        else
            return false;
    }
    
    public boolean isFull()
    {
        int cells = 0;
        
        for (int x = 0; x < 9; x++)
        {
            if (Board[x].getVal() != " ")
            {
                cells++;
            }//end if. 
        }
        
        if (cells == 9)
            return true;
        else 
            return false;       
    }//checks to see if the board is full.
    
    
    /**
     * The following booleans check
     * for a winning situation.
     * The layout for the board is as follows:
     * 
     * |0|1|2|
     * -------
     * |3|4|5|
     * -------
     * |6|7|8|
     */
    public boolean checkRowsWin(String side)
    {
        if (hasSameThree(side, 0, 1, 2))
        {
            return true;
        }//checks the first row.
        
        if (hasSameThree(side, 3, 4, 5))
        {
            return true;
        }//checks the second row.
        
        if (hasSameThree(side, 6, 7, 8))
        {
            return true;
        }
        
        else    
            return false;
    }//checks all of the rows for a win.
    
    public boolean checkColsWin(String side)
    {
        if (hasSameThree(side, 0, 3, 6))
            return true;
        if (hasSameThree(side, 1, 4, 7))
            return true;
        if (hasSameThree(side, 2, 5, 8))
            return true;
        else
            return false;
    }//checks all the columns for a win.
    
    public boolean checkDiagWin(String side)
    {
        if (hasSameThree(side, 0, 4, 8))
            return true;
        if (hasSameThree(side, 2, 4, 6))
            return true;
        else
            return false;
    }//checks both diags for a win.
    
    public boolean checkForWin(String side)
    {
        if (checkDiagWin(side))
            return true;
        if (checkColsWin(side))
            return true;
        if (checkRowsWin(side))
            return true;
        else    
            return false;
    }//checks for a win.
}//end class.
