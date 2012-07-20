
/**
 * This is the square class. This holds all info
 * for a single square on the board.
 * 
 * @author Hunter Allen
 * @version November 8, 2010
 */

public class Square
{
    private String val;
    
    public Square()
    {
        val = " ";
    }//sets up the square 
    
    public String getVal()
    {
        return val;
    }//gets the value for val
    
    public void makeX()
    {
        val = "X";
    }//change val to x
    
    public void makeO()
    {
        val = "O";
    }//change val to O
    
    public void makeBLANK()
    {
        val = " ";
    }//makes val a blank.
    
    public boolean isX()
    {
        if (val == "X")
        {
            return true;
        }//end if.
        
        else {
            return false;
            
        }//end else.
    }//returns true if val == x
    
    public boolean isO()
    {
        if (val == "O")
        {
            return true;
        }//end if.
        
        else {
            return false;
        }//end else.
    }//returns true if val == O.
    
    public boolean isOccupied()
    {
        if (isO() || isX())
            return true;
        else    
            return false;
    }//returns true if the square is occupied.
}//end class.
