
/**
 * All code related to a tic-tac-toe game.
 * You can play a game person vs person
 * or a game cpu vs. cpu.
 * 
 * @author Hunter Allen
 * @version November 17, 2010
 */

import java.util.Random;

public class Game
{
    private int gameType;
    private String player1 = "Player1";
    private String player2 = "Player2";
    
    public Keyboard input = new Keyboard();
    public Board game = new Board();
    
    public void pregameStuff()
    {
        System.out.println("Welcome to Hunter's Tic-Tac-Toe game.");
        System.out.println("-------------------------------------");
        System.out.println();
        System.out.println("1. Human vs. Human");
        System.out.println("2. Cpu vs Cpu");
        System.out.println();
        System.out.print("Please enter your game type: ");
        gameType = input.readInt();
        if (gameType == 1)
        {
            System.out.print("Enter player 1 name: ");
            player1 = input.readString();
            if (player1 == "" || player1 == " ")
                player1 = "Player 1";
            System.out.println();
            System.out.print("Enter player 2 name: ");
            player2 = input.readString();
            if (player2 == "" || player2 == " ")
                player2 = "Player 2";
        }//end if.
    }//all the pregame stuff.
    
    public void showBoardKey()
    {
        System.out.println();
        System.out.println("Move Key: ");
        System.out.println();
        System.out.println("|0|1|2|");
        System.out.println("|3|4|5|");
        System.out.println("|6|7|8|");
        System.out.println();
    }//shows the key.
    
    public void playComputerVsComputer()
    {
        boolean haveFinishedGame = false;
        
        Random rnd = new Random();
        
        int count = 1;
        
        System.out.println(player1+" will go first.");
        System.out.println(player1+" is X's.");
        System.out.println(player2+" is O's.");  
        
        while (!haveFinishedGame)
        {
            boolean hasPickedX = false;
            boolean hasPickedO = false;
            
            showBoardKey();
            game.showBoard();
            System.out.println();
            if (game.checkForWin("X"))
            {
                System.out.print(player1+" wins!");
                break;
            }//end if.
            
            if (game.checkForWin("O"))
            {
                System.out.print(player2+" wins!");
                haveFinishedGame = true;;
            }//end if.
            
            if (game.isFull())
            {
                System.out.print("It's a cat's game...");
            }//end if.
            
            if (count % 2 != 0)
            {
                System.out.println(player1+" enter move choice: ");
                
                while (hasPickedX)
                {
                    int x = rnd.nextInt(8);
                    
                    if (!game.Board[x].isOccupied())
                    {
                        game.Board[x].makeX();
                        System.out.println();
                        break;
                    }//end if.
                }//end while true.
            }//end if.
            
            if (count % 2 == 0)
            {
                System.out.println(player2+" enter move choice: ");
                while (hasPickedO)
                {
                    int y = rnd.nextInt(8);
                    
                    if (!game.Board[y].isOccupied())
                    {
                        game.Board[y].makeO();
                        System.out.println();
                        break;
                    }//end if.
                }//end while true
            }//end if.
            
            count++;
        }//end while(1).
    }//play one game.
    
    public void playOneGame()
    {
        boolean haveFinishedGame = false;
        
        int count = 1;
        
        System.out.println(player1+" will go first.");
        System.out.println(player1+" is X's.");
        System.out.println(player2+" is O's.");  
        
        while (!haveFinishedGame)
        {
            showBoardKey();
            game.showBoard();
            System.out.println();
            
            if (game.checkForWin("X"))
            {
                System.out.print(player1+" wins!");
                haveFinishedGame = true;
            }//end if.
            
            if (game.checkForWin("O"))
            {
                System.out.print(player2+" wins!");
                haveFinishedGame = true;
            }//end if.
            
            if (game.isFull())
            {
                haveFinishedGame = true;
            }//end if.
            
            if (count % 2 != 0)
            {
                System.out.println(player1+" enter move choice: ");
                game.Board[input.readInt()].makeX();
                System.out.println();
            }//end if.
            
            if (count % 2 == 0)
            {
                System.out.println(player2+" enter move choice: ");
                game.Board[input.readInt()].makeO();
                System.out.println();
            }//end if.
            
            count++;
        }//end while(1).
    }//play one game.
        
        public void main()
        {
            pregameStuff();
            if (gameType == 1)
                playOneGame();
            else
            {
                playComputerVsComputer();
            }//end else. 
        }//end main.
}//end class.
