import java.util.ArrayList;
import java.util.Random;
import hsa.*;
/**
 * Write a description of class PoemClass here.
 * 
 * @author (your name) 
 * @version (a version number or a date)
 */
public class PoemClass
{
    private final int NUM_LINES = 10;
    private ArrayList<String> poem;
    private TextInputFile main;
    private ArrayList<TextInputFile> input;
    private TextOutputFile output;
    
    public void main()
    {
        main = new TextInputFile("main.txt");
        poem = new ArrayList();
        input = new ArrayList();
        readFile();
        fillInFiles();
        
        for (int x = 0; x < NUM_LINES; x++)
            fillLine(x);
        outputFile();
    }//end main.
    
    public void readFile()
    {
        while (!main.eof())
            poem.add(main.readLine());
    }//end void.
    
    public void fillInFiles()
    {
        for (int x = 0; x < NUM_LINES; x++)
            input.add(setInFile("line"+(x + 1)+".txt"));
    }//fill arrayLIst of input files.
    
    public void outputFile()
    {
        setOutFile("RandomPoem.txt");
        
        for (int x = 0; x < poem.size(); x++)
            output.println(poem.get(x));
    }//end void.
    
    public void fillLine(int lineNum)
    {
        String line = poem.get(lineNum);
        line += " "+getEnding(lineNum); 
        poem.set(lineNum, line);
    }//fill up a line.
    
    public String getEnding(int lineNum)
    {
        Random rand = new Random();
        int line = rand.nextInt(NUM_LINES);
        int x = 0; 
        String line2 = "";
        
        while (!input.get(lineNum).eof() && x != line || x == 0)
        {
            line2 = input.get(lineNum).readLine();
            x++;
        }//end while
        
        return line2;
    }//get a random line ending.
    
    public TextInputFile setInFile(String fileName){ return new TextInputFile(fileName); }//set in file.
    public void setOutFile(String fileName){ output = new TextOutputFile(fileName); }//set out file.
}
