
import java.util.*;
import java.awt.*;
import java.awt.geom.*;



public class CarState extends State {

    
    
	CarState parent;
    double angle;
	double x;
	double y;


	public CarState (CarState parent, double angle, double x, double y)
    {
		this.parent = parent;
        this.angle = angle;
        this.x = x;
        this.y = y;
    }
    

    
    

    public boolean equals (Object obj)
    {
        if (! (obj instanceof CarState) ) {
            return false;
        }
        CarState c = (CarState) obj;
        if(c.x != x || c.y != y || c.angle != angle)
        	return false;
        return true;
    }
    

    public String toString ()
    {
        String str = "CarState: ";
        
        str += "angle: " + angle + " angleInDeg: " +  angle/Math.PI* 180 + " x: " + x + " y: " + y;
        return str;
    }



    // How far is (x,y) from the tip of the arm?

    public double tipDistance (double endX, double endY)
    {
        return distance (x,y, endX, endY);
    	//return 0;endX
    }
    
    
    public double distanceFromStart (double startX, double startY)
    {
        return distance (startX,startY, x, y);
    	//return 0;endX
    }
    

    double distance (double x1, double y1, double x2, double y2)
    {
	return Math.sqrt ( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
    }





	@Override
	public State getParent() {
		// TODO Auto-generated method stub
		return this.parent;
	}

}