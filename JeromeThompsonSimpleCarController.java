import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Rectangle2D.Double;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

public class JeromeThompsonSimpleCarController implements CarController {

	// Angular velocities.
    //double mu1=0, mu2=0, phi = 0, vel = 1000;
	double  phi = 0, vel = 1000, acc;
	int index;
	
	int offsetSize = 0;
	double lenOfCar = 40+offsetSize, widthOfCar = 20+ offsetSize;
    
	// Is the first control an accelerator?
    boolean isAccelModel = false;
    
	LinkedList<State> frontier;
	LinkedList<State> visitedStates;
	int numMoves;
	static int maxMoves = 100000;
	int numNodes = 8;
	double deltaAngle = Math.PI/4;
	double delta = 20;
	
	ArrayList<Rectangle2D.Double> obstacles;
	SensorPack sensors;
	private double initX;
	private double initY;
	private double endX;
	private double endY;
	
	CarState currentState;
	CarState start;
    LinkedList<State> plan;
    Iterator<State> planIterator;
    
    // For equality comparisons.
    static double epsilon = 0.01;
    //TODO
     
    
	@Override
	public void init(double initX, double initY, double initTheta, double endX, double endY, double endTheta,
			ArrayList<Rectangle2D.Double> obstacles, SensorPack sensors) {
		// TODO Auto-generated method stub
		this.obstacles = obstacles;
		this.sensors = sensors;
		this.initX =initX;
		this.initY =initY;
		this.endX =endX;
		this.endY =endY;
		start = new CarState(null, initTheta, initX, initY);
		currentState = start;
		index = 0;
		plan = makePlan ( );
		
	}
	
	@Override
	public void draw(Graphics2D g2, Dimension D) {
		// TODO Auto-generated method stub
		//System.out.println("aosuhrgjasejgruyasegrhisuyer");
		
	}

	@Override
	public void move()  {
		// TODO Auto-generated method stub
		//SimpleCarSimulator carSim;
		//carSim = new SimpleCarSimulator (false, true);
		/*
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
		
		double angleDelTa = .04;
		double distanceDelTa = 4.0;
		boolean stay = true;
		double changeOfAngle = Math.PI/4;
		
		
			
			if(index < this.plan.size() && this.plan.size() > 0)
			{
				CarState state = (CarState) this.plan.get(index);
				//if (lessThanOrBothSo (sensors.getTheta() , state.angle , angleDelTa) )
				System.out.println("index: " + index + " sensors.getTheta(): " + sensors.getTheta()  + " XY: " + sensors.getX() + " - "+ sensors.getY() + " statedddd: " + state + " bool: " + lessThanOrBothSo(sensors.getTheta() , state.angle,  angleDelTa));
				//while(stay)
				//{
					
					//if(sensors.getTheta() != state.angle )
				if (! lessThanOrBothSo (sensors.getTheta() , state.angle , angleDelTa) )
					{
						
						//phi = state.angle; 
						phi = changeOfAngle;
						vel = 0; 
						System.out.println("sfadfsdfasd: " + state.x + " - " + state.y);
					}
				
				    else if( ! lessThanOrBothSo(sensors.getX(), state.x , distanceDelTa) || !lessThanOrBothSo (sensors.getY(),state.y , distanceDelTa))
					{
						phi = 0; 
						vel = 10; 
						System.out.println("dfohaiuyriuhuakjertmaeklr: " + state.x + " - " + state.y);
					}
					else
					{
						//stay = false;
						System.out.println("SET ME FREE");
						index++;
					}
						
				//}
			}
			
		//System.out.println("sensors.getTheta(): " + sensors.getTheta() + " state.angle: " + state.angle);
		
	}

	public boolean lessThanOrBothSo(double x, double y, double delta)
	{
		return (x  <= y + delta ) && (x >= y - delta);
	}
	
	@Override
	public double getControl(int i) {
		if (i == 1) {
		    if (isAccelModel) {
			return acc;
		    }
		    else {
			return vel;
		    }
		}
		else if (i == 2) {
		    return phi;
		}
		return 0;
	}
	
	public boolean satisfiesGoal (State state)
    {
		CarState c = (CarState) state;
        double x = c.x;
        double y = c.y;
        // Use a distance of at least delta.
        if ( (Math.abs(endX-x) > delta+epsilon) || (Math.abs(endY-y) > delta+epsilon) ) {
            return false;
        }
        return true;
    }
	
	
	public LinkedList<State> makePlan ()
    {
        // Initialize.
		frontier = new LinkedList<State> ();
	 visitedStates = new LinkedList<State> ();
	numMoves = 0;

        // The start node is the first one to place in frontier.
	frontier.add (start);

	while (numMoves < maxMoves) {

            // If nothing to explore, we're done.
	    if (frontier.size() == 0) {
		break;
	    }

	    // Get first node in frontier and expand.
	    State currentState = removeBest ();
            // problem.drawState (currentState);

            // If we're at a goal node, build the solution.
	    if (satisfiesGoal (currentState)) {
		return makeSolution (currentState);
	    }

	    numMoves ++;

	    // Put in visited list.
	    visitedStates.add (currentState);

	    // Expand current state (look at its neighbors) and place in frontier.
	    ArrayList<State> neighbors = getNeighbors (currentState);
	    for (State s: neighbors) {
		if ( ! visitedStates.contains (s) ) {
                    int index = frontier.indexOf (s);
                    if (index >= 0) {
                        State altS = frontier.get (index);
                        if (s.costFromStart < altS.costFromStart) {
                            frontier.set (index, s);
                        }
                    }
                    else {
                        frontier.add (s);
                    }
		}
	    }

	    if (numMoves % 100 == 0) {
		System.out.println ("After " + numMoves + ": |F|=" + frontier.size() + "  |V|=" + visitedStates.size());
	    }

	} // endwhile

	System.out.println ("Cost-based: No solution found after " + numMoves + " moves");
	return null;
    }

	public LinkedList<State> makeSolution (State goalState)
    {
	LinkedList<State> solution = new LinkedList<State> ();
	solution.add (goalState);
	//System.out.println("goal: " + goalState);
        // Start from the goal and work backwards, following
        // parent pointers.
	State currentState = goalState;
	while (currentState.getParent() != null) {
		System.out.println("currentState: " + currentState);
	    solution.addFirst (currentState.getParent());
	    currentState = currentState.getParent();
	    
	}

	System.out.println ("Cost: Solution of length=" + solution.size() + " found with cost=" + goalState.costFromStart + " after " + numMoves + " moves");

	return solution;
    }    
	
	
	
	public ArrayList<State> getNeighbors (State state)
    {
        CarState c = (CarState) state;
        ArrayList<State> neighbors = new ArrayList<State> ();
        double angle, angleParent;
        
        /*
        if(state.getParent() == null)
        	angleParent = 0;
        else
        {
        	CarState parentState = (CarState) state.getParent();
        	angleParent = parentState.angle;
        }
        */
        
        //angle = c.angle;
        //double x = c.x;
        //double y = c.y;
        angle = 0;
        // Look at the 8 neighboring points at distance (+- delta, +-delta).
        double val = Math.cos(Math.PI/4 );
        //for (int i=numNodes-1; i>1; i--) {
        for (int i=0; i<numNodes; i++) {
        		double y = 0, x = 0;
        		if(angle == 0)
        		{
        			y = c.y;
        			x = c.x + delta;
        		}
        		else if(angle == Math.PI/4 )
        		{
        			//x = c.x + delta;
        			//y = c.y + delt
        			
        			x = c.x + val*delta;
        			y = c.y + val*delta;
        		}
        		else if(angle == Math.PI/2 )
        		{
        			x =c.x;
        				y = c.y + delta;
        		}
        		else if(angle == 3*Math.PI/4 )
        		{
        				y = c.y +  val*delta;
        				x = c.x -  val*delta;
        		}
        		else if(angle == Math.PI )
        		{
        				y = c.y;
        				x =c.x- delta;
        		}
        		else if(angle == 5*Math.PI/4 )
        		{
        				
        				x = c.x -  val*delta;
        				y = c.y -  val*delta;
        		}
        		else if(angle == 3*Math.PI/2 )
        		{
        				
        				x = c.y;
        				y = c.y - delta;
        		}
        		else if(angle == 7*Math.PI/4 )
        		{
        				
        			x = c.x +  val*delta;
        				y = c.y -  val*delta;
        		}
        		
                    CarState b = 	new CarState (c,angle, x,  y);
                    if (isValid (b)) {
                        b.costFromStart = b.distanceFromStart(this.initX, this.initY);
                        b.estimatedCostToGoal = b.tipDistance (this.endX, this.endY);
                        neighbors.add (b);
                        
                    }
                    angle += Math.PI/4;
        }
        
        return neighbors;
    }
	
        
    
	
	boolean isValid (CarState c)
    {
        if (c == null) {
            return false;
        }

        // See if within bounds.
        
            double x = c.x ;
            double y = c.y;
            if ( (x < 0) || (y < 0) || (x > this.endX && y > this.endY) )
            {
                return false;
            }
            
        

        // See if the new state hits the obstacle.
        //Dimension D = this.getSize();
            /*
		for(int count = 0; count < this.obstacles.size(); count++)
		{
			Rectangle2D.Double R = obstacles.get(count);
			if (R.intersectsLine (this.currentState.x,this.currentState.y, this.currentState.x + delta,this.currentState.y+ delta)) {
                return false;
            }
			//return false;
        }
        */
            SimpleCarSimulator carSim = new SimpleCarSimulator (false, true);
            //carSim.init (c.x+this.lenOfCar/2, c.y + this.widthOfCar/2, c.angle, this.obstacles);
            //carSim.init (c.x, c.y, c.angle, this.obstacles);
            carSim.init (c.x+this.widthOfCar/2, c.y+this.widthOfCar/2, c.angle, this.obstacles);
            
            if(carSim.hitObstacle())
            	return false;
            
            carSim.init (c.x-this.widthOfCar/2, c.y-this.widthOfCar/2, c.angle, this.obstacles);
            
            if(carSim.hitObstacle())
            	return false;
        //** Note: we have not checked whether a link crosses the
        // rest of the arm.

        return true;
    }
        
    State removeBest ()
    {
        // INSERT YOUR CODE HERE
	// Pick the state s with the least s.costFromStart
	double cost = Integer.MAX_VALUE;
	State state = null;

	for(int x=0; x< frontier.size(); x++)
	{
		State thisState = frontier.get (x);
		//Below is is more complete heurtistic but for examples too a bit loger so used just estimated caost
		//double thisCost = thisState.costFromStart + thisState.estimatedCostToGoal ;
		double thisCost =  thisState.estimatedCostToGoal ;
		if(thisCost < cost)
		{
			state = thisState;
			cost = thisCost;
			
		}
	}
	//TODO
	//System.out.println(state);
	frontier.remove(state);
	return state;
    }

}
