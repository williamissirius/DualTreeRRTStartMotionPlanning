#include <vector>
#include <cmath>
#include <iostream>

using namespace std;
using namespace OpenRAVE;

typedef std::vector<double> Configuration;

class RRTNode
{
  Configuration _config;
  int _parentID;
  
  public:
		RRTNode(const Configuration &config)
		{
    		_config = config;
    		_parentID = -1;	
		}
	    void setParent(int parent)
		{
			_parentID = parent;
		}

		int getParent()
   		{
    		return _parentID;
   		}	

	  	Configuration getConfig()
	  	{
  			return _config;
  		}
};


class NodeTree
{
  EnvironmentBasePtr env;
  int _TotNode;
  int _configSize;
  bool isConnected;
  vector<double>  _GoalConfig;
  float _StepSize;
  vector<double> weight;

  vector<RRTNode> strTree;

  vector<RRTNode> goalTree;

public:
  NodeTree(EnvironmentBasePtr envir,Configuration str,Configuration goal, float step)
  {
      
      RRTNode currentNode(str);
      strTree.push_back(currentNode);
      RRTNode currentNode2(goal);
      goalTree.push_back(currentNode2);

      _configSize = currentNode.getConfig().size();
      isConnected = false;

      _StepSize = step;
      env = envir;
      
    weight = str;
     weight[0] = 2;
      weight[1] = 2;
       weight[2] = 2;
        weight[3] = 0.5;
         weight[4] = 0.5;
          weight[5] = 0.5;


  }

  bool FindGoal()
  {
  	return isConnected;
  }
  void biExtend(const Configuration &config)
  {
  	
  	if (strTree.size() <= goalTree.size())
  	{
  		int str_nnID =  NearestNode(config, strTree);
  		int lastID_str = add(config,strTree,str_nnID);
  		int goal_nnID = NearestNode(strTree[lastID_str].getConfig(), goalTree);
  		int lastID_goal = add(strTree[lastID_str].getConfig(),goalTree,goal_nnID);
  			if (ConnectCheck(lastID_str,lastID_goal))
	    	{
						cout<<"	"<< "goal Found!!!!"<< endl;
						isConnected =true;
						return;
	    	}

  	}
  	else
  	{
  		
  		int goal_nnID =  NearestNode(config, goalTree);
  		int lastID_goal = add(config,goalTree,goal_nnID);
  		int str_nnID = NearestNode(goalTree[lastID_goal].getConfig(), strTree);
  		int lastID_str = add(goalTree[lastID_goal].getConfig(),strTree,str_nnID);

   			if (ConnectCheck(lastID_str,lastID_goal))
	    	{
						cout<<"	"<< "goal Found!!!!"<< endl;
						isConnected =true;
						return;
	    	}
  	}
  }
  int add(Configuration config,vector<RRTNode> &_nodes,int ID)
  {
        int nnID = ID;
       	int lastID;
       	lastID = ID;
		if (!withinStepSize(nnID,config,_nodes))
		{
			int TotSteps = distance(_nodes[nnID].getConfig(),config)/_StepSize;
	    	float TotStepsPos = (TotSteps*_StepSize)/distance(_nodes[nnID].getConfig(),config); 
			
			for (int step = 1; step <= TotSteps; step++)
	    	{
		        Configuration newConfig( _configSize);
		        for (int i = 0; i <  _configSize; ++i)
		        	newConfig[i] = _nodes[nnID].getConfig()[i] + ((config[i]-_nodes[nnID].getConfig()[i])*TotStepsPos*step)/TotSteps;

		        if (!CollideCheck(newConfig))
		        {

		        	RRTNode newNode( newConfig);
		        	newNode.setParent(nnID);
		        	_nodes.push_back(newNode);
		        	lastID ++;
	    	        nnID = _nodes.size()-1; 
	    	        
	    	    }
	    	    else
	    	    {
	    	    	return lastID;		    	 
	    	    }
	    	}
		}
			
		if (!CollideCheck(config))
	    {

	    	RRTNode newNode(config);
	    	newNode.setParent(nnID);
	    	lastID ++;
	    	_nodes.push_back(newNode);

	    }
	    else
	    {
	    	return lastID ;
		}
      return lastID ;
  }
  
  
   int NearestNode(Configuration sampleConfig, vector<RRTNode> nodes)
	{
		float dist = 100000.0;
		int index = -1;

	    for (int i = 0; i < nodes.size(); i++)
		{
        	if (distance(nodes[i].getConfig() , sampleConfig) < dist)
			{
					if (dist <= 0.01)
					{
						index = i;
						break;
					}
					else
					{
						dist =distance(nodes[i].getConfig() , sampleConfig);
						index = i;
					}
				}

			}
			return index;
		} 
		
  double distance(const Configuration &a , const Configuration &b)
	{
      		double dist = 0.0;
          int langth = a.size();
      		for (int i = 0; i < langth; ++i)
      			dist += weight[i]*(a[i] - b[i]) * (a[i] - b[i]);
      
      		return sqrt(dist);
	}
 
  bool CollideCheck(const Configuration &sampleConfig)
  {
        	env->GetRobot("Quadrotor")->SetActiveDOFValues(sampleConfig);
		    if ( env->CheckCollision(env->GetRobot("Quadrotor")) || env->GetRobot("Quadrotor")->CheckSelfCollision() )
		      	return true;
		    else
		       	return false;
  }

  bool ConnectCheck( const int &strID, const int &goalID)
	{
	  int length = strTree[strID].getConfig().size();
	  for (int i = 0; i < length; i++)
	  {
	  	if( (strTree[strID].getConfig()[i] != goalTree[goalID].getConfig()[i]) )
	  	{
	  		return false;
	  	}
	  }
	  return true;
	}
	
	bool withinStepSize(int nnID,const Configuration &config, vector<RRTNode> _nodes)
	{
		
		if ( distance( _nodes[nnID].getConfig(),config) > _StepSize)
			return false;
		else
			return true;
	}
	vector<Configuration> GivePath()
	{
		int ID =  strTree.size()-1;
		vector<Configuration> path1;
		vector<Configuration> path2;

		while(ID >=0)
		{
			path1.push_back( strTree[ID].getConfig());
			ID = strTree[ID].getParent();
		}


		ID =  goalTree.size()-1;
		while(ID >=0)
		{
			path2.push_back( goalTree[ID].getConfig());
			ID = goalTree[ID].getParent();
		}
		vector<Configuration> path;
		reverse(path1.begin(),path1.end());
		path.insert(path.end(),path1.begin(),path1.end());
		path.insert(path.end(),path2.begin(),path2.end());
		
		return path;
		
	}

};


