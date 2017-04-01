#include <openrave/plugin.h>
#include <openrave/planningutils.h>
#include <boost/bind.hpp>

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

 #include "myRRT.h";


using namespace OpenRAVE;
using namespace std;


std::vector<double> _upper;
std::vector<double> _lower;



class myRRT : public ModuleBase
{

    // vector<double> _StrConfig;
    // vector<double> _GoalConfig;
    float _GoalBias;
    float _StepSize;
    bool _biFlag ;
    int _MaxIteration;

    int _configSize;
    vector<double> _lower ;
    vector<double> _upper ;
    vector<double> _StrConfig;
    vector<double> _GoalConfig;

        vector<Configuration> path;

public:
    myRRT(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("SetStrGoal",boost::bind(&myRRT::SetStrGoal,this,_1,_2),"This is an example command");

        RegisterCommand("SetPara",boost::bind(&myRRT::SetPara,this,_1,_2),"This is an example command");

        RegisterCommand("FindPath",boost::bind(&myRRT::FindPath,this,_1,_2),"find a path to goal");

        RegisterCommand("PathSmooth",boost::bind(&myRRT::PathSmooth,this,_1,_2),"smooth path");
          
    }

    virtual ~myRRT() {}
    
    bool SetStrGoal(std::ostream& sout, std::istream& sinput)
    {
        // srand(time(NULL));
        std::string input;
        
        char temp = '1';
        sinput >> input;
        float q;

        if (input == "str")
            while (temp!= ';')
            {
                sinput >> q;
                _StrConfig.push_back(q);
                sinput>>temp;
            }

        sinput >> input;
        temp = '1';
        if (input == "goal")
            while(temp != ';')
            {
                sinput >> q;
                _GoalConfig.push_back(q);
                sinput>>temp;
            }
        
        return true;
    }
    bool SetPara(std::ostream& sout, std::istream& sinput)
    {
        char temp = '1';

        sinput >> _GoalBias;
        sinput >>temp;
        sinput >> _StepSize;
        cout<<"goalbias is: "<<_GoalBias<<endl;
        cout<<"step: "<<_StepSize<<endl;
        // cout<<"bidirection? : "<<_biFlag<<endl;
        // cout<<"MaxIteration : "<<_MaxIteration<<endl;

        GetEnv()->GetRobot("Quadrotor")->GetActiveDOFValues(_lower);
        _upper = _lower;
        _lower[0] = -4.9;
        _lower[1] = -2.4;
        _lower[2] =  0.1;
        _lower[3] = _lower[4] = _lower[5] = -3.14;

        _upper[0] = 4.9;
        _upper[1] = 2.0;
        _upper[2] =  2;
        _upper[3] = _upper[4] = _upper[5] = 3.14;

        _configSize =  _lower.size();


        return true;
    }

    bool FindPath(std::ostream& sout, std::istream& sinput)
    {
    
         NodeTree RRTtree( GetEnv(),_StrConfig,_GoalConfig,_StepSize);
    
          srand((int)time(0));
          int counter=0;
      cout<<"biTREE"<<endl;    
          do{
            vector<double> temp =  sampleConfig();  
            RRTtree.biExtend(temp);
            // cout<<counter<<endl;
            //             counter ++;
          }while(!RRTtree.FindGoal());
           path = RRTtree.GivePath();

          // RobotBasePtr robot = GetEnv()->GetRobot("Quadrotor");


     for(unsigned int i=0;i<path.size();i++)
         {
         for (unsigned int j=0;j < 6 ;j++)
             {
             sout<< path[i][j];
             if (j !=5) sout<<",";
             }
             if (i !=path.size()-1) sout<<endl;
         } 
          return true;
    }

   
    Configuration sampleConfig()
    {
 
        int _configSize = _lower.size();

            Configuration sample(_configSize);

            for(int i = 0; i < _configSize; ++i) 
            {

                sample[i] =  ((float)rand()/RAND_MAX) * (_upper[i]- _lower[i]) +  _lower[i];
               
            }
            return sample;
        
    }
    bool PathSmooth(std::ostream& sout, std::istream& sinput)
    {

 
      float ssc_step = 0.1;
		for (int i = 0; i < 200; ++i)
		{
            if (path.size() <= 10)
                break;

            int sample1 = rand()%path.size();
            int sample2 = rand()%path.size();
 
            while ((sample1 == sample2) || (abs(sample1-sample2) <= 1) )
            {
                sample1 = rand()%path.size();
                sample2 = rand()%path.size();
            }

            int first = min(sample1, sample2);
            int second = max(sample1, sample2);

            // RRTNode firstNode(path[first]);

            if (distanceTo( path[first], path[second]) > ssc_step)
            {
                int TotSteps =distanceTo(path[first], path[second])/ ssc_step;  
                float TotStepsPos = (TotSteps * ssc_step)/distanceTo(path[first], path[second]); 
                
                vector<Configuration> stepPath;
                bool shortCutFailed = false;
                for (int step = 1; step <= TotSteps; step++)
                {
                    Configuration newStepConfig(6);
                    for (int d = 0; d < 6; d++)
                    {
                        newStepConfig[d] = path[first][d] + ( (path[second][d]-path[first][d])*TotStepsPos*step)/TotSteps;
                    }
                    if (inCollision(newStepConfig))
                    {
                        shortCutFailed = true;
                        break;
                    }
                    stepPath.push_back( newStepConfig);
                }

                if (shortCutFailed == false)
                {
                   path.erase(path.begin()+first, path.begin()+second+1);
        			     for (int step = 0; step < (TotSteps); step++)
               		 {
               			path.insert(path.begin()+first+step, stepPath[step]);
               		 }       
                }

                // cout << "path len: " << path.size() << endl;

            }

        }
          for(unsigned int i=0;i<path.size();i++)
         {
         for (unsigned int j=0;j < 6 ;j++)
             {
             sout<< path[i][j];
             if (j !=5) sout<<",";
             }
             if (i !=path.size()-1) sout<<endl;
          } 
        return true;
    }
  double distanceTo(const Configuration &a , const Configuration &b)
    {
      double weight[] =  { 1,1,1,0,0,0};
         double dist = 0.0;
          int langth = a.size();
            for (int i = 0; i < langth; ++i)
                dist += weight[i]*(a[i] - b[i]) * (a[i] - b[i]);
      
            return sqrt(dist);
    }
 
  bool inCollision(const Configuration &sampleConfig)
  {
            GetEnv()->GetRobot("Quadrotor")->SetActiveDOFValues(sampleConfig);
            if ( GetEnv()->CheckCollision(GetEnv()->GetRobot("Quadrotor")) ||GetEnv()->GetRobot("Quadrotor")->CheckSelfCollision() )
                return true;
            else
                return false;
  }

};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "myrrt" ) {
        return InterfaceBasePtr(new myRRT(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("myRRT");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}




