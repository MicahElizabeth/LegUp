using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Optimization
{
    public class DecisionVariable
    {
        private string _name;
        private double _optimalValue;
        private double _objectiveCoeff;


        public string Name { get { return _name; } }
        public double Value { get { return _optimalValue; } set { _optimalValue = value; } }
        public double ObjectiveCoeff { get { return _objectiveCoeff; } set { _objectiveCoeff = value; } }

        //constructor
        public DecisionVariable(string name)
        {
            _name = name;
            _optimalValue = 0.0;
            _objectiveCoeff = 0.0;
        }
        //copy constructor
        public DecisionVariable(DecisionVariable x)
        {
            _name = x._name;
            _optimalValue = x._optimalValue;
            _objectiveCoeff = x._objectiveCoeff;
        }
    }

    
}
