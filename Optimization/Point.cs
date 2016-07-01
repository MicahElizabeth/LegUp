using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Optimization
{
    public class Point
    {
        private List<DecisionVariable> _point;
        private double _value;
        public double Value { get { return _value; } set { _value = value; } }
        public List<DecisionVariable> PointVars { get { return _point; } }

        //constructor
        public Point()
        {
            _point = new List<DecisionVariable>();
            _value = 0.0;

        }
        //copy constructor
        public Point(Point p)
        {
            _point = new List<DecisionVariable>(p.PointVars);
            _value = p._value;
        }

        public void addVar(DecisionVariable var)
        {
            _point.Add(var);
        }
    }
}
